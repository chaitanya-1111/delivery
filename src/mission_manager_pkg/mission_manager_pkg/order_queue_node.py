#!/usr/bin/env python3
"""
order_queue_node.py

PRODUCTION ROLE:
  Manages the queue of pending delivery orders.
  Acts as a buffer between the order intake sources and the
  mission_manager_node, which can only process ONE order at a time.

  ORDER INTAKE SOURCES:
    1. /order/new (String JSON)       ← from api_bridge_pkg (Step 7: POS system)
    2. /order/manual (String JSON)    ← from operator tablet / RViz panel
    3. /nlu/intent (robot_interfaces/Intent) ← from session_manager (verbal orders)

  QUEUE FEATURES:
    - Priority queue: urgent orders jump the line
    - Deduplication: same order_id never queued twice
    - Persistence: orders survive a node restart (saved to disk)
    - Expiry: orders older than max_age_sec are dropped
    - Status reporting: operators see full queue state

TOPICS SUBSCRIBED:
  /order/new           (std_msgs/String JSON) ← POS system / API
  /order/manual        (std_msgs/String JSON) ← operator input
  /nlu/intent          (robot_interfaces/Intent) ← verbal orders
  /mission/active      (std_msgs/Bool)        ← is robot busy?
  /mission/state       (std_msgs/String JSON) ← full mission state

TOPICS PUBLISHED:
  /mission/new_order   (std_msgs/String JSON) → mission_manager_node
  /order/queue_status  (std_msgs/String JSON) → monitoring UI
  /order/confirmed     (std_msgs/String)      → acknowledge accepted orders
  /robot/speech        (std_msgs/String)      → verbal confirmation of order

SERVICES:
  /order/cancel_order  (std_srvs/Trigger-like) → remove specific order from queue
  /order/clear_queue   (std_srvs/Trigger)       → emergency clear all orders
"""

import json
import os
import time
import heapq
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

try:
    from robot_interfaces.msg import Intent
    HAS_INTENT = True
except ImportError:
    HAS_INTENT = False


@dataclass(order=True)
class QueuedOrder:
    """
    Priority queue entry.
    Python's heapq is a min-heap, so negate priority for max-priority behavior.
    Secondary sort: earlier timestamp = served first (FIFO within priority).
    """
    sort_key  : tuple = field(compare=True)   # (-priority, timestamp)
    order_id  : str   = field(compare=False)
    table     : str   = field(compare=False)
    items     : list  = field(compare=False, default_factory=list)
    priority  : int   = field(compare=False, default=1)
    notes     : str   = field(compare=False, default='')
    timestamp : float = field(compare=False, default_factory=time.time)
    source    : str   = field(compare=False, default='manual')

    @classmethod
    def from_dict(cls, d: dict, source: str = 'api') -> 'QueuedOrder':
        ts = d.get('timestamp', time.time())
        pri = int(d.get('priority', 1))
        return cls(
            sort_key  = (-pri, ts),
            order_id  = str(d.get('order_id', f'ord_{int(ts)}')),
            table     = d.get('table', 'table_1'),
            items     = d.get('items', []),
            priority  = pri,
            notes     = d.get('notes', ''),
            timestamp = ts,
            source    = source,
        )

    def to_dict(self) -> dict:
        return {
            'order_id' : self.order_id,
            'table'    : self.table,
            'items'    : self.items,
            'priority' : self.priority,
            'notes'    : self.notes,
            'timestamp': self.timestamp,
            'source'   : self.source,
        }


class OrderQueueNode(Node):

    def __init__(self):
        super().__init__('order_queue_node')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('max_queue_size',   10)
        self.declare_parameter('max_order_age_sec', 3600.0)   # 1 hour
        self.declare_parameter('dispatch_check_hz',    2.0)
        self.declare_parameter('persist_queue',       True)
        self.declare_parameter('queue_file',
            os.path.expanduser('~/delivery_bot_ws/logs/order_queue.json'))

        self._max_size     = self.get_parameter('max_queue_size').value
        self._max_age      = self.get_parameter('max_order_age_sec').value
        self._dispatch_hz  = self.get_parameter('dispatch_check_hz').value
        self._persist      = self.get_parameter('persist_queue').value
        self._queue_file   = self.get_parameter('queue_file').value

        # ── State ─────────────────────────────────────────────
        self._heap          : List[QueuedOrder] = []   # min-heap by sort_key
        self._seen_ids      : set = set()              # dedup
        self._robot_busy    : bool = False
        self._mission_state : str  = 'IDLE'

        # ── Load persisted queue (survive restarts) ────────────
        if self._persist:
            self._load_queue_from_disk()

        # ── Subscriptions ─────────────────────────────────────
        self._new_order_sub = self.create_subscription(
            String, '/order/new', self._on_new_order, 10)
        self._manual_sub = self.create_subscription(
            String, '/order/manual', self._on_manual_order, 10)
        self._active_sub = self.create_subscription(
            Bool, '/mission/active', self._on_mission_active, 10)
        self._state_sub = self.create_subscription(
            String, '/mission/state', self._on_mission_state, 10)

        # Verbal order intake from session_manager_pkg
        if HAS_INTENT:
            self._intent_sub = self.create_subscription(
                Intent, '/nlu/intent', self._on_intent, 10)

        # ── Publishers ────────────────────────────────────────
        self._dispatch_pub   = self.create_publisher(
            String, '/mission/new_order', 10)
        self._queue_pub      = self.create_publisher(
            String, '/order/queue_status', 10)
        self._confirm_pub    = self.create_publisher(
            String, '/order/confirmed', 10)
        self._speech_pub     = self.create_publisher(
            String, '/robot/speech', 10)

        # ── Services ──────────────────────────────────────────
        self._clear_srv = self.create_service(
            Trigger, '/order/clear_queue', self._handle_clear_queue)

        # ── Timers ────────────────────────────────────────────
        self._dispatch_timer = self.create_timer(
            1.0 / self._dispatch_hz, self._dispatch_tick)
        self._status_timer   = self.create_timer(
            2.0, self._publish_queue_status)
        self._expire_timer   = self.create_timer(
            30.0, self._expire_old_orders)

        self.get_logger().info(
            f'OrderQueueNode started. '
            f'Queue size: {len(self._heap)}/{self._max_size}. '
            f'Max order age: {self._max_age/3600:.1f}h'
        )

    # ──────────────────────────────────────────────────────────
    # ORDER INTAKE
    # ──────────────────────────────────────────────────────────

    def _on_new_order(self, msg: String):
        """Order from POS/API bridge (Step 7)."""
        try:
            data = json.loads(msg.data)
            self._enqueue(QueuedOrder.from_dict(data, source='api'))
        except Exception as e:
            self.get_logger().error(f'Invalid order from /order/new: {e}')

    def _on_manual_order(self, msg: String):
        """Order from operator tablet or RViz panel."""
        try:
            data = json.loads(msg.data)
            self._enqueue(QueuedOrder.from_dict(data, source='manual'))
        except Exception as e:
            self.get_logger().error(f'Invalid manual order: {e}')

    def _on_intent(self, msg):
        """
        Verbal order from session_manager_pkg via NLU pipeline.
        Intent type must be 'order_delivery' or 'request_food'.
        The intent raw_text is parsed for table information.

        Example: "Can you bring food to table 3?"
          → intent_type: 'order_delivery'
          → raw_text: "Can you bring food to table 3?"
          → We extract 'table_3' from the text.
        """
        if msg.intent_type not in ('order_delivery', 'request_food',
                                   'delivery_request'):
            return

        # Extract table number from raw_text
        table = self._extract_table_from_text(msg.raw_text)
        if not table:
            self.get_logger().warn(
                f'Could not extract table from intent: "{msg.raw_text}"')
            return

        order_data = {
            'order_id': f'verbal_{msg.session_id}_{int(time.time())}',
            'table'   : table,
            'items'   : [],   # staff will specify items at kitchen
            'priority': 1,
            'notes'   : f'Verbal order: {msg.raw_text}',
            'source'  : 'verbal',
        }
        self._enqueue(QueuedOrder.from_dict(order_data, source='verbal'))

    def _extract_table_from_text(self, text: str) -> Optional[str]:
        """
        Simple rule-based table number extraction.
        e.g. "table 3", "table three", "number 5" → "table_3", "table_5"
        """
        import re
        text = text.lower()

        # Match "table N" or "table number N"
        m = re.search(r'table\s+(?:number\s+)?(\d+)', text)
        if m:
            return f'table_{m.group(1)}'

        # Word to digit mapping
        words = {'one': 1, 'two': 2, 'three': 3, 'four': 4,
                 'five': 5, 'six': 6, 'seven': 7, 'eight': 8,
                 'nine': 9, 'ten': 10}
        for word, num in words.items():
            if f'table {word}' in text:
                return f'table_{num}'

        return None

    # ──────────────────────────────────────────────────────────
    # QUEUE MANAGEMENT
    # ──────────────────────────────────────────────────────────

    def _enqueue(self, order: QueuedOrder):
        """Add an order to the priority queue with validation."""
        # Deduplication
        if order.order_id in self._seen_ids:
            self.get_logger().warn(
                f'Duplicate order {order.order_id} — ignored.')
            return

        # Queue full check
        if len(self._heap) >= self._max_size:
            self.get_logger().error(
                f'Order queue FULL ({self._max_size} orders). '
                f'Order {order.order_id} DROPPED. '
                f'Increase max_queue_size or process orders faster.')
            return

        heapq.heappush(self._heap, order)
        self._seen_ids.add(order.order_id)

        self.get_logger().info(
            f'📥 Order queued: {order.order_id} → {order.table} '
            f'(priority={order.priority}, source={order.source}). '
            f'Queue depth: {len(self._heap)}'
        )

        # Confirm receipt
        confirm_msg = String()
        confirm_msg.data = json.dumps({
            'order_id': order.order_id,
            'status'  : 'queued',
            'position': len(self._heap),
        })
        self._confirm_pub.publish(confirm_msg)

        # Verbal confirmation if verbal order
        if order.source == 'verbal':
            self._speech_pub.publish(
                String(data=f'I have received an order for {order.table.replace("_", " ")}. '
                            f'I will deliver it as soon as possible.'))

        # Persist to disk
        if self._persist:
            self._save_queue_to_disk()

    def _dispatch_tick(self):
        """
        Called every 500ms. If robot is free and queue has orders,
        dispatch the highest-priority order to mission_manager_node.
        """
        if self._robot_busy:
            return
        if not self._heap:
            return
        if self._mission_state not in ('IDLE',):
            return

        # Pop highest priority order
        order = heapq.heappop(self._heap)

        self.get_logger().info(
            f'📤 Dispatching order {order.order_id} → mission_manager. '
            f'Remaining in queue: {len(self._heap)}'
        )

        msg = String()
        msg.data = json.dumps(order.to_dict())
        self._dispatch_pub.publish(msg)

        if self._persist:
            self._save_queue_to_disk()

    def _expire_old_orders(self):
        """Remove orders that have been waiting too long."""
        now = time.time()
        before = len(self._heap)
        self._heap = [
            o for o in self._heap
            if (now - o.timestamp) < self._max_age
        ]
        heapq.heapify(self._heap)
        expired = before - len(self._heap)
        if expired > 0:
            self.get_logger().warn(
                f'{expired} order(s) expired (older than '
                f'{self._max_age/3600:.1f}h) and removed from queue.')

    # ──────────────────────────────────────────────────────────
    # STATE TRACKING
    # ──────────────────────────────────────────────────────────

    def _on_mission_active(self, msg: Bool):
        self._robot_busy = msg.data

    def _on_mission_state(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._mission_state = data.get('state', 'IDLE')
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────
    # SERVICES + PUBLISHING
    # ──────────────────────────────────────────────────────────

    def _handle_clear_queue(self, request, response):
        count = len(self._heap)
        self._heap = []
        self._seen_ids.clear()
        if self._persist:
            self._save_queue_to_disk()
        response.success = True
        response.message = f'Queue cleared: {count} orders removed.'
        self.get_logger().warn(f'Queue CLEARED: {count} orders removed.')
        return response

    def _publish_queue_status(self):
        orders_summary = [
            {
                'order_id': o.order_id,
                'table'   : o.table,
                'priority': o.priority,
                'age_sec' : round(time.time() - o.timestamp, 0),
                'source'  : o.source,
            }
            for o in sorted(self._heap, key=lambda x: x.sort_key)
        ]
        status = {
            'queue_depth'   : len(self._heap),
            'max_size'      : self._max_size,
            'robot_busy'    : self._robot_busy,
            'mission_state' : self._mission_state,
            'orders'        : orders_summary,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._queue_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    # PERSISTENCE
    # ──────────────────────────────────────────────────────────

    def _save_queue_to_disk(self):
        try:
            os.makedirs(os.path.dirname(self._queue_file), exist_ok=True)
            data = [o.to_dict() for o in self._heap]
            with open(self._queue_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Queue save failed: {e}')

    def _load_queue_from_disk(self):
        if not os.path.exists(self._queue_file):
            return
        try:
            with open(self._queue_file) as f:
                data = json.load(f)
            count = 0
            for d in data:
                order = QueuedOrder.from_dict(d, source=d.get('source', 'persisted'))
                # Only restore if not too old
                if (time.time() - order.timestamp) < self._max_age:
                    heapq.heappush(self._heap, order)
                    self._seen_ids.add(order.order_id)
                    count += 1
            if count:
                self.get_logger().info(
                    f'Restored {count} orders from disk queue.')
        except Exception as e:
            self.get_logger().warn(f'Queue restore failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OrderQueueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
