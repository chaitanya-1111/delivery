#!/usr/bin/env python3
"""
mission_manager_node.py

THE DELIVERY BRAIN — production-grade finite state machine (FSM)
that orchestrates every delivery from order receipt to completion.

═══════════════════════════════════════════════════════════════
STATE MACHINE DIAGRAM
═══════════════════════════════════════════════════════════════

  [IDLE] ←──────────────────────────────────────────────────┐
    │ order received                                          │
    ▼                                                         │
  [GOING_TO_KITCHEN]                                          │
    │ nav SUCCEEDED                    nav FAILED (×3) →     │
    ▼                                  [MISSION_FAILED] ──────┤
  [WAITING_FOR_LOAD]                                          │
    │ staff confirms load              timeout → announce,    │
    │   OR timeout (return)                      ─────────────┤
    ▼                                                         │
  [GOING_TO_TABLE]                                            │
    │ nav SUCCEEDED                    nav FAILED (×3) →     │
    ▼                                  [MISSION_FAILED] ──────┤
  [DELIVERING]                                                 │
    │ customer confirms pickup                                │
    │   OR timeout → announce → return with food              │
    ▼                                                         │
  [RETURNING_TO_DOCK]                                         │
    │ nav SUCCEEDED (or failed — we try, not critical)        │
    └──────────────────────────────────────────────────────►[IDLE]

INTERRUPTS (can fire in any state):
  • E-stop received     → [EMERGENCY_STOPPED] → resume on clear
  • Battery critical    → [RETURNING_TO_DOCK] (forced)
  • Order cancelled     → [RETURNING_TO_DOCK]
  • Navigation STUCK    → retry or [MISSION_FAILED]

═══════════════════════════════════════════════════════════════

TOPICS SUBSCRIBED:
  /mission/new_order      (std_msgs/String JSON)  ← from order_queue_node
  /mission/cancel         (std_msgs/Bool)         ← operator cancel
  /navigation/status      (std_msgs/String)       ← from nav_client_node
  /navigation/result      (std_msgs/String JSON)  ← nav outcome
  /navigation/health      (std_msgs/String)       ← from nav_monitor_node
  /navigation/stuck       (std_msgs/Bool)         ← stuck detection
  /mission/load_confirm   (std_msgs/Bool)         ← kitchen staff press button
  /mission/pickup_confirm (std_msgs/Bool)         ← customer press button
  /safety/estop           (std_msgs/Bool)         ← from safety_supervisor (Step 5)
  /battery/critical       (std_msgs/Bool)         ← from battery_manager (Step 6)

TOPICS PUBLISHED:
  /navigation/go_to       (std_msgs/String)       → location_server_node
  /navigation/cancel      (std_msgs/Bool)         → nav_client_node
  /robot/speech           (std_msgs/String)       → tts_player_pkg
  /mission/state          (std_msgs/String JSON)  → monitoring / UI
  /mission/active         (std_msgs/Bool)         → is a delivery in progress?
  /mission/stats          (std_msgs/String JSON)  → session totals
  /mission/event          (std_msgs/String JSON)  → mission_logger_node
"""

import json
import math
import os
import time
from enum import Enum, auto
from dataclasses import dataclass, field, asdict
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml

from std_msgs.msg import String, Bool


# ── State Definitions ────────────────────────────────────────
class MissionState(Enum):
    IDLE                = auto()
    GOING_TO_KITCHEN    = auto()
    WAITING_FOR_LOAD    = auto()
    GOING_TO_TABLE      = auto()
    DELIVERING          = auto()
    WAITING_FOR_PICKUP  = auto()
    RETURNING_TO_DOCK   = auto()
    MISSION_FAILED      = auto()
    EMERGENCY_STOPPED   = auto()
    PAUSED              = auto()


# ── Order Data Class ─────────────────────────────────────────
@dataclass
class DeliveryOrder:
    order_id:    str
    table:       str           # location name, e.g. "table_3"
    items:       list          # list of item descriptions
    priority:    int  = 1
    timestamp:   float = field(default_factory=time.time)
    notes:       str  = ''

    def to_dict(self):
        return asdict(self)


# ══════════════════════════════════════════════════════════════
class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        # ── Load config ───────────────────────────────────────
        self._cfg = self._load_config()
        t = self._cfg.get('mission', {}).get('timeouts', {})
        b = self._cfg.get('mission', {}).get('behavior', {})
        s = self._cfg.get('mission', {}).get('speech', {})

        self._kitchen_load_timeout  = t.get('kitchen_load_sec',       60.0)
        self._pickup_timeout        = t.get('customer_pickup_sec',     90.0)
        self._nav_timeout           = t.get('navigation_timeout_sec', 180.0)
        self._first_reminder_sec    = t.get('first_reminder_sec',      30.0)
        self._stuck_escalation      = t.get('stuck_escalation_sec',    25.0)
        self._max_nav_retries       = self._cfg.get('mission', {}) \
                                          .get('retries', {}) \
                                          .get('navigation_max_retries', 2)
        self._return_when_idle      = b.get('return_to_dock_when_idle', True)
        self._preempt_dock          = b.get('preempt_dock_return', True)
        self._pickup_timeout_action = b.get('pickup_timeout_action', 'return')
        self._speech                = s

        # ── Callback group ────────────────────────────────────
        self._cbg = ReentrantCallbackGroup()

        # ── Subscriptions ─────────────────────────────────────
        self._new_order_sub = self.create_subscription(
            String, '/mission/new_order', self._on_new_order, 10,
            callback_group=self._cbg)
        self._cancel_sub = self.create_subscription(
            Bool, '/mission/cancel', self._on_cancel, 10,
            callback_group=self._cbg)
        self._nav_status_sub = self.create_subscription(
            String, '/navigation/status', self._on_nav_status, 10,
            callback_group=self._cbg)
        self._nav_result_sub = self.create_subscription(
            String, '/navigation/result', self._on_nav_result, 10,
            callback_group=self._cbg)
        self._nav_health_sub = self.create_subscription(
            String, '/navigation/health', self._on_nav_health, 10,
            callback_group=self._cbg)
        self._stuck_sub = self.create_subscription(
            Bool, '/navigation/stuck', self._on_stuck, 10,
            callback_group=self._cbg)
        self._load_confirm_sub = self.create_subscription(
            Bool, '/mission/load_confirm', self._on_load_confirm, 10,
            callback_group=self._cbg)
        self._pickup_confirm_sub = self.create_subscription(
            Bool, '/mission/pickup_confirm', self._on_pickup_confirm, 10,
            callback_group=self._cbg)
        self._estop_sub = self.create_subscription(
            Bool, '/safety/estop', self._on_estop, 10,
            callback_group=self._cbg)
        self._battery_sub = self.create_subscription(
            Bool, '/battery/critical', self._on_battery_critical, 10,
            callback_group=self._cbg)

        # ── Publishers ────────────────────────────────────────
        self._go_to_pub      = self.create_publisher(String, '/navigation/go_to',  10)
        self._nav_cancel_pub = self.create_publisher(Bool,   '/navigation/cancel',  10)
        self._speech_pub     = self.create_publisher(String, '/robot/speech',       10)
        self._state_pub      = self.create_publisher(String, '/mission/state',      10)
        self._active_pub     = self.create_publisher(Bool,   '/mission/active',     10)
        self._stats_pub      = self.create_publisher(String, '/mission/stats',      10)
        self._event_pub      = self.create_publisher(String, '/mission/event',      10)

        # ── FSM state ─────────────────────────────────────────
        self._state         : MissionState = MissionState.IDLE
        self._prev_state    : Optional[MissionState] = None  # for e-stop resume
        self._current_order : Optional[DeliveryOrder] = None
        self._nav_status    : str  = 'IDLE'
        self._nav_health    : str  = 'HEALTHY'
        self._is_stuck      : bool = False
        self._estop_active  : bool = False

        # Navigation tracking
        self._nav_retries        : int   = 0
        self._nav_goal_sent_time : Optional[float] = None
        self._current_nav_target : str   = ''

        # Waiting-state tracking
        self._state_entered_time : float = time.time()
        self._reminder_sent      : bool  = False

        # Session statistics
        self._session_start     = time.time()
        self._total_deliveries  = 0
        self._failed_deliveries = 0
        self._total_nav_meters  = 0.0

        # ── Timers ────────────────────────────────────────────
        # Main FSM tick: 2Hz is sufficient for state logic
        self._fsm_timer     = self.create_timer(0.5, self._fsm_tick,
                                                callback_group=self._cbg)
        # Status publish: 1Hz
        self._status_timer  = self.create_timer(1.0, self._publish_status,
                                                callback_group=self._cbg)

        self.get_logger().info(
            '╔══════════════════════════════════╗\n'
            '║  Mission Manager Node STARTED    ║\n'
            '╠══════════════════════════════════╣\n'
            f'║  Kitchen load timeout: {self._kitchen_load_timeout:.0f}s     \n'
            f'║  Pickup timeout:       {self._pickup_timeout:.0f}s     \n'
            f'║  Nav timeout:          {self._nav_timeout:.0f}s    \n'
            f'║  Max nav retries:      {self._max_nav_retries}          \n'
            '╠══════════════════════════════════╣\n'
            '║  Waiting for orders on           ║\n'
            '║  /mission/new_order (JSON)       ║\n'
            '╚══════════════════════════════════╝'
        )

    # ══════════════════════════════════════════════════════════
    # MAIN FSM TICK  (runs every 500ms)
    # ══════════════════════════════════════════════════════════
    def _fsm_tick(self):
        """
        The heart of the mission manager.
        Called every 500ms to evaluate state transitions and timeouts.
        Each state checks:
          1. Was a navigation result received? → transition
          2. Has a timeout expired? → action or transition
          3. Was a confirmation received? → transition
        """
        now = time.time()
        elapsed_in_state = now - self._state_entered_time

        # ── E-STOP overrides everything ───────────────────────
        if self._estop_active and self._state != MissionState.EMERGENCY_STOPPED:
            self._transition(MissionState.EMERGENCY_STOPPED,
                             reason='e-stop received')
            return

        # ── State handlers ────────────────────────────────────
        if self._state == MissionState.IDLE:
            self._handle_idle()

        elif self._state == MissionState.GOING_TO_KITCHEN:
            self._handle_going_to_kitchen(elapsed_in_state)

        elif self._state == MissionState.WAITING_FOR_LOAD:
            self._handle_waiting_for_load(elapsed_in_state)

        elif self._state == MissionState.GOING_TO_TABLE:
            self._handle_going_to_table(elapsed_in_state)

        elif self._state == MissionState.DELIVERING:
            self._handle_delivering(elapsed_in_state)

        elif self._state == MissionState.RETURNING_TO_DOCK:
            self._handle_returning_to_dock(elapsed_in_state)

        elif self._state == MissionState.MISSION_FAILED:
            self._handle_mission_failed()

        elif self._state == MissionState.EMERGENCY_STOPPED:
            # Wait until e-stop is cleared
            pass

    # ──────────────────────────────────────────────────────────
    # STATE HANDLERS
    # ──────────────────────────────────────────────────────────

    def _handle_idle(self):
        """
        IDLE: No active delivery.
        Waits for an order from order_queue_node.
        If return_to_dock_when_idle is true, robot stays at dock.
        """
        pass  # Order intake via _on_new_order callback

    def _handle_going_to_kitchen(self, elapsed: float):
        """
        GOING_TO_KITCHEN: Robot is navigating to the food pickup area.

        Transitions:
          nav SUCCEEDED → WAITING_FOR_LOAD
          nav FAILED    → retry or MISSION_FAILED
          timeout       → cancel + MISSION_FAILED
        """
        # Navigation timeout guard
        if elapsed > self._nav_timeout:
            self.get_logger().error(
                f'Navigation to kitchen TIMED OUT after {elapsed:.0f}s!')
            self._cancel_navigation()
            self._mission_failed('kitchen_nav_timeout')
            return

        # Wait for nav result (handled in _on_nav_result callback)

    def _handle_waiting_for_load(self, elapsed: float):
        """
        WAITING_FOR_LOAD: Robot is at kitchen, waiting for staff to
        place food on the tray.

        Staff confirms via physical button → /mission/load_confirm
        OR timer expires → give up and return

        Transitions:
          load_confirm=True  → GOING_TO_TABLE
          elapsed > first_reminder → speak reminder
          elapsed > timeout  → RETURNING_TO_DOCK (nothing to deliver)
        """
        # First reminder
        if (elapsed > self._first_reminder_sec and
                not self._reminder_sent):
            self._speak(self._speech.get(
                'kitchen_reminder',
                'Still waiting for the order. Please place it on my tray.'))
            self._reminder_sent = True

        # Timeout
        if elapsed > self._kitchen_load_timeout:
            self.get_logger().warn(
                f'Kitchen load TIMED OUT after {elapsed:.0f}s. '
                'No food loaded. Returning to dock.')
            self._speak(self._speech.get(
                'kitchen_timeout',
                'No order received. Returning to dock.'))
            self._emit_event('kitchen_load_timeout', {
                'order_id': self._current_order.order_id
                    if self._current_order else 'unknown'
            })
            # Mark mission as failed — nothing to deliver
            self._mission_failed('kitchen_load_timeout')

    def _handle_going_to_table(self, elapsed: float):
        """
        GOING_TO_TABLE: Robot is navigating to the delivery table.

        Transitions:
          nav SUCCEEDED → DELIVERING
          nav FAILED    → retry up to max_nav_retries, then MISSION_FAILED
          timeout       → MISSION_FAILED
        """
        if elapsed > self._nav_timeout:
            self.get_logger().error(
                f'Navigation to table TIMED OUT after {elapsed:.0f}s!')
            self._cancel_navigation()
            self._mission_failed('table_nav_timeout')

    def _handle_delivering(self, elapsed: float):
        """
        DELIVERING: Robot has arrived at the table.
        Announces arrival, waits for customer to collect food.

        Customer confirms via button → /mission/pickup_confirm
        OR timer expires → action based on config

        Transitions:
          pickup_confirm=True         → RETURNING_TO_DOCK (success!)
          elapsed > first_reminder   → speak reminder
          elapsed > pickup_timeout   → action: return or wait or alert
        """
        # First reminder
        if (elapsed > self._first_reminder_sec and
                not self._reminder_sent):
            self._speak(self._speech.get(
                'table_reminder',
                'Reminder: your food is ready. Please collect it.'))
            self._reminder_sent = True
            self._emit_event('table_reminder_sent', {})

        # Pickup timeout
        if elapsed > self._pickup_timeout:
            action = self._pickup_timeout_action
            self.get_logger().warn(
                f'Customer pickup TIMED OUT ({elapsed:.0f}s). '
                f'Action: {action}')
            self._emit_event('pickup_timeout', {'action': action})

            if action == 'return':
                self._speak(self._speech.get(
                    'table_timeout_return',
                    'Returning the order. Sorry for the inconvenience.'))
                self._mission_failed('pickup_timeout')

            elif action == 'alert_staff':
                # Alert but don't transition — staff will handle it
                self._speak('Please ask a staff member to assist table '
                            + (self._current_order.table if self._current_order else ''))
                # Extend the wait (re-arm reminder)
                self._state_entered_time = time.time()
                self._reminder_sent = False

            elif action == 'wait_longer':
                # Keep waiting — mission_config should set a reasonable timeout
                self._state_entered_time = time.time()
                self._reminder_sent = False

    def _handle_returning_to_dock(self, elapsed: float):
        """
        RETURNING_TO_DOCK: Mission complete or failed.
        Robot returns to charging dock.
        This is non-critical: even if navigation fails, we reset to IDLE.
        """
        if elapsed > self._nav_timeout:
            # Could not return to dock — reset anyway to accept new orders
            self.get_logger().warn(
                'Could not return to dock within timeout. Resetting to IDLE.')
            self._transition(MissionState.IDLE, reason='dock_return_timeout')

    def _handle_mission_failed(self):
        """
        MISSION_FAILED: Announce failure and return to dock.
        Transitions automatically to RETURNING_TO_DOCK.
        """
        self._failed_deliveries += 1
        self._emit_event('mission_failed', {
            'order': self._current_order.to_dict()
                if self._current_order else {}
        })
        # Clear order and go home
        self._current_order = None
        self._navigate_to('dock')
        self._transition(MissionState.RETURNING_TO_DOCK,
                         reason='mission_failed_returning')

    # ──────────────────────────────────────────────────────────
    # INCOMING EVENT HANDLERS
    # ──────────────────────────────────────────────────────────

    def _on_new_order(self, msg: String):
        """
        New delivery order received from order_queue_node.
        JSON format: {"order_id": "47", "table": "table_3",
                       "items": ["burger", "fries"], "priority": 1}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid order JSON: {e}')
            return

        order = DeliveryOrder(
            order_id = str(data.get('order_id', f'ord_{int(time.time())}')),
            table    = data.get('table', 'table_1'),
            items    = data.get('items', []),
            priority = int(data.get('priority', 1)),
            notes    = data.get('notes', ''),
        )

        self.get_logger().info(
            f'📦 New order received!\n'
            f'  Order ID : {order.order_id}\n'
            f'  Table    : {order.table}\n'
            f'  Items    : {", ".join(order.items)}\n'
            f'  Priority : {order.priority}\n'
            f'  Current state: {self._state.name}'
        )

        # ── Accept order based on current state ───────────────
        if self._state == MissionState.IDLE:
            self._start_delivery(order)

        elif (self._state == MissionState.RETURNING_TO_DOCK and
              self._preempt_dock):
            # Robot is returning but hasn't docked yet — reroute!
            self.get_logger().info(
                'Preempting dock return for new order!')
            self._cancel_navigation()
            self._start_delivery(order)

        else:
            self.get_logger().warn(
                f'Robot busy ({self._state.name}). '
                f'Order {order.order_id} should be queued '
                f'by order_queue_node and re-sent when IDLE.')
            # order_queue_node handles the queue — we just reject here

    def _on_cancel(self, msg: Bool):
        """Operator cancels the current delivery."""
        if not msg.data:
            return
        if self._state == MissionState.IDLE:
            self.get_logger().info('Cancel received but nothing active.')
            return
        self.get_logger().warn(
            f'Mission CANCELLED by operator! '
            f'(was in {self._state.name})')
        self._speak(self._speech.get('mission_cancelled',
                                     'Delivery cancelled. Returning to dock.'))
        self._cancel_navigation()
        self._emit_event('mission_cancelled', {
            'order': self._current_order.to_dict()
                if self._current_order else {},
            'cancelled_in_state': self._state.name
        })
        self._current_order = None
        self._navigate_to('dock')
        self._transition(MissionState.RETURNING_TO_DOCK,
                         reason='operator_cancel')

    def _on_nav_status(self, msg: String):
        self._nav_status = msg.data

    def _on_nav_result(self, msg: String):
        """
        Navigation completed — determine what to do next based on
        current FSM state and the outcome.
        """
        try:
            result = json.loads(msg.data)
        except Exception:
            return

        outcome = result.get('outcome', 'FAILED')
        reason  = result.get('reason', '')

        self.get_logger().info(
            f'Navigation result: {outcome} (reason={reason}) '
            f'in state {self._state.name}')

        # ── GOING_TO_KITCHEN ──────────────────────────────────
        if self._state == MissionState.GOING_TO_KITCHEN:
            if outcome == 'SUCCEEDED':
                self._nav_retries = 0
                self._speak(self._speech.get(
                    'kitchen_arrival',
                    'I am at the kitchen. Please place the order on my tray.'))
                self._emit_event('arrived_kitchen', {})
                self._transition(MissionState.WAITING_FOR_LOAD,
                                 reason='kitchen_arrived')
            else:
                self._handle_nav_failure('kitchen', outcome)

        # ── GOING_TO_TABLE ────────────────────────────────────
        elif self._state == MissionState.GOING_TO_TABLE:
            if outcome == 'SUCCEEDED':
                self._nav_retries = 0
                self._speak(self._speech.get(
                    'table_arrival',
                    'Hello! Your order has arrived. Please take your food.'))
                self._emit_event('arrived_table', {
                    'table': self._current_order.table
                        if self._current_order else ''
                })
                self._transition(MissionState.DELIVERING,
                                 reason='table_arrived')
            else:
                self._handle_nav_failure('table', outcome)

        # ── RETURNING_TO_DOCK ─────────────────────────────────
        elif self._state == MissionState.RETURNING_TO_DOCK:
            if outcome == 'SUCCEEDED':
                self.get_logger().info('✅ Returned to dock successfully.')
                self._emit_event('returned_to_dock', {})
            else:
                self.get_logger().warn(
                    'Could not return to dock. Resetting to IDLE anyway.')
            # Either way — reset to IDLE
            self._current_order = None
            self._transition(MissionState.IDLE, reason='dock_reached')

    def _handle_nav_failure(self, target_name: str, outcome: str):
        """Shared navigation failure handler with retry logic."""
        self._nav_retries += 1
        self.get_logger().error(
            f'Navigation to {target_name} FAILED '
            f'(attempt {self._nav_retries}/{self._max_nav_retries}). '
            f'Outcome: {outcome}')

        if self._nav_retries <= self._max_nav_retries:
            # Retry
            self.get_logger().info(
                f'Retrying navigation to {target_name}...')
            self._emit_event('nav_retry', {
                'target': target_name,
                'attempt': self._nav_retries
            })
            self._navigate_to(self._current_nav_target)
        else:
            # Give up
            self.get_logger().error(
                f'Navigation to {target_name} failed {self._nav_retries} times. '
                'Declaring MISSION_FAILED.')
            self._emit_event('nav_max_retries_exceeded', {'target': target_name})
            self._speak(self._speech.get(
                'robot_stuck',
                'I am unable to complete this delivery. Please check my path.'))
            self._mission_failed(f'{target_name}_nav_failed')

    def _on_nav_health(self, msg: String):
        self._nav_health = msg.data
        if msg.data == 'CRITICAL' and self._state not in (
                MissionState.IDLE, MissionState.EMERGENCY_STOPPED):
            self.get_logger().error(
                'Navigation health CRITICAL! Pausing mission.')

    def _on_stuck(self, msg: Bool):
        if msg.data and not self._is_stuck:
            self.get_logger().error(
                f'Robot STUCK detected in state {self._state.name}!')
            self._speak(self._speech.get(
                'robot_stuck',
                'I appear to be stuck. Please check my path.'))
            self._emit_event('robot_stuck', {'state': self._state.name})
        self._is_stuck = msg.data

    def _on_load_confirm(self, msg: Bool):
        """Kitchen staff presses button confirming food is loaded."""
        if not msg.data:
            return
        if self._state != MissionState.WAITING_FOR_LOAD:
            self.get_logger().warn(
                f'Load confirm received in unexpected state: {self._state.name}')
            return

        self.get_logger().info('✅ Food load confirmed by kitchen staff!')
        self._emit_event('food_loaded', {
            'order_id': self._current_order.order_id
                if self._current_order else 'unknown'
        })

        # Now navigate to the delivery table
        if self._current_order:
            self._navigate_to(self._current_order.table)
            self._transition(MissionState.GOING_TO_TABLE,
                             reason='food_loaded')
        else:
            self.get_logger().error(
                'Load confirmed but no active order! Returning to dock.')
            self._navigate_to('dock')
            self._transition(MissionState.RETURNING_TO_DOCK,
                             reason='no_order_after_load')

    def _on_pickup_confirm(self, msg: Bool):
        """Customer presses button confirming food collected."""
        if not msg.data:
            return
        if self._state != MissionState.DELIVERING:
            return

        self.get_logger().info('✅ Pickup confirmed by customer!')
        self._speak(self._speech.get('mission_complete', 'Enjoy your meal!'))
        self._total_deliveries += 1
        self._emit_event('delivery_complete', {
            'order_id': self._current_order.order_id
                if self._current_order else 'unknown',
            'table': self._current_order.table
                if self._current_order else 'unknown',
        })
        # Return to dock
        self._current_order = None
        self._navigate_to('dock')
        self._transition(MissionState.RETURNING_TO_DOCK,
                         reason='pickup_confirmed')

    def _on_estop(self, msg: Bool):
        """Emergency stop signal from safety_supervisor (Step 5)."""
        if msg.data:
            if self._state != MissionState.EMERGENCY_STOPPED:
                self.get_logger().error('🚨 E-STOP RECEIVED!')
                self._prev_state = self._state
                self._cancel_navigation()
                self._transition(MissionState.EMERGENCY_STOPPED,
                                 reason='estop_active')
            self._estop_active = True
        else:
            if self._state == MissionState.EMERGENCY_STOPPED:
                self.get_logger().info('E-stop cleared. Resuming...')
                self._emit_event('estop_cleared', {})
                self._estop_active = False
                # Resume: re-enter IDLE so next FSM tick picks up
                self._transition(MissionState.IDLE,
                                 reason='estop_cleared')

    def _on_battery_critical(self, msg: Bool):
        """Battery manager signals critically low battery."""
        if not msg.data:
            return
        if self._state in (MissionState.IDLE,
                           MissionState.RETURNING_TO_DOCK,
                           MissionState.EMERGENCY_STOPPED):
            return

        self.get_logger().error(
            '🔋 BATTERY CRITICAL! Abandoning mission to charge.')
        self._speak('Battery is critically low. Returning to charging dock.')
        self._emit_event('battery_critical_abort', {
            'state': self._state.name,
            'order': self._current_order.to_dict()
                if self._current_order else {}
        })
        self._cancel_navigation()
        self._current_order = None
        self._navigate_to('dock')
        self._transition(MissionState.RETURNING_TO_DOCK,
                         reason='battery_critical')

    # ──────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────

    def _start_delivery(self, order: DeliveryOrder):
        """Begin a new delivery mission."""
        self.get_logger().info(
            f'🚀 Starting delivery: order={order.order_id}, '
            f'table={order.table}')
        self._current_order   = order
        self._nav_retries     = 0
        self._emit_event('mission_started', {'order': order.to_dict()})

        # Head to kitchen first
        self._navigate_to('kitchen')
        self._transition(MissionState.GOING_TO_KITCHEN,
                         reason='new_order')

    def _navigate_to(self, location: str):
        """Send a navigation goal to location_server_node."""
        self._current_nav_target = location
        self._nav_goal_sent_time = time.time()
        self._nav_retries       = 0 if location != self._current_nav_target else self._nav_retries

        msg = String()
        msg.data = location
        self._go_to_pub.publish(msg)
        self.get_logger().info(f'Navigation goal sent: → {location}')

    def _cancel_navigation(self):
        """Cancel any in-flight navigation goal."""
        msg = Bool()
        msg.data = True
        self._nav_cancel_pub.publish(msg)

    def _speak(self, text: str):
        """Publish speech text to TTS system."""
        msg = String()
        msg.data = text
        self._speech_pub.publish(msg)
        self.get_logger().info(f'🔊 Speech: "{text}"')

    def _mission_failed(self, reason: str):
        """Transition to MISSION_FAILED state with reason."""
        self.get_logger().error(f'Mission FAILED: {reason}')
        self._transition(MissionState.MISSION_FAILED, reason=reason)

    def _transition(self, new_state: MissionState, reason: str = ''):
        """
        Perform a state transition.
        Logs the transition, resets per-state variables,
        and emits a state_change event.
        """
        old_state = self._state
        self._state              = new_state
        self._state_entered_time = time.time()
        self._reminder_sent      = False

        self.get_logger().info(
            f'State: {old_state.name} → {new_state.name}'
            + (f' ({reason})' if reason else ''))

        self._emit_event('state_change', {
            'from'  : old_state.name,
            'to'    : new_state.name,
            'reason': reason,
        })

    def _emit_event(self, event_type: str, data: dict):
        """Publish a structured event for the mission logger."""
        event = {
            'event'    : event_type,
            'timestamp': time.time(),
            'state'    : self._state.name,
            'order_id' : self._current_order.order_id
                if self._current_order else None,
            **data
        }
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    # STATUS PUBLISHER  (1Hz)
    # ──────────────────────────────────────────────────────────

    def _publish_status(self):
        # Active flag
        active_msg = Bool()
        active_msg.data = self._state not in (
            MissionState.IDLE, MissionState.EMERGENCY_STOPPED)
        self._active_pub.publish(active_msg)

        # Full state JSON
        order_dict = self._current_order.to_dict() \
            if self._current_order else None
        state_data = {
            'state'          : self._state.name,
            'nav_status'     : self._nav_status,
            'nav_health'     : self._nav_health,
            'active_order'   : order_dict,
            'nav_target'     : self._current_nav_target,
            'nav_retries'    : self._nav_retries,
            'is_stuck'       : self._is_stuck,
            'estop'          : self._estop_active,
            'elapsed_in_state':
                round(time.time() - self._state_entered_time, 1),
        }
        state_msg = String()
        state_msg.data = json.dumps(state_data)
        self._state_pub.publish(state_msg)

        # Session stats
        uptime = time.time() - self._session_start
        stats = {
            'total_deliveries'  : self._total_deliveries,
            'failed_deliveries' : self._failed_deliveries,
            'success_rate_pct'  : round(
                100.0 * self._total_deliveries /
                max(1, self._total_deliveries + self._failed_deliveries), 1),
            'uptime_hours'      : round(uptime / 3600.0, 2),
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self._stats_pub.publish(stats_msg)

    # ──────────────────────────────────────────────────────────
    def _load_config(self) -> dict:
        """Load mission_config.yaml from the package share directory."""
        search_paths = [
            os.path.join(os.path.expanduser('~'), 'delivery_bot_ws',
                         'install', 'mission_manager_pkg', 'share',
                         'mission_manager_pkg', 'config', 'mission_config.yaml'),
            os.path.join(os.path.dirname(__file__),
                         '..', 'config', 'mission_config.yaml'),
        ]
        for p in search_paths:
            p = os.path.normpath(p)
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        cfg = yaml.safe_load(f)
                    self.get_logger().info(f'Loaded config: {p}')
                    return cfg or {}
                except Exception as e:
                    self.get_logger().warn(f'Config parse error: {e}')
        self.get_logger().warn('mission_config.yaml not found — using defaults.')
        return {}


# ── Import yaml lazily ────────────────────────────────────────
try:
    import yaml
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pyyaml'])
    import yaml


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
