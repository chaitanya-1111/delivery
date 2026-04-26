#!/usr/bin/env python3
"""
mission_logger_node.py

PRODUCTION ROLE:
  Records every mission event to structured log files on disk.
  This is critical for:
    - Debugging delivery failures ("why did order #47 fail?")
    - Performance analysis ("average delivery time per table")
    - Compliance / audit trail for the restaurant
    - Detecting patterns in failures (always fails at table_5 = obstacle)

  Logs are rotated daily and retained for retention_days.

WHAT IS LOGGED (every event from /mission/event):
  state_change        → FSM transitions with timestamp
  mission_started     → new delivery begun
  arrived_kitchen     → at kitchen, waiting for load
  food_loaded         → staff confirmed food placed
  arrived_table       → at table, waiting for customer
  table_reminder_sent → first reminder sent
  delivery_complete   → customer collected food ✅
  mission_failed      → delivery failed ❌
  mission_cancelled   → operator cancelled
  robot_stuck         → stuck detection triggered
  nav_retry           → navigation retry attempt
  battery_critical_abort → forced dock return
  kitchen_load_timeout   → staff didn't load in time
  pickup_timeout         → customer didn't collect

LOG FORMAT (JSONL — one JSON object per line):
  {"ts": 1701234567.89, "event": "delivery_complete",
   "order_id": "47", "table": "table_3",
   "elapsed_sec": 245.3, ...}

  This format is directly importable into pandas/Excel/Grafana.

TOPICS SUBSCRIBED:
  /mission/event   (std_msgs/String JSON)  ← all events from mission_manager

TOPICS PUBLISHED:
  /mission/log_status (std_msgs/String JSON) ← log health (file, size, errors)
"""

import json
import os
import time
from datetime import datetime, date
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionLoggerNode(Node):

    def __init__(self):
        super().__init__('mission_logger_node')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('log_directory',
            os.path.expanduser('~/delivery_bot_ws/logs/missions'))
        self.declare_parameter('retention_days', 30)
        self.declare_parameter('flush_interval_sec', 5.0)

        self._log_dir        = Path(self.get_parameter('log_directory').value)
        self._retention_days = self.get_parameter('retention_days').value
        self._flush_interval = self.get_parameter('flush_interval_sec').value

        # Ensure log directory exists
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # ── State ─────────────────────────────────────────────
        self._current_log_file = None
        self._current_log_date = None
        self._file_handle      = None
        self._events_logged    = 0
        self._errors           = 0
        self._session_start    = datetime.now().isoformat()

        # Open log file for today
        self._rotate_log_if_needed()

        # ── Subscriptions ─────────────────────────────────────
        self._event_sub = self.create_subscription(
            String, '/mission/event', self._on_event, 10)

        # ── Publishers ────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/mission/log_status', 10)

        # ── Timers ────────────────────────────────────────────
        self._flush_timer  = self.create_timer(
            self._flush_interval, self._flush)
        self._status_timer = self.create_timer(
            30.0, self._publish_status)
        self._rotate_timer = self.create_timer(
            300.0, self._rotate_log_if_needed)   # check every 5 min
        self._cleanup_timer = self.create_timer(
            86400.0, self._cleanup_old_logs)       # daily cleanup

        self.get_logger().info(
            f'MissionLoggerNode started.\n'
            f'  Log directory: {self._log_dir}\n'
            f'  Today\'s log:  {self._current_log_file}\n'
            f'  Retention:     {self._retention_days} days'
        )

    # ──────────────────────────────────────────────────────────
    def _on_event(self, msg: String):
        """Write an event to the log file."""
        try:
            # Parse to validate JSON
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid event JSON: {e}')
            self._errors += 1
            return

        # Ensure log is current (may need rotation at midnight)
        self._rotate_log_if_needed()

        # Enrich with logger metadata
        data['_logged_at'] = time.time()
        data['_logger']    = 'mission_logger_node'

        # Write as JSONL (one JSON object per line)
        try:
            if self._file_handle:
                self._file_handle.write(json.dumps(data) + '\n')
                self._events_logged += 1

                # Log significant events to ROS console too
                event_type = data.get('event', '')
                if event_type in ('delivery_complete', 'mission_failed',
                                  'mission_cancelled', 'mission_started',
                                  'battery_critical_abort'):
                    self.get_logger().info(
                        f'📝 Logged: {event_type} '
                        f'(order={data.get("order_id", "?")})')

        except Exception as e:
            self.get_logger().error(f'Log write failed: {e}')
            self._errors += 1
            # Try to reopen the log file
            self._open_log_file()

    # ──────────────────────────────────────────────────────────
    def _rotate_log_if_needed(self):
        """Open a new log file if the date has changed (midnight rotation)."""
        today = date.today()
        if self._current_log_date == today:
            return  # no rotation needed

        # Close existing file
        self._flush()
        if self._file_handle:
            try:
                self._file_handle.close()
            except Exception:
                pass

        self._current_log_date = today
        filename = f'missions_{today.strftime("%Y_%m_%d")}.jsonl'
        self._current_log_file = self._log_dir / filename
        self._open_log_file()

        # Write session header
        header = {
            'event'        : 'logger_session_start',
            'timestamp'    : time.time(),
            'date'         : today.isoformat(),
            'session_start': self._session_start,
            'log_file'     : str(self._current_log_file),
        }
        if self._file_handle:
            self._file_handle.write(json.dumps(header) + '\n')

        self.get_logger().info(
            f'Log rotated → {self._current_log_file}')

    def _open_log_file(self):
        """Open the current log file in append mode."""
        try:
            self._file_handle = open(
                str(self._current_log_file), 'a', buffering=1)
        except Exception as e:
            self.get_logger().error(f'Cannot open log file: {e}')
            self._file_handle = None
            self._errors += 1

    def _flush(self):
        """Flush buffered writes to disk."""
        if self._file_handle:
            try:
                self._file_handle.flush()
                os.fsync(self._file_handle.fileno())
            except Exception:
                pass

    def _cleanup_old_logs(self):
        """Delete log files older than retention_days."""
        cutoff = time.time() - (self._retention_days * 86400)
        deleted = 0
        for log_file in self._log_dir.glob('missions_*.jsonl'):
            if log_file.stat().st_mtime < cutoff:
                try:
                    log_file.unlink()
                    deleted += 1
                except Exception as e:
                    self.get_logger().warn(f'Could not delete {log_file}: {e}')
        if deleted:
            self.get_logger().info(
                f'Cleaned up {deleted} old log files '
                f'(older than {self._retention_days} days).')

    def _publish_status(self):
        """Publish logger health information."""
        size_bytes = 0
        if self._current_log_file and Path(self._current_log_file).exists():
            size_bytes = Path(self._current_log_file).stat().st_size

        status = {
            'log_file'      : str(self._current_log_file),
            'events_logged' : self._events_logged,
            'errors'        : self._errors,
            'file_size_kb'  : round(size_bytes / 1024, 1),
            'log_date'      : self._current_log_date.isoformat()
                if self._current_log_date else None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def destroy_node(self):
        """Flush and close log file on shutdown."""
        self._flush()
        if self._file_handle:
            # Write session end marker
            end = {
                'event'    : 'logger_session_end',
                'timestamp': time.time(),
                'events_logged': self._events_logged,
            }
            try:
                self._file_handle.write(json.dumps(end) + '\n')
                self._file_handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
