#!/usr/bin/env python3
"""
scan_watchdog_node.py

PRODUCTION ROLE:
  In a restaurant environment, the delivery robot runs 8-14 hours per day.
  USB connections can be intermittently lost due to vibration, power issues,
  or the lidar driver crashing. This watchdog:

  1. Monitors /scan_raw for continuous data
  2. If scan stops → attempts USB/driver recovery via subprocess
  3. Publishes /lidar/watchdog_status for operators
  4. Logs all events with timestamps for maintenance records

  RECOVERY STRATEGY:
    Level 1 (scan silent > 3s):  Log warning, wait
    Level 2 (scan silent > 8s):  Try to reconnect via driver restart signal
    Level 3 (scan silent > 30s): Full recovery — disable/re-enable USB device
    Level 4 (scan silent > 60s): Escalate — publish CRITICAL alert

  WHY NOT JUST RESTART THE DRIVER NODE:
    In production, we use systemd to manage rplidar_ros. The watchdog
    sends a signal to systemd to restart the service, rather than trying
    to kill/restart the ROS node itself (which can leave TF and topic
    state in an inconsistent state).

TOPICS SUBSCRIBED:
  /scan_raw  (sensor_msgs/LaserScan)

TOPICS PUBLISHED:
  /lidar/watchdog_status  (std_msgs/String)  — OK / WARNING / RECOVERING / CRITICAL
  /lidar/recovery_events  (std_msgs/String)  — JSON event log entry

PARAMETERS:
  warn_timeout_sec     : seconds of silence before WARN
  recovery_timeout_sec : seconds of silence before attempting recovery
  critical_timeout_sec : seconds of silence before CRITICAL alert
  enable_auto_recovery : bool — enable automatic USB/driver recovery
"""

import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanWatchdogNode(Node):

    def __init__(self):
        super().__init__('scan_watchdog_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('warn_timeout_sec',     3.0)
        self.declare_parameter('recovery_timeout_sec', 8.0)
        self.declare_parameter('critical_timeout_sec', 30.0)
        self.declare_parameter('enable_auto_recovery', True)
        self.declare_parameter('lidar_service_name', 'rplidar.service')
        self.declare_parameter('lidar_device_path',  '/dev/rplidar')

        self._warn_t     = self.get_parameter('warn_timeout_sec').value
        self._recover_t  = self.get_parameter('recovery_timeout_sec').value
        self._critical_t = self.get_parameter('critical_timeout_sec').value
        self._auto_rec   = self.get_parameter('enable_auto_recovery').value
        self._svc_name   = self.get_parameter('lidar_service_name').value
        self._dev_path   = self.get_parameter('lidar_device_path').value

        # ── QoS ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, sensor_qos)

        # ── Publishers ────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/lidar/watchdog_status', 10)
        self._event_pub  = self.create_publisher(String, '/lidar/recovery_events', 10)

        # ── State ─────────────────────────────────────────────────
        self._last_scan_time: float = None
        self._state: str = 'WAITING'      # WAITING, OK, WARNING, RECOVERING, CRITICAL
        self._recovery_attempts: int = 0
        self._recovery_in_progress: bool = False
        self._startup_time: float = time.monotonic()
        self._total_scans: int = 0

        # Grace period on startup: lidar takes ~3s to spin up
        self._startup_grace_sec = 10.0

        # ── Watchdog timer: check every 500ms ─────────────────────
        self._watch_timer = self.create_timer(0.5, self._watchdog_tick)

        self.get_logger().info(
            f'ScanWatchdog started. '
            f'warn={self._warn_t}s, recover={self._recover_t}s, '
            f'critical={self._critical_t}s. '
            f'Auto-recovery: {"ENABLED" if self._auto_rec else "DISABLED"}'
        )

    # ── Scan Callback ─────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        """Update last-seen timestamp every time a scan arrives."""
        now = time.monotonic()
        self._last_scan_time = now
        self._total_scans += 1

        # If we were recovering and scan came back — recovery succeeded
        if self._state in ('WARNING', 'RECOVERING'):
            self.get_logger().info(
                f'Lidar RECOVERED after {self._recovery_attempts} attempt(s). '
                f'Scan is back!'
            )
            self._log_event('RECOVERY_SUCCESS', {
                'recovery_attempts': self._recovery_attempts,
                'total_scans': self._total_scans,
            })
            self._recovery_in_progress = False

        if self._total_scans == 1:
            self.get_logger().info('First scan received! RPLidar is working.')
            self._log_event('LIDAR_ONLINE', {'device': self._dev_path})

        self._state = 'OK'

    # ── Watchdog Tick (every 500ms) ───────────────────────────────
    def _watchdog_tick(self):
        """
        Main watchdog logic — runs every 500ms.
        Checks scan staleness and takes action.
        """
        now = time.monotonic()

        # ── Still in startup grace period ─────────────────────────
        if (self._last_scan_time is None and
                (now - self._startup_time) < self._startup_grace_sec):
            self._state = 'WAITING'
            self._publish_status('WAITING')
            return

        # ── First scan never came ──────────────────────────────────
        if self._last_scan_time is None:
            elapsed = now - self._startup_time
            self._state = 'CRITICAL'
            self.get_logger().error(
                f'RPLidar NEVER produced a scan after {elapsed:.1f}s! '
                f'Check:\n'
                f'  1. USB cable connected?\n'
                f'  2. ls /dev/ttyUSB*  (should show /dev/ttyUSB0)\n'
                f'  3. ls /dev/rplidar  (udev rule applied?)\n'
                f'  4. systemctl status rplidar.service\n'
                f'  5. ros2 node list  (is rplidar_node running?)'
            )
            self._publish_status('CRITICAL')
            return

        # ── Compute silence duration ──────────────────────────────
        silence = now - self._last_scan_time

        if silence < self._warn_t:
            # ── Normal ───────────────────────────────────────────
            self._state = 'OK'
            self._recovery_in_progress = False
            self._publish_status('OK')

        elif silence < self._recover_t:
            # ── Warning zone ──────────────────────────────────────
            if self._state != 'WARNING':
                self.get_logger().warn(
                    f'Lidar scan silent for {silence:.1f}s! '
                    f'USB issue or driver crash? '
                    f'Recovery will trigger at {self._recover_t}s.'
                )
                self._log_event('SCAN_SILENT_WARN', {'silence_sec': round(silence, 1)})
            self._state = 'WARNING'
            self._publish_status('WARNING')

        elif silence < self._critical_t:
            # ── Recovery zone ─────────────────────────────────────
            if not self._recovery_in_progress:
                self._attempt_recovery(silence)
            self._publish_status('RECOVERING')

        else:
            # ── Critical ─────────────────────────────────────────
            if self._state != 'CRITICAL':
                self.get_logger().error(
                    f'LIDAR CRITICAL: scan silent for {silence:.1f}s! '
                    f'Auto-recovery failed. Manual intervention required.\n'
                    f'Try: sudo systemctl restart {self._svc_name}'
                )
                self._log_event('LIDAR_CRITICAL', {
                    'silence_sec': round(silence, 1),
                    'recovery_attempts': self._recovery_attempts,
                })
            self._state = 'CRITICAL'
            self._publish_status('CRITICAL')

    # ── Recovery Logic ────────────────────────────────────────────
    def _attempt_recovery(self, silence_sec: float):
        """
        Attempt to recover the lidar after a scan dropout.

        Recovery strategy:
          1. First attempt: try to restart the systemd service
          2. Second attempt: USB device power cycle via uhubctl (if available)
          3. Further: just log and wait for manual intervention
        """
        if not self._auto_rec:
            self.get_logger().warn(
                'Auto-recovery disabled. Manual restart required: '
                f'sudo systemctl restart {self._svc_name}'
            )
            return

        self._recovery_attempts += 1
        self._recovery_in_progress = True

        self.get_logger().warn(
            f'Attempting lidar recovery #{self._recovery_attempts} '
            f'(silent for {silence_sec:.1f}s)...'
        )
        self._log_event('RECOVERY_ATTEMPT', {
            'attempt': self._recovery_attempts,
            'silence_sec': round(silence_sec, 1),
        })

        if self._recovery_attempts == 1:
            self._restart_lidar_service()
        elif self._recovery_attempts == 2:
            self._power_cycle_usb()
        else:
            self.get_logger().error(
                f'Recovery attempt #{self._recovery_attempts}: '
                f'All automatic recovery options exhausted. '
                f'Waiting for manual intervention.'
            )

    def _restart_lidar_service(self):
        """
        Restart the rplidar systemd service.
        Requires: sudo systemctl restart rplidar.service
        For this to work without password: set up sudoers rule (see README).
        """
        try:
            self.get_logger().info(
                f'Restarting systemd service: {self._svc_name}...')
            result = subprocess.run(
                ['sudo', 'systemctl', 'restart', self._svc_name],
                capture_output=True, text=True, timeout=15.0
            )
            if result.returncode == 0:
                self.get_logger().info(
                    f'Service {self._svc_name} restarted successfully.')
            else:
                self.get_logger().error(
                    f'Service restart failed: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Service restart timed out!')
        except Exception as e:
            self.get_logger().error(f'Service restart error: {e}')

    def _power_cycle_usb(self):
        """
        Power cycle the USB port using uhubctl (if available).
        This physically cuts and restores power to the lidar USB port.
        More aggressive than service restart — fixes USB lockup conditions.

        Install: sudo apt install uhubctl
        Find your hub: uhubctl (run once to list hubs)
        """
        try:
            self.get_logger().info('Attempting USB power cycle via uhubctl...')
            # Power off
            subprocess.run(
                ['sudo', 'uhubctl', '-a', '0', '-l', '1-1'],
                capture_output=True, timeout=5.0
            )
            time.sleep(2.0)
            # Power on
            subprocess.run(
                ['sudo', 'uhubctl', '-a', '1', '-l', '1-1'],
                capture_output=True, timeout=5.0
            )
            self.get_logger().info('USB power cycle complete. Waiting for lidar to spin up...')
        except FileNotFoundError:
            self.get_logger().warn(
                'uhubctl not found. Install: sudo apt install uhubctl\n'
                'Falling back to service restart instead.'
            )
            self._restart_lidar_service()
        except Exception as e:
            self.get_logger().error(f'USB power cycle error: {e}')

    # ── Helpers ───────────────────────────────────────────────────
    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _log_event(self, event_type: str, data: dict):
        """Publish a structured event log entry for maintenance records."""
        event = {
            'event'    : event_type,
            'timestamp': time.time(),
            **data
        }
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)
        self.get_logger().info(f'[WATCHDOG EVENT] {event_type}: {data}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
