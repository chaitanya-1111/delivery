#!/usr/bin/env python3
"""
Safety Logger Node - Step 5

Safety event JSONL logging system.
- Subscribes to safety events
- Logs to timestamped JSONL files
- Provides query interface for past events
- Maintains rotating logs for disk space management

Log format:
  {
    "timestamp": 1234567890.5,
    "event": "ESTOP_TRIGGERED",
    "severity": "CRITICAL",
    "state": "ESTOP_ACTIVE",
    "zone": "EMERGENCY",
    "distance": 0.15,
    "speed_scale": 0.0
  }
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
import gzip


class SafetyLoggerNode(Node):
    """
    Logs all safety events to JSONL files for analysis and debugging.
    
    Responsibilities:
    1. Subscribe to safety event topics
    2. Write structured logs to JSONL files
    3. Manage log rotation
    4. Provide query interface
    """
    
    def __init__(self):
        super().__init__('safety_logger_node')
        
        # Configuration
        self.declare_parameter('log_dir', '~/.ros/safety_logs')
        self.declare_parameter('max_log_size_mb', 100)
        self.declare_parameter('compress_old_logs', True)
        self.declare_parameter('keep_logs_days', 30)
        
        self.log_dir = Path(self.get_parameter('log_dir').value).expanduser()
        self.max_log_size = self.get_parameter('max_log_size_mb').value * 1024 * 1024
        self.compress_old = self.get_parameter('compress_old_logs').value
        self.keep_days = self.get_parameter('keep_logs_days').value
        
        # Create log directory
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Current log file
        self.current_log_file = self._get_log_filename()
        self.log_file_size = 0
        
        # Event counters
        self.event_counts: Dict[str, int] = {}
        self.severity_counts: Dict[str, int] = {}
        
        # QoS settings
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # Subscriptions
        self.safety_event_sub = self.create_subscription(
            String, '/safety/event', self._safety_event_callback, qos_reliable)
        
        # Publishers
        self.log_stats_pub = self.create_publisher(
            String, '/logging/stats', qos_reliable)
        
        # Initialize log file with metadata
        self._write_log_entry({
            "type": "LOG_START",
            "timestamp": datetime.now().isoformat(),
            "node": "safety_logger",
            "version": "1.0"
        })
        
        # Housekeeping timer
        self.create_timer(60.0, self._housekeeping_loop)  # Every minute
        self.create_timer(300.0, self._stats_report_loop)  # Every 5 minutes
        
        self.get_logger().info(f"Safety Logger initialized. Log dir: {self.log_dir}")
    
    def _get_log_filename(self) -> Path:
        """Generate timestamped log filename."""
        now = datetime.now()
        filename = f"safety_{now.strftime('%Y%m%d_%H%M%S')}.jsonl"
        return self.log_dir / filename
    
    def _safety_event_callback(self, msg: String) -> None:
        """Process incoming safety event."""
        try:
            # Parse event JSON
            event_data = json.loads(msg.data)
            
            # Add metadata
            event_data['received_time'] = datetime.now().isoformat()
            
            # Track counts
            event_type = event_data.get('event', 'UNKNOWN')
            severity = event_data.get('severity', 'UNKNOWN')
            
            self.event_counts[event_type] = self.event_counts.get(event_type, 0) + 1
            self.severity_counts[severity] = self.severity_counts.get(severity, 0) + 1
            
            # Write to log file
            self._write_log_entry(event_data)
            
            # Log to console for critical events
            if severity in ['CRITICAL', 'FATAL']:
                self.get_logger().critical(
                    f"Safety Event: {event_type} - {event_data.get('state', 'UNKNOWN')}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse safety event: {e}")
            self._write_log_entry({
                "type": "LOG_ERROR",
                "error": str(e),
                "raw_data": msg.data
            })
    
    def _write_log_entry(self, data: Dict) -> None:
        """Write a single line to the JSONL log file."""
        try:
            # Check if we need to rotate log file
            if self.log_file_size > self.max_log_size:
                self._rotate_log_file()
            
            # Ensure timestamp
            if 'timestamp' not in data:
                data['timestamp'] = datetime.now().timestamp()
            
            # Write line
            line = json.dumps(data) + '\n'
            
            with open(self.current_log_file, 'a') as f:
                f.write(line)
            
            self.log_file_size += len(line.encode('utf-8'))
            
        except IOError as e:
            self.get_logger().error(f"Failed to write log entry: {e}")
    
    def _rotate_log_file(self) -> None:
        """Rotate to new log file."""
        try:
            # Write end marker
            self._write_log_entry({
                "type": "LOG_END",
                "timestamp": datetime.now().isoformat()
            })
            
            # Optionally compress old log
            if self.compress_old:
                self._compress_log_file(self.current_log_file)
            
            # Start new log file
            self.current_log_file = self._get_log_filename()
            self.log_file_size = 0
            
            self.get_logger().info(
                f"Log rotated. New file: {self.current_log_file.name}")
            
        except Exception as e:
            self.get_logger().error(f"Log rotation failed: {e}")
    
    def _compress_log_file(self, log_file: Path) -> None:
        """Compress old log file."""
        try:
            gz_file = log_file.with_suffix('.jsonl.gz')
            with open(log_file, 'rb') as f_in:
                with gzip.open(gz_file, 'wb') as f_out:
                    f_out.writelines(f_in)
            
            # Remove original
            log_file.unlink()
            self.get_logger().debug(f"Compressed: {log_file.name} → {gz_file.name}")
            
        except Exception as e:
            self.get_logger().warning(f"Compression failed: {e}")
    
    def _housekeeping_loop(self) -> None:
        """Cleanup old logs and maintain disk space."""
        try:
            now = datetime.now()
            cutoff_timestamp = (now.timestamp() - self.keep_days * 86400)
            
            # Remove old log files
            for log_file in self.log_dir.glob("safety_*.jsonl*"):
                if log_file.stat().st_mtime < cutoff_timestamp:
                    log_file.unlink()
                    self.get_logger().info(f"Deleted old log: {log_file.name}")
            
        except Exception as e:
            self.get_logger().warning(f"Housekeeping failed: {e}")
    
    def _stats_report_loop(self) -> None:
        """Publish logging statistics."""
        try:
            # Count log files
            log_files = list(self.log_dir.glob("safety_*.jsonl*"))
            total_size = sum(f.stat().st_size for f in log_files)
            
            stats = {
                "timestamp": datetime.now().isoformat(),
                "total_log_files": len(log_files),
                "total_size_mb": total_size / (1024 * 1024),
                "current_file": self.current_log_file.name,
                "event_counts": self.event_counts,
                "severity_counts": self.severity_counts
            }
            
            # Publish stats
            msg = String()
            msg.data = json.dumps(stats)
            self.log_stats_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Stats reporting failed: {e}")
    
    def query_events(self, event_type: Optional[str] = None,
                     severity: Optional[str] = None,
                     minutes: float = 5.0) -> List[Dict]:
        """
        Query logged events from the past N minutes.
        
        Args:
            event_type: Filter by event type (None = all)
            severity: Filter by severity (None = all)
            minutes: Look back N minutes
        
        Returns:
            List of matching event dictionaries
        """
        cutoff_time = datetime.now().timestamp() - (minutes * 60)
        events = []
        
        try:
            # Read current log file
            with open(self.current_log_file, 'r') as f:
                for line in f:
                    if not line.strip():
                        continue
                    
                    try:
                        event = json.loads(line)
                        
                        # Check timestamp
                        ts = event.get('timestamp', 0)
                        if isinstance(ts, str):
                            ts = datetime.fromisoformat(ts).timestamp()
                        
                        if ts < cutoff_time:
                            continue
                        
                        # Apply filters
                        if event_type and event.get('event') != event_type:
                            continue
                        if severity and event.get('severity') != severity:
                            continue
                        
                        events.append(event)
                    
                    except json.JSONDecodeError:
                        pass
        
        except FileNotFoundError:
            pass
        
        return events


def main(args=None):
    rclpy.init(args=args)
    node = SafetyLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
