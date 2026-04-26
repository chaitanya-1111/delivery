#!/usr/bin/env python3
"""
Hardware Watchdog Node - Step 5

Component heartbeat monitoring dashboard.
- Monitors heartbeats from all critical hardware components
- Tracks latencies and failure modes
- Publishes overall health summary
- Triggers alerts on component loss

Components monitored:
  - Motor controllers
  - LIDAR sensor
  - IMU/Odometry
  - Power management
  - Network interfaces
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import json
import time
from dataclasses import dataclass, asdict
from enum import Enum
from typing import Dict, Optional


class ComponentState(Enum):
    """Component health states."""
    UNKNOWN = "UNKNOWN"
    ONLINE = "ONLINE"
    LATENT = "LATENT"
    OFFLINE = "OFFLINE"
    ERROR = "ERROR"


@dataclass
class ComponentHealth:
    """Health status of a single component."""
    name: str
    state: str
    last_heartbeat: float
    latency_ms: float
    failure_count: int
    last_error: Optional[str]


class HardwareWatchdogNode(Node):
    """
    Monitors hardware component heartbeats and publishes health dashboard.
    
    Responsibilities:
    1. Monitor heartbeat topics from each hardware component
    2. Track latency and failure patterns
    3. Publish aggregated health status
    4. Generate diagnostics for troubleshooting
    """
    
    def __init__(self):
        super().__init__('hardware_watchdog_node')
        
        # Configuration
        self.declare_parameter('heartbeat_timeout', 1.5)
        self.declare_parameter('latency_warning_threshold', 500)  # ms
        
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.latency_warn_threshold = self.get_parameter('latency_warning_threshold').value
        
        # Component tracking
        self.components: Dict[str, ComponentHealth] = {}
        self.component_timestamps: Dict[str, float] = {}
        
        # Initialize components
        self._init_components()
        
        # QoS settings
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to heartbeats
        self._setup_heartbeat_subscriptions()
        
        # Publishers
        self.health_pub = self.create_publisher(
            String, '/system/health', qos_reliable)
        
        self.health_summary_pub = self.create_publisher(
            String, '/system/health_summary', qos_reliable)
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics_watchdog', qos_reliable)
        
        self.component_status_pub = self.create_publisher(
            String, '/system/component_status', qos_reliable)
        
        # Monitoring loop
        self.create_timer(0.1, self._watchdog_loop)  # 10 Hz
        self.create_timer(1.0, self._diagnostics_loop)  # 1 Hz
        self.create_timer(5.0, self._health_report_loop)  # 5 sec
        
        self.get_logger().info("Hardware Watchdog Node initialized")
    
    def _init_components(self) -> None:
        """Initialize component list and state."""
        component_names = [
            'motor_controller_left',
            'motor_controller_right',
            'lidar_scanner',
            'imu_sensor',
            'power_management',
            'network_interface',
        ]
        
        current_time = time.time()
        for name in component_names:
            self.components[name] = ComponentHealth(
                name=name,
                state=ComponentState.UNKNOWN.value,
                last_heartbeat=current_time,
                latency_ms=0.0,
                failure_count=0,
                last_error=None
            )
            self.component_timestamps[name] = current_time
    
    def _setup_heartbeat_subscriptions(self) -> None:
        """Setup subscriptions for each component heartbeat."""
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        heartbeat_topics = {
            'motor_controller_left': '/hardware/motor_left/heartbeat',
            'motor_controller_right': '/hardware/motor_right/heartbeat',
            'lidar_scanner': '/scan_info',  # Proxy for LIDAR activity
            'imu_sensor': '/imu/data',  # Proxy for IMU activity
            'power_management': '/battery/heartbeat',
            'network_interface': '/network/heartbeat',
        }
        
        for component_name, topic in heartbeat_topics.items():
            # Create callback with closure to capture component name
            def make_callback(comp_name):
                def callback(msg):
                    self._heartbeat_callback(comp_name, msg)
                return callback
            
            # Subscribe with custom QoS
            self.create_subscription(
                Bool, topic, make_callback(component_name), qos_best_effort)
            
            self.get_logger().info(f"Subscribed to {topic} for {component_name}")
    
    def _heartbeat_callback(self, component_name: str, msg: Bool) -> None:
        """Handle heartbeat message from component."""
        current_time = time.time()
        prev_time = self.component_timestamps[component_name]
        latency = (current_time - prev_time) * 1000  # Convert to ms
        
        self.component_timestamps[component_name] = current_time
        
        # Update component state
        comp = self.components[component_name]
        comp.last_heartbeat = current_time
        comp.latency_ms = latency
        
        # Determine state
        if msg.data:  # Heartbeat indicates health
            if latency > self.latency_warn_threshold:
                comp.state = ComponentState.LATENT.value
            else:
                comp.state = ComponentState.ONLINE.value
        else:
            comp.state = ComponentState.ERROR.value
            comp.failure_count += 1
            comp.last_error = f"Health flag False at {current_time}"
        
        self.get_logger().debug(
            f"{component_name}: {comp.state} (latency: {latency:.1f}ms)")
    
    def _watchdog_loop(self) -> None:
        """Main watchdog monitoring loop."""
        current_time = time.time()
        
        # Check timeouts for each component
        for component_name, comp in self.components.items():
            time_since_heartbeat = current_time - self.component_timestamps[component_name]
            
            if time_since_heartbeat > self.heartbeat_timeout:
                # Component timeout
                if comp.state != ComponentState.OFFLINE.value:
                    comp.state = ComponentState.OFFLINE.value
                    comp.failure_count += 1
                    comp.last_error = f"Heartbeat timeout at {current_time}"
                    self.get_logger().error(
                        f"WATCHDOG: {component_name} offline (missed for {time_since_heartbeat:.2f}s)")
            
            # Track state transitions
            if comp.state == ComponentState.UNKNOWN.value and time_since_heartbeat < 5.0:
                # Give components 5s to send first heartbeat
                pass
    
    def _get_overall_health(self) -> str:
        """Calculate overall system health."""
        states = [comp.state for comp in self.components.values()]
        
        # Count states
        online_count = states.count(ComponentState.ONLINE.value)
        unknown_count = states.count(ComponentState.UNKNOWN.value)
        latent_count = states.count(ComponentState.LATENT.value)
        error_count = states.count(ComponentState.ERROR.value)
        offline_count = states.count(ComponentState.OFFLINE.value)
        
        total = len(states)
        
        # Decision logic
        if offline_count > 0 or error_count > 1:
            return "CRITICAL"
        elif error_count > 0 or latent_count > 2:
            return "DEGRADED"
        elif latent_count > 0 or unknown_count > 2:
            return "CAUTION"
        else:
            return "OK"
    
    def _diagnostics_loop(self) -> None:
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Overall watchdog diagnostic
        watchdog_diag = DiagnosticStatus()
        watchdog_diag.name = "Hardware Watchdog"
        watchdog_diag.hardware_id = "watchdog_0"
        
        overall_health = self._get_overall_health()
        if overall_health == "OK":
            watchdog_diag.level = DiagnosticStatus.OK
        elif overall_health == "CAUTION":
            watchdog_diag.level = DiagnosticStatus.WARN
        else:
            watchdog_diag.level = DiagnosticStatus.ERROR
        
        watchdog_diag.message = overall_health
        diag_array.status.append(watchdog_diag)
        
        # Per-component diagnostics
        for comp in self.components.values():
            comp_diag = DiagnosticStatus()
            comp_diag.name = f"Component: {comp.name}"
            comp_diag.hardware_id = comp.name
            
            if comp.state == ComponentState.ONLINE.value:
                comp_diag.level = DiagnosticStatus.OK
            elif comp.state in [ComponentState.LATENT.value, ComponentState.UNKNOWN.value]:
                comp_diag.level = DiagnosticStatus.WARN
            else:
                comp_diag.level = DiagnosticStatus.ERROR
            
            comp_diag.message = (
                f"State: {comp.state}, "
                f"Latency: {comp.latency_ms:.1f}ms, "
                f"Failures: {comp.failure_count}"
            )
            
            diag_array.status.append(comp_diag)
        
        self.diagnostics_pub.publish(diag_array)
    
    def _health_report_loop(self) -> None:
        """Publish health reports."""
        current_time = time.time()
        
        # Build detailed health report
        health_data = {
            "timestamp": current_time,
            "overall_health": self._get_overall_health(),
            "components": {
                comp.name: asdict(comp) for comp in self.components.values()
            }
        }
        
        # Publish detailed health
        health_msg = String()
        health_msg.data = json.dumps(health_data, default=str)
        self.health_pub.publish(health_msg)
        
        # Publish summary
        summary_msg = String()
        summary_msg.data = self._get_overall_health()
        self.health_summary_pub.publish(summary_msg)
        
        # Publish component status
        component_status_data = {
            "timestamp": current_time,
            "components": [
                {
                    "name": comp.name,
                    "state": comp.state,
                    "latency_ms": comp.latency_ms,
                    "failures": comp.failure_count
                }
                for comp in self.components.values()
            ]
        }
        
        status_msg = String()
        status_msg.data = json.dumps(component_status_data)
        self.component_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareWatchdogNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
