#!/usr/bin/env python3
"""
Safety Supervisor Node - Step 5

Main safety module: gates all velocity commands before reaching hardware.
- Monitors LIDAR proximity zones (FREE, CAUTION, CRITICAL, EMERGENCY)
- Implements speed scaling based on distance
- Manages emergency stop and safety events
- Publishes safety status and diagnostics

Architecture:
  Nav2/Mission  →  /cmd_vel      ⤐
  Teleop        →  /cmd_vel_teleop ⤴→ [Safety Gate] → /cmd_vel_safe → Hardware
  
  LIDAR/Proximity → Distance check
  E-Stop → Hardware trigger
  Heartbeat → Component health
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import json
import time
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, Optional, Tuple


class SafetyZone(Enum):
    """Safety zones based on obstacle distance."""
    FREE = 0         # > 0.60m
    CAUTION = 1      # 0.35-0.60m
    CRITICAL = 2     # 0.20-0.35m
    EMERGENCY = 3    # < 0.20m


class SafetyState(Enum):
    """Safety system states."""
    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    CRITICAL = "CRITICAL"


@dataclass
class SafetyConfig:
    """Safety configuration parameters."""
    # Zone distance thresholds (meters)
    free_min: float = 0.60
    caution_max: float = 0.60
    caution_min: float = 0.35
    critical_max: float = 0.35
    critical_min: float = 0.20
    emergency_threshold: float = 0.20
    
    # Speed scaling factors
    speed_scale_caution: float = 0.5      # 50% speed in CAUTION zone
    speed_scale_critical: float = 0.1     # 10% speed in CRITICAL zone
    speed_scale_emergency: float = 0.0    # 0% (full stop) in EMERGENCY
    
    # Timeouts (seconds)
    lidar_timeout: float = 2.0
    heartbeat_timeout: float = 1.0
    nav_health_timeout: float = 3.0
    
    # E-Stop configuration
    estop_zero_threshold: float = 0.01    # Consider below this as zero velocity
    
    # Safety policy flags
    enforce_caution_limits: bool = True
    enforce_critical_limits: bool = True
    enforce_emergency_stop: bool = True
    degrade_on_sensor_loss: bool = True
    
    # Diagnostic thresholds
    min_lidar_samples: int = 10


@dataclass
class SafetyStatus:
    """Current safety status snapshot."""
    state: str
    current_zone: str
    speed_scale: float
    closest_distance: float
    lidar_healthy: bool
    nav_health: str
    estop_active: bool
    timestamp: float


class SafetyHardwareNode(Node):
    """
    Hardware Safety Supervisor Node.
    
    Responsibilities:
    1. Gate all velocity commands through safety checks
    2. Monitor LIDAR and proximity zones
    3. Manage emergency stop state
    4. Log safety-critical events
    5. Publish diagnostics and health status
    """
    
    def __init__(self):
        super().__init__('safety_supervisor_node')
        
        # Load configuration
        self.declare_parameter('safety_config_path', 'safety_config.yaml')
        self.config = self._load_config()
        self.get_logger().info(f"Safety config loaded: {self.config}")
        
        # State tracking
        self.safety_state = SafetyState.NOMINAL
        self.current_zone = SafetyZone.FREE
        self.speed_scale = 1.0
        self.closest_distance = float('inf')
        self.estop_active = False
        
        # Heartbeat tracking
        self.last_lidar_time = time.time()
        self.last_nav_health_time = time.time()
        self.last_hw_heartbeat_time = time.time()
        
        # Component health
        self.lidar_healthy = False
        self.nav_health_status = "UNKNOWN"
        self.hardware_healthy = False
        
        # Topic tracking
        self.last_cmd_vel_time = None
        self.last_cmd_vel_teleop_time = None
        
        # QoS for reliability
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos_reliable)
        
        self.cmd_vel_teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self._cmd_vel_teleop_callback, qos_reliable)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_reliable)
        
        self.estop_request_sub = self.create_subscription(
            Bool, '/safety/estop_request', self._estop_request_callback, qos_reliable)
        
        self.nav_health_sub = self.create_subscription(
            String, '/system/health_summary', self._nav_health_callback, qos_reliable)
        
        self.hw_heartbeat_sub = self.create_subscription(
            Bool, '/hardware/heartbeat', self._hw_heartbeat_callback, qos_reliable)
        
        # Publishers
        self.cmd_vel_safe_pub = self.create_publisher(
            Twist, '/cmd_vel_safe', qos_reliable)
        
        self.safety_status_pub = self.create_publisher(
            String, '/safety/status', qos_reliable)
        
        self.speed_scale_pub = self.create_publisher(
            Float32, '/safety/speed_scale', qos_reliable)
        
        self.estop_pub = self.create_publisher(
            Bool, '/safety/estop', qos_reliable)
        
        self.safety_event_pub = self.create_publisher(
            String, '/safety/event', qos_reliable)
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', qos_reliable)
        
        # Timer for main loop
        self.create_timer(0.05, self._safety_loop)  # 20 Hz
        self.create_timer(1.0, self._diagnostics_loop)  # 1 Hz
        
        self.get_logger().info("Safety Supervisor Node initialized")
    
    def _load_config(self) -> SafetyConfig:
        """Load safety configuration from YAML or use defaults."""
        # For now, use defaults. In production, parse YAML
        return SafetyConfig()
    
    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle incoming velocity command from navigation."""
        self.last_cmd_vel_time = time.time()
        self._gate_and_publish_velocity(msg, source="nav")
    
    def _cmd_vel_teleop_callback(self, msg: Twist) -> None:
        """Handle incoming velocity command from teleop."""
        self.last_cmd_vel_teleop_time = time.time()
        self._gate_and_publish_velocity(msg, source="teleop")
    
    def _scan_callback(self, msg: LaserScan) -> None:
        """Process LIDAR scan for proximity detection."""
        self.last_lidar_time = time.time()
        
        # Find minimum distance
        distances = []
        for i, range_val in enumerate(msg.ranges):
            if msg.range_min <= range_val <= msg.range_max:
                distances.append(range_val)
        
        if len(distances) >= self.config.min_lidar_samples:
            self.closest_distance = min(distances)
            self.lidar_healthy = True
            self._update_zone()
        else:
            self.lidar_healthy = False
            self.get_logger().warn(f"LIDAR: insufficient samples ({len(distances)})")
    
    def _estop_request_callback(self, msg: Bool) -> None:
        """Handle emergency stop requests."""
        if msg.data and not self.estop_active:
            self.estop_active = True
            self._log_event("ESTOP_TRIGGERED", severity="CRITICAL")
            self.get_logger().error("E-STOP ACTIVATED")
            
        elif not msg.data and self.estop_active:
            self.estop_active = False
            self._log_event("ESTOP_RELEASED", severity="INFO")
            self.get_logger().info("E-STOP released")
        
        # Publish estop state
        estop_msg = Bool()
        estop_msg.data = self.estop_active
        self.estop_pub.publish(estop_msg)
    
    def _nav_health_callback(self, msg: String) -> None:
        """Track navigation system health."""
        self.last_nav_health_time = time.time()
        self.nav_health_status = msg.data
        
        if msg.data == "CRITICAL":
            self._log_event("NAV_DEGRADED", severity="WARN")
    
    def _hw_heartbeat_callback(self, msg: Bool) -> None:
        """Track hardware heartbeat."""
        self.last_hw_heartbeat_time = time.time()
        self.hardware_healthy = msg.data
        
        if not msg.data:
            self._log_event("HW_HEARTBEAT_LOST", severity="CRITICAL")
    
    def _update_zone(self) -> None:
        """Update safety zone based on closest distance."""
        d = self.closest_distance
        
        if d > self.config.free_min:
            new_zone = SafetyZone.FREE
            self.speed_scale = 1.0
        elif d > self.config.caution_min:
            new_zone = SafetyZone.CAUTION
            self.speed_scale = self.config.speed_scale_caution
        elif d > self.config.critical_min:
            new_zone = SafetyZone.CRITICAL
            self.speed_scale = self.config.speed_scale_critical
        else:
            new_zone = SafetyZone.EMERGENCY
            self.speed_scale = self.config.speed_scale_emergency
        
        # Log zone transitions
        if new_zone != self.current_zone:
            self._log_event(f"ZONE_CHANGE_{self.current_zone.name}_to_{new_zone.name}",
                          severity="WARN" if new_zone.value > 1 else "INFO")
            self.current_zone = new_zone
        else:
            self.current_zone = new_zone
    
    def _gate_and_publish_velocity(self, cmd: Twist, source: str) -> None:
        """Apply safety gates and publish gated velocity command."""
        # Start with input command
        safe_cmd = Twist()
        safe_cmd.linear = cmd.linear
        safe_cmd.angular = cmd.angular
        
        # Check timeout conditions
        if not self._check_system_health():
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = 0.0
        
        # Apply E-STOP
        if self.estop_active:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = 0.0
        
        # Apply speed scaling from proximity zones
        if self.speed_scale < 1.0:
            safe_cmd.linear.x *= self.speed_scale
            safe_cmd.linear.y *= self.speed_scale
            safe_cmd.linear.z *= self.speed_scale
            safe_cmd.angular.x *= self.speed_scale
            safe_cmd.angular.y *= self.speed_scale
            safe_cmd.angular.z *= self.speed_scale
        
        # Publish gated command
        self.cmd_vel_safe_pub.publish(safe_cmd)
        
        # Publish speed scale
        scale_msg = Float32()
        scale_msg.data = self.speed_scale
        self.speed_scale_pub.publish(scale_msg)
    
    def _check_system_health(self) -> bool:
        """Check if system is healthy enough to move."""
        current_time = time.time()
        
        # Check LIDAR timeout
        if current_time - self.last_lidar_time > self.config.lidar_timeout:
            if self.config.degrade_on_sensor_loss:
                self.get_logger().warn("LIDAR timeout - safety system degraded")
                self.safety_state = SafetyState.DEGRADED
                return False
        
        # Check hardware heartbeat
        if current_time - self.last_hw_heartbeat_time > self.config.heartbeat_timeout:
            self.get_logger().error("Hardware heartbeat lost")
            return False
        
        # Check navigation health (optional degradation)
        if current_time - self.last_nav_health_time > self.config.nav_health_timeout:
            if self.nav_health_status == "CRITICAL":
                self.get_logger().warn("Navigation system critical")
                return False
        
        return True
    
    def _safety_loop(self) -> None:
        """Main safety monitoring loop."""
        # Update system state
        if not self._check_system_health():
            self.safety_state = SafetyState.DEGRADED
        elif self.estop_active:
            self.safety_state = SafetyState.ESTOP_ACTIVE
        elif self.current_zone == SafetyZone.EMERGENCY:
            self.safety_state = SafetyState.CRITICAL
        else:
            self.safety_state = SafetyState.NOMINAL
        
        # Publish status
        status = SafetyStatus(
            state=self.safety_state.value,
            current_zone=self.current_zone.name,
            speed_scale=self.speed_scale,
            closest_distance=self.closest_distance,
            lidar_healthy=self.lidar_healthy,
            nav_health=self.nav_health_status,
            estop_active=self.estop_active,
            timestamp=time.time()
        )
        
        status_msg = String()
        status_msg.data = json.dumps(asdict(status), default=str)
        self.safety_status_pub.publish(status_msg)
    
    def _diagnostics_loop(self) -> None:
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Safety status diagnostic
        safety_diag = DiagnosticStatus()
        safety_diag.name = "Safety Supervisor"
        safety_diag.hardware_id = "safety_supervisor_node"
        
        if self.safety_state == SafetyState.NOMINAL:
            safety_diag.level = DiagnosticStatus.OK
        elif self.safety_state == SafetyState.DEGRADED:
            safety_diag.level = DiagnosticStatus.WARN
        else:
            safety_diag.level = DiagnosticStatus.ERROR
        
        safety_diag.message = self.safety_state.value
        diag_array.status.append(safety_diag)
        
        # LIDAR diagnostic
        lidar_diag = DiagnosticStatus()
        lidar_diag.name = "LIDAR Sensor"
        lidar_diag.hardware_id = "lidar_0"
        lidar_diag.level = DiagnosticStatus.OK if self.lidar_healthy else DiagnosticStatus.ERROR
        lidar_diag.message = f"Distance: {self.closest_distance:.2f}m, Zone: {self.current_zone.name}"
        diag_array.status.append(lidar_diag)
        
        # Hardware diagnostic
        hw_diag = DiagnosticStatus()
        hw_diag.name = "Hardware Interface"
        hw_diag.hardware_id = "hardware_0"
        hw_diag.level = DiagnosticStatus.OK if self.hardware_healthy else DiagnosticStatus.ERROR
        hw_diag.message = "Online" if self.hardware_healthy else "Offline"
        diag_array.status.append(hw_diag)
        
        self.diagnostics_pub.publish(diag_array)
    
    def _log_event(self, event_type: str, severity: str = "INFO") -> None:
        """Log a safety event."""
        event_data = {
            "event": event_type,
            "severity": severity,
            "timestamp": time.time(),
            "state": self.safety_state.value,
            "zone": self.current_zone.name,
            "distance": self.closest_distance,
            "speed_scale": self.speed_scale
        }
        
        event_msg = String()
        event_msg.data = json.dumps(event_data)
        self.safety_event_pub.publish(event_msg)
        
        log_level = getattr(self.get_logger(), severity.lower(), self.get_logger().info)
        log_level(f"Safety Event: {event_type} ({severity})")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyHardwareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
