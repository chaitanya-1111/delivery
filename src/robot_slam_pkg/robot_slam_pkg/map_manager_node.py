#!/usr/bin/env python3
"""
map_manager_node.py

PRODUCTION ROLE:
  Manages the complete lifecycle of restaurant maps.

  In production, you don't just have ONE map file. You have:
    - restaurant_map_v1.yaml  (opening day)
    - restaurant_map_v2.yaml  (after renovation)
    - restaurant_map_night.yaml (chairs stacked, more free space)

  This node:
    1. Saves maps with automatic versioning + metadata (timestamp,
       who triggered it, quality score, coverage area)
    2. Maintains a map registry (maps/registry.json)
    3. Exposes services to save/load/list/activate maps
    4. Publishes which map is currently active
    5. Auto-saves during mapping at regular intervals (crash safety)
    6. Detects if active map is stale (restaurant rearranged)

SERVICES PROVIDED:
  /slam/save_map       (std_srvs/Trigger) → save current map with auto-name
  /slam/save_map_named (custom)           → save with specific name
  /slam/list_maps      (std_srvs/Trigger) → publish map list to /slam/map_list
  /slam/map_info       (std_srvs/Trigger) → publish current map metadata

TOPICS PUBLISHED:
  /slam/active_map     (std_msgs/String)  → currently loaded map name
  /slam/map_list       (std_msgs/String JSON) → available maps
  /slam/map_age_days   (std_msgs/Float32) → how old is current map

TOPICS SUBSCRIBED:
  /map  (nav_msgs/OccupancyGrid) → monitors the live map for quality metrics
"""

import json
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger


class MapManagerNode(Node):

    def __init__(self):
        super().__init__('map_manager_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('maps_directory',
                               os.path.expanduser('~/delivery_bot_ws/maps'))
        self.declare_parameter('map_base_name', 'restaurant_map')
        self.declare_parameter('auto_save_interval_sec', 120.0)  # every 2 min
        self.declare_parameter('is_mapping_mode', True)

        self._maps_dir   = Path(self.get_parameter('maps_directory').value)
        self._base_name  = self.get_parameter('map_base_name').value
        self._auto_int   = self.get_parameter('auto_save_interval_sec').value
        self._is_mapping = self.get_parameter('is_mapping_mode').value

        # Ensure maps directory exists
        self._maps_dir.mkdir(parents=True, exist_ok=True)

        self._registry_file = self._maps_dir / 'registry.json'
        self._registry = self._load_registry()

        # ── State ─────────────────────────────────────────────────
        self._active_map: str = None
        self._current_map_msg: OccupancyGrid = None
        self._map_start_time: float = time.time()
        self._last_auto_save: float = time.time()
        self._save_count: int = 0

        # ── Subscriptions ─────────────────────────────────────────
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 10)

        # ── Publishers ────────────────────────────────────────────
        self._active_map_pub = self.create_publisher(String, '/slam/active_map', 10)
        self._map_list_pub   = self.create_publisher(String, '/slam/map_list', 10)
        self._map_age_pub    = self.create_publisher(Float32, '/slam/map_age_days', 10)

        # ── Services ──────────────────────────────────────────────
        self._save_srv       = self.create_service(
            Trigger, '/slam/save_map', self._handle_save_map)
        self._list_srv       = self.create_service(
            Trigger, '/slam/list_maps', self._handle_list_maps)
        self._info_srv       = self.create_service(
            Trigger, '/slam/map_info', self._handle_map_info)

        # ── Timers ────────────────────────────────────────────────
        self._status_timer   = self.create_timer(5.0,  self._publish_status)
        self._autosave_timer = self.create_timer(30.0, self._auto_save_check)

        self.get_logger().info(
            f'MapManagerNode started.\n'
            f'  Maps directory: {self._maps_dir}\n'
            f'  Mode: {"MAPPING" if self._is_mapping else "LOCALIZATION"}\n'
            f'  Auto-save interval: {self._auto_int}s\n'
            f'  Known maps: {len(self._registry.get("maps", {}))}'
        )

    # ── Map Subscription ─────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        """Receive the live occupancy grid for quality monitoring."""
        self._current_map_msg = msg

    # ── Auto-Save ─────────────────────────────────────────────────
    def _auto_save_check(self):
        """
        During mapping sessions, auto-save at regular intervals.
        This prevents losing a long mapping session to a crash.
        """
        if not self._is_mapping:
            return
        now = time.time()
        if (now - self._last_auto_save) >= self._auto_int:
            self.get_logger().info('Auto-saving map (crash safety)...')
            success, name = self._save_map_to_disk('autosave')
            if success:
                self._last_auto_save = now
                self.get_logger().info(f'Auto-save complete: {name}')

    # ── Save Map ──────────────────────────────────────────────────
    def _handle_save_map(self, request, response):
        """Service handler: save current map with auto-generated name."""
        success, name = self._save_map_to_disk(self._base_name)
        response.success = success
        response.message = f'Saved: {name}' if success else f'Save FAILED: {name}'
        return response

    def _save_map_to_disk(self, base_name: str) -> tuple:
        """
        Save the current SLAM map to disk using nav2 map_saver_cli.

        Creates:
          maps/restaurant_map_v3.pgm   (image: free=white, occupied=black)
          maps/restaurant_map_v3.yaml  (metadata: resolution, origin, etc.)

        Also saves SLAM Toolbox pose graph:
          maps/restaurant_map_v3.posegraph
          maps/restaurant_map_v3.data

        Returns (success: bool, name_or_error: str)
        """
        # Generate versioned filename
        version    = self._next_version(base_name)
        timestamp  = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name   = f'{base_name}_v{version}_{timestamp}'
        map_path   = str(self._maps_dir / map_name)

        self.get_logger().info(f'Saving map to: {map_path}.*')

        # ── Save occupancy grid (.pgm + .yaml) ───────────────────
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path,
                 '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=30.0
            )
            if result.returncode != 0:
                err = result.stderr or result.stdout
                self.get_logger().error(f'map_saver_cli failed: {err}')
                return False, err
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out!')
            return False, 'timeout'
        except Exception as e:
            self.get_logger().error(f'map_saver_cli error: {e}')
            return False, str(e)

        # ── Save SLAM Toolbox pose graph (for resuming/localization) ─
        try:
            # Call slam_toolbox's serialize_map service
            subprocess.run(
                ['ros2', 'service', 'call',
                 '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{filename: "{map_path}"}}'],
                capture_output=True, text=True, timeout=15.0
            )
            self.get_logger().info('Pose graph serialized.')
        except Exception as e:
            self.get_logger().warn(
                f'Pose graph serialization failed: {e}. '
                f'Map image saved but localization mode may not work.')

        # ── Compute map quality metrics ───────────────────────────
        quality = self._compute_map_quality()

        # ── Update registry ───────────────────────────────────────
        entry = {
            'name'          : map_name,
            'path'          : map_path,
            'version'       : version,
            'timestamp'     : timestamp,
            'created_epoch' : time.time(),
            'quality'       : quality,
            'base_name'     : base_name,
        }

        maps = self._registry.setdefault('maps', {})
        maps[map_name] = entry
        self._registry['latest'] = map_name
        self._save_registry()

        self._active_map = map_name
        self._save_count += 1

        self.get_logger().info(
            f'Map saved successfully!\n'
            f'  Name    : {map_name}\n'
            f'  Version : v{version}\n'
            f'  Quality : {quality}\n'
            f'  Files   : {map_path}.pgm / .yaml / .posegraph'
        )

        return True, map_name

    def _compute_map_quality(self) -> dict:
        """
        Analyze the current occupancy grid to compute quality metrics.

        Returns a dict with:
          free_cells_pct:     % of known-free cells
          occupied_cells_pct: % of known-occupied cells
          unknown_cells_pct:  % of unknown cells (not yet mapped)
          total_cells:        total grid cells
          map_area_m2:        approximate mapped area in m²
          coverage_quality:   'POOR' / 'FAIR' / 'GOOD' / 'EXCELLENT'
        """
        if self._current_map_msg is None:
            return {'coverage_quality': 'UNKNOWN', 'reason': 'No map received yet'}

        grid  = self._current_map_msg
        data  = grid.data
        total = len(data)

        if total == 0:
            return {'coverage_quality': 'EMPTY'}

        free     = sum(1 for c in data if c == 0)
        occupied = sum(1 for c in data if c == 100)
        unknown  = sum(1 for c in data if c == -1)

        free_pct     = free     / total
        occupied_pct = occupied / total
        unknown_pct  = unknown  / total

        # Area in m²: free cells × resolution²
        res      = grid.info.resolution     # meters per cell
        area_m2  = free * (res ** 2)

        # Quality assessment
        # A good restaurant map has:
        #   - < 20% unknown cells (most areas scanned)
        #   - > 30% free cells (open dining areas captured)
        #   - Sufficient occupied cells (walls detected)
        if unknown_pct > 0.50:
            quality_str = 'POOR'
        elif unknown_pct > 0.30:
            quality_str = 'FAIR'
        elif unknown_pct > 0.15:
            quality_str = 'GOOD'
        else:
            quality_str = 'EXCELLENT'

        return {
            'free_cells_pct'    : round(free_pct     * 100, 1),
            'occupied_cells_pct': round(occupied_pct * 100, 1),
            'unknown_cells_pct' : round(unknown_pct  * 100, 1),
            'total_cells'       : total,
            'map_area_m2'       : round(area_m2, 1),
            'coverage_quality'  : quality_str,
        }

    # ── List Maps Service ─────────────────────────────────────────
    def _handle_list_maps(self, request, response):
        maps = self._registry.get('maps', {})
        summary = []
        for name, entry in sorted(maps.items(),
                                   key=lambda x: x[1].get('created_epoch', 0),
                                   reverse=True):
            summary.append({
                'name'      : name,
                'version'   : entry.get('version'),
                'timestamp' : entry.get('timestamp'),
                'quality'   : entry.get('quality', {}).get('coverage_quality', '?'),
            })

        msg = String()
        msg.data = json.dumps({'maps': summary, 'count': len(summary)})
        self._map_list_pub.publish(msg)

        response.success = True
        response.message = f'{len(summary)} maps available. Published to /slam/map_list.'
        return response

    # ── Map Info Service ──────────────────────────────────────────
    def _handle_map_info(self, request, response):
        if self._active_map:
            entry = self._registry.get('maps', {}).get(self._active_map, {})
            response.success = True
            response.message = json.dumps(entry)
        else:
            response.success = False
            response.message = 'No active map'
        return response

    # ── Status Publisher ──────────────────────────────────────────
    def _publish_status(self):
        # Active map name
        msg = String()
        msg.data = self._active_map or 'none'
        self._active_map_pub.publish(msg)

        # Map age (days since last save)
        latest = self._registry.get('latest')
        if latest:
            entry = self._registry.get('maps', {}).get(latest, {})
            epoch = entry.get('created_epoch', time.time())
            age_days = (time.time() - epoch) / 86400.0
            age_msg = Float32()
            age_msg.data = float(age_days)
            self._map_age_pub.publish(age_msg)

            if age_days > 30:
                self.get_logger().warn(
                    f'Active map is {age_days:.0f} days old. '
                    f'Consider re-mapping if restaurant layout has changed.')

    # ── Registry Helpers ──────────────────────────────────────────
    def _load_registry(self) -> dict:
        if self._registry_file.exists():
            try:
                return json.loads(self._registry_file.read_text())
            except Exception as e:
                self.get_logger().warn(f'Registry load failed: {e}. Starting fresh.')
        return {'maps': {}, 'latest': None}

    def _save_registry(self):
        try:
            self._registry_file.write_text(
                json.dumps(self._registry, indent=2))
        except Exception as e:
            self.get_logger().error(f'Registry save failed: {e}')

    def _next_version(self, base_name: str) -> int:
        maps = self._registry.get('maps', {})
        versions = [
            v.get('version', 0)
            for v in maps.values()
            if v.get('base_name') == base_name
        ]
        return max(versions, default=0) + 1


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
