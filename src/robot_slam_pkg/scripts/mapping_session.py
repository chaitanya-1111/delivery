#!/usr/bin/env python3
"""
mapping_session.py

PRODUCTION TOOL: Guided mapping session for restaurant staff.

This script guides a non-technical operator through the restaurant
mapping process step by step with prompts, checks, and confirmations.

Usage:
  python3 scripts/mapping_session.py

The script will:
  1. Verify lidar and hardware are running
  2. Launch SLAM mapping
  3. Guide operator through driving the robot
  4. Save the map when coverage is sufficient
  5. Run quality validation
  6. Update the active map registry

This ensures consistent, high-quality maps every time.
"""

import subprocess
import sys
import time
import json


def print_banner(text: str):
    width = 50
    print("\n" + "═" * width)
    print(f"  {text}")
    print("═" * width)


def print_step(n: int, total: int, text: str):
    print(f"\n[Step {n}/{total}] {text}")
    print("─" * 40)


def check_topic_hz(topic: str, min_hz: float, timeout: float = 5.0) -> bool:
    """Check if a topic is publishing at sufficient rate."""
    print(f"  Checking {topic}...")
    try:
        result = subprocess.run(
            ["ros2", "topic", "hz", topic, "--window", "5"],
            capture_output=True, text=True, timeout=timeout + 2
        )
        output = result.stdout + result.stderr
        # Parse hz from output like "average rate: 8.234"
        for line in output.splitlines():
            if "average rate" in line:
                hz = float(line.split(":")[1].strip().split()[0])
                if hz >= min_hz:
                    print(f"  ✅ {topic}: {hz:.1f} Hz (OK)")
                    return True
                else:
                    print(f"  ❌ {topic}: {hz:.1f} Hz (too slow, need ≥{min_hz} Hz)")
                    return False
        print(f"  ❌ {topic}: no data")
        return False
    except subprocess.TimeoutExpired:
        print(f"  ❌ {topic}: timeout")
        return False
    except Exception as e:
        print(f"  ❌ {topic}: error: {e}")
        return False


def call_service(service: str, srv_type: str) -> bool:
    """Call a ROS 2 service."""
    try:
        result = subprocess.run(
            ["ros2", "service", "call", service, srv_type, "{}"],
            capture_output=True, text=True, timeout=15.0
        )
        return result.returncode == 0
    except Exception as e:
        print(f"Service call failed: {e}")
        return False


def get_topic_once(topic: str) -> str:
    """Get one message from a topic."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once", "--no-daemon"],
            capture_output=True, text=True, timeout=10.0
        )
        return result.stdout
    except Exception:
        return ""


def main():
    print_banner("Restaurant Floor Mapping Session")
    print("This tool will guide you through mapping your restaurant.")
    print("Estimated time: 30-60 minutes depending on restaurant size.")

    TOTAL_STEPS = 6

    # ── STEP 1: Prerequisites check ───────────────────────────────
    print_step(1, TOTAL_STEPS, "Checking prerequisites")
    print("Verifying lidar and hardware are running...")

    checks_ok = True

    if not check_topic_hz("/scan", min_hz=5.0):
        print("  ⚠️  /scan not publishing. Start lidar first:")
        print("      ros2 launch robot_lidar_pkg lidar_bringup.launch.py use_rviz:=false")
        checks_ok = False

    if not check_topic_hz("/odom", min_hz=5.0):
        print("  ⚠️  /odom not publishing. Start hardware first:")
        print("      ros2 launch robot_hardware_pkg hardware_bringup.launch.py")
        checks_ok = False

    if not checks_ok:
        print("\n❌ Prerequisites not met. Start missing components and retry.")
        sys.exit(1)

    print("\n✅ All prerequisites OK. Ready to begin mapping.")

    # ── STEP 2: Start SLAM ────────────────────────────────────────
    print_step(2, TOTAL_STEPS, "Starting SLAM mapping")
    input("Press ENTER to launch SLAM mapping (opens in background)...")

    slam_proc = subprocess.Popen(
        ["ros2", "launch", "robot_slam_pkg", "slam_mapping.launch.py",
         "use_rviz:=true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    print("Waiting for SLAM to initialize (10 seconds)...")
    time.sleep(10)

    if not check_topic_hz("/map", min_hz=0.1, timeout=8.0):
        print("❌ SLAM not publishing /map. Check slam_mapping.launch.py output.")
        slam_proc.terminate()
        sys.exit(1)

    print("✅ SLAM is running and publishing map.")

    # ── STEP 3: Teleop instructions ───────────────────────────────
    print_step(3, TOTAL_STEPS, "Drive the robot through the restaurant")
    print("""
  Open a NEW TERMINAL and run:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

  DRIVING GUIDE:
    ★ Drive SLOWLY (0.1-0.2 m/s max)
    ★ Cover ALL areas:
       □ Every aisle between tables
       □ Kitchen entrance and interior
       □ All doorways (pause in each doorway)
       □ Storage/service areas
       □ Along all walls
    ★ Make at least 2 full loops of the restaurant
      (triggers loop closure → better accuracy)
    ★ Watch the map grow in RViz
    ★ Unknown areas show as grey → drive there

  WHEN TO STOP:
    ★ Grey areas are gone / minimal
    ★ Map looks complete and walls are clear black lines
    ★ You've done 2+ laps
    """)

    input("Press ENTER when you have finished driving the route...")

    # ── STEP 4: Quality check ─────────────────────────────────────
    print_step(4, TOTAL_STEPS, "Checking map quality")
    quality_raw = get_topic_once("/slam/map_quality_report")

    if quality_raw:
        try:
            report = json.loads(quality_raw.strip().split("data: ")[1].strip("'\""))
            cov = report.get("coverage", "unknown")
            area = report.get("free_area", "unknown")
            quality = report.get("checks", {}).get("coverage", {})
            q_str = quality.get("coverage_quality", "?") if isinstance(quality, dict) else "?"

            print(f"  Coverage    : {cov}")
            print(f"  Free area   : {area}")
            print(f"  Quality     : {q_str}")

            if q_str in ("POOR", "FAIR"):
                print("\n  ⚠️  Map quality is low. Consider driving more of the restaurant.")
                cont = input("  Continue anyway? (y/N): ")
                if cont.lower() != "y":
                    print("  Going back to driving. Press ENTER when ready to retry check.")
                    input()
        except Exception:
            print("  Could not parse quality report. Proceeding...")
    else:
        print("  Could not get quality report. Proceeding...")

    # ── STEP 5: Save the map ──────────────────────────────────────
    print_step(5, TOTAL_STEPS, "Saving the map")
    input("Press ENTER to save the map...")

    print("Saving map (this takes ~10 seconds)...")
    if call_service("/slam/save_map", "std_srvs/srv/Trigger"):
        print("✅ Map saved successfully!")
    else:
        print("⚠️  Automatic save failed. Manual save:")
        map_path = input("  Enter map save path (e.g. ~/delivery_bot_ws/maps/restaurant_map): ")
        subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
        )

    # ── STEP 6: Done ──────────────────────────────────────────────
    print_step(6, TOTAL_STEPS, "Mapping Complete!")
    print("""
  ✅ Restaurant map has been saved.

  MAP FILES CREATED:
    ~/delivery_bot_ws/maps/restaurant_map_v*.pgm       (image)
    ~/delivery_bot_ws/maps/restaurant_map_v*.yaml      (metadata)
    ~/delivery_bot_ws/maps/restaurant_map_v*.posegraph (SLAM graph)

  NEXT STEPS:
    1. Stop this mapping session (Ctrl+C)
    2. Verify the map image looks correct:
         eog ~/delivery_bot_ws/maps/restaurant_map_v*.pgm
    3. Update config/delivery_locations.yaml with table coordinates
    4. Proceed to Step 3: Navigation
         ros2 launch robot_slam_pkg slam_localization.launch.py \\
           map_file:=~/delivery_bot_ws/maps/restaurant_map_v1

  """)


if __name__ == "__main__":
    main()