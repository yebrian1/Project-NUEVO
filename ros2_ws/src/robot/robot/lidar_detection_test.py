from __future__ import annotations
import time
import math
import numpy as np
import rclpy
from robot.robot import Robot, Unit
from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
)

# ---------------------------------------------------------------------------
# Test Configuration
# ---------------------------------------------------------------------------
DETECTION_RADIUS_MM = 200.0
FOV_DEG = 60.0  # Total arc angle (+/- 30 degrees)
TICK_RATE_HZ = 10.0

def run(robot: Robot) -> None:
    # 1. Setup
    robot.set_unit(Unit.MM)
    robot.enable_lidar()
    robot.set_lidar_filter(range_min_mm=0.0) # Disable min filter for close-range testing
    
    print("\n" + "="*50)
    print("      LiDAR DETECTION TEST")
    print("="*50)
    print(f"Detection Arc:  Front +/- {FOV_DEG/2:.1f}°")
    print(f"Trigger Radius: {DETECTION_RADIUS_MM} mm")
    print("Press Ctrl+C to stop.\n")

    period = 1.0 / TICK_RATE_HZ
    next_tick = time.monotonic()

    while rclpy.ok():
        # 2. Get latest LiDAR points in robot body frame
        points = np.asarray(robot.get_obstacles())
        
        if points.size > 0:
            x = points[:, 0]
            y = points[:, 1]
            
            # Calculate polar coordinates
            dists = np.linalg.norm(points, axis=1)
            angles_deg = np.degrees(np.arctan2(y, x))
            
            # 3. Filter for the specified arc
            # angles_deg: 0 is straight ahead, positive is left, negative is right
            mask = (dists <= DETECTION_RADIUS_MM) & (np.abs(angles_deg) <= (FOV_DEG / 2.0))
            detected = points[mask]
            
            if detected.size > 0:
                min_dist = np.min(dists[mask])
                avg_angle = np.mean(angles_deg[mask])
                
                # Signal detection via LED and Print
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                print(f"[!] OBSTACLE: Dist={min_dist:5.1f}mm | Angle={avg_angle:+5.1f}° | Points={len(detected)}")
            else:
                # No obstacle in the arc
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 100)
        else:
            # No LiDAR data at all
            robot.set_led(LED.RED, 50) # Dim red to indicate no data
            robot.set_led(LED.GREEN, 0)

        # 4. Control loop frequency
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

if __name__ == "__main__":
    from robot.robot_node import RobotNode
    
    rclpy.init()
    node = RobotNode()
    robot = Robot(node)
    
    try:
        run(robot)
    except KeyboardInterrupt:
        print("\nShutting down LiDAR test...")
    finally:
        robot.stop()
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, 0)
        rclpy.shutdown()
