#!/usr/bin/env python3
import rclpy
from robot.robot_node import RobotNode
from robot.robot import Robot
from robot.lidar_helpers import save_lidar_plot
import time
import sys

def main():
    rclpy.init()
    node = RobotNode()
    robot = Robot(node)
    robot.enable_lidar()
    
    print("[SNAPSHOT] Waiting for LIDAR data on /scan...")
    
    # Give it a few seconds to receive at least one scan
    timeout = time.time() + 5.0
    found = False
    while time.time() < timeout:
        points = robot.get_obstacles()
        if points and len(points) > 0:
            print(f"[SNAPSHOT] Captured {len(points)} points. Saving plot...")
            save_lidar_plot(robot, filename="manual_snapshot.png")
            print("[SNAPSHOT] Done. Saved to ros2_ws/runtime_output/manual_snapshot.png")
            found = True
            break
        time.sleep(0.1)
    
    if not found:
        print("[SNAPSHOT] Error: No LIDAR data received within 5 seconds.")
    
    robot.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
