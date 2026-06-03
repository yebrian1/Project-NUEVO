from __future__ import annotations
import time
import math
import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline
from robot.path_planner import APFPlanner

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Obstacle Avoidance Configuration
# ---------------------------------------------------------------------------
OA_X_L = 300.0
OA_START_Y = 1500.0
OA_END_Y = 3750.0

OA_WAYPOINTS = [
    (OA_X_L, OA_START_Y),
    (OA_X_L, OA_END_Y),
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.disable_gps()
    robot.enable_lidar()
    robot.set_lidar_filter(range_min_mm=15.0, range_max_mm=800.0)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_print_at = 0.0
    STATUS_PRINT_INTERVAL_S = 0.5
    
    remaining_path = []

    while True:
        now = time.monotonic()
        
        # --- ABORT BUTTON ---
        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            robot.stop() 
            print("[FSM] IDLE — path cancelled/stopped")
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            print("[FSM] IDLE — Press BTN_4 to start APF Obstacle Avoidance")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_4):
                # Setup Obstacle Avoidance Path
                path = densify_polyline(OA_WAYPOINTS, spacing=100.0) 
                
                # Initialize APF Planner with parameters similar to your PP config
                robot.planner = APFPlanner(
                    max_linear=150.0,
                    max_angular=2.5,
                    repulsion_gain=800.0,
                    repulsion_range=600.0,
                    goal_tolerance=20.0,
                    robot_front_mm=250.0,
                )
                remaining_path = path
                
                print(f"[FSM] MOVING — APF Avoidance Mode Enabled")
                state = "EXEC_OA_DRIVING"
                last_status_print_at = now

        elif state == "EXEC_OA_DRIVING":
            is_status_tick = (now - last_status_print_at >= STATUS_PRINT_INTERVAL_S)
            
            # --- CUSTOM NAVIGATION LOOP ---
            pose = robot._get_pose_mm() # (x_mm, y_mm, theta_rad)
            
            # Advance path waypoints
            while len(remaining_path) > 1:
                # If close to current waypoint, pop it
                if math.hypot(pose[0] - remaining_path[0][0], pose[1] - remaining_path[0][1]) < 150.0:
                    remaining_path.pop(0)
                else:
                    break
            
            # Check if target reached
            if math.hypot(pose[0] - remaining_path[0][0], pose[1] - remaining_path[0][1]) < 20.0:
                print("[FSM] DONE — Goal reached")
                robot.stop()
                state = "IDLE"
                continue

            # Fetch obstacles in world frame from the robot's tracker
            obs_world = robot._get_nearest_tracked_obstacle_disks_world_mm(pose, 800.0, 6)
            
            # Compute velocity using APF toward current waypoint
            v, w = robot.planner.navigate_to_goal(pose, remaining_path[0], obs_world)
            robot.set_velocity(v, math.degrees(w))
            
            if is_status_tick:
                print_status(robot)
                print(f"  [DEBUG] v={v:5.1f} w={math.degrees(w):+5.1f} | APF active")
                last_status_print_at = now

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

if __name__ == "__main__":
    import rclpy
    from robot.robot_node import RobotNode
    
    rclpy.init()
    node = RobotNode()
    robot = Robot(node)
    
    try:
        run(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        rclpy.shutdown()
