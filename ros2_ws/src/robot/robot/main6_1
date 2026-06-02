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
OA_X_L = 1340.0
OA_START_Y = 1500.0
OA_END_Y = 3750.0
OA_OFFSET = 150.0  # WIDER OFFSET: Force robot to swerve further away

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
    
    last_lane = None

    while True:
        now = time.monotonic()
        
        # --- ABORT BUTTON ---
        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            robot.stop() 
            print("[FSM] IDLE — path cancelled/stopped")
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            print("[FSM] IDLE — Press BTN_4 to start AGGRESSIVE Obstacle Avoidance")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_4):
                # Setup Obstacle Avoidance Path
                path = densify_polyline(OA_WAYPOINTS, spacing=100.0) 
                
                robot._nav_follow_pp_path(
                    lookahead_distance=250.0,
                    max_linear_speed=150.0,
                    max_angular_speed=1.8,
                    goal_tolerance=20.0,
                    obstacles_range=450.0,
                    view_angle=math.radians(70.0),
                    safe_dist=200.0,
                    avoidance_delay=150,
                    alpha_Ld=0.4,
                    offset=270.0,
                    lane_width=450.0,
                    obstacle_avoidance=True,
                    x_L=300.0,
                    )
                robot.planner.set_path(path)
                last_lane = robot.planner.current_lane
                
                print(f"[FSM] MOVING — Sharp Avoidance Mode Enabled")
                state = "EXEC_OA_DRIVING"
                last_status_print_at = now

        elif state == "EXEC_OA_DRIVING":
            is_status_tick = (now - last_status_print_at >= STATUS_PRINT_INTERVAL_S)
            
            # --- CUSTOM NAVIGATION LOOP ---
            with robot._lock:
                obstacles = robot._obstacles_mm.copy()
                pose = robot._pose # (x_mm, y_mm, theta_rad)
            
            processed_obstacles = obstacles
            
            # Compute velocity
            v, w = robot.planner.compute_velocity(pose, processed_obstacles)
            robot.set_velocity(v, math.degrees(w))
            
            if is_status_tick:
                print_status(robot)
                print(f"  [DEBUG] v={v:5.1f} w={math.degrees(w):+5.1f} lane={robot.planner.current_lane}")
                last_status_print_at = now
            
            # Detection Check & Log
            if processed_obstacles.size > 0:
                view_angle = robot.planner.view_angle
                obs_range = robot.planner.obstacles_range
                lane_width = robot.planner.lane_width
                x_L = robot.planner.x_L
                
                angles = np.arctan2(processed_obstacles[:,1], processed_obstacles[:,0])
                in_fov = processed_obstacles[np.abs(angles) <= view_angle, :]
                dists = np.linalg.norm(in_fov, axis=1) if in_fov.size > 0 else np.array([])
                in_range = in_fov[dists < obs_range, :] if in_fov.size > 0 else np.array([])
                
                if in_range.size > 0:
                    R = np.array([[np.cos(pose[2]), -np.sin(pose[2])], [np.sin(pose[2]), np.cos(pose[2])]])
                    obs_world = (R @ in_range.T).T + np.array([[pose[0], pose[1]],])
                    in_lane_mask = (np.abs(obs_world[:, 0] - x_L) < lane_width) | (np.abs(in_range[:, 1]) < 400.0)
                    in_lane = in_range[in_lane_mask, :]
                    
                    if in_lane.size > 0 and is_status_tick:
                        print(f"  ✅ CONE IN VIEW! closest={np.min(np.linalg.norm(in_lane, axis=1)):.1f}mm")

            # Lane Detection Print
            current_lane = robot.planner.current_lane
            if current_lane != last_lane:
                print(f"*** LANE SWITCH! Switched to {current_lane} lane ***")
                last_lane = current_lane

            # Check if target reached
            if robot.planner.TargetReached(robot.planner.remaining_path, pose[0], pose[1]):
                print("[FSM] DONE — Goal reached")
                robot.stop()
                state = "IDLE"

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
