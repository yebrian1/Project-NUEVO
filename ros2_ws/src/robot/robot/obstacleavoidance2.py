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
OA_OFFSET = 250.0

OA_WAYPOINTS = [
    (OA_X_L, OA_START_Y),
    (OA_X_L, OA_END_Y),
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.disable_gps() # Explicitly disable GPS fusion
    robot.enable_lidar() # LiDAR is REQUIRED for obstacle detection
    robot.set_lidar_filter(range_min_mm=15.0, range_max_mm=800.0) # Standard filter

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

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)

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
            show_idle_leds(robot)
            print("[FSM] IDLE — path cancelled/stopped")
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — GPS DISABLED. Press BTN_4 to start Obstacle Avoidance (Odom Only)")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.was_button_pressed(Button.BTN_4):
                state = "CMD_OA_START"

        elif state == "CMD_OA_START":
            show_running_leds(robot)
            
            # Wait for firmware bridge
            bridge_wait = time.monotonic()
            while time.monotonic() - bridge_wait < 5.0:
                if robot.get_state() == FirmwareState.RUNNING:
                    break
                time.sleep(0.1)

            # Setup Obstacle Avoidance Path
            path = densify_polyline(OA_WAYPOINTS, spacing=200.0) 
            
            robot._nav_follow_pp_path(
                lookahead_distance=200.0,
                max_linear_speed=110.0,
                max_angular_speed=2.0,
                goal_tolerance=50.0,
                obstacles_range=400.0, # Monitor maximum 400mm in front
                view_angle=math.radians(25.0), # 50 degree cone FOV (+/- 25 deg)
                safe_dist=300.0,
                avoidance_delay=60,
                alpha_Ld=0.5,
                offset=OA_OFFSET,
                lane_width=1500.0, # Robust lane width
                obstacle_avoidance=True,
                x_L=OA_X_L,
            )
            robot.planner.set_path(path)
            last_lane = robot.planner.current_lane
            
            print(f"[FSM] MOVING — ODOMETRY ONLY. Start Lane: {last_lane}")
            state = "EXEC_OA_DRIVING"
            last_status_print_at = now

        elif state == "EXEC_OA_DRIVING":
            is_status_tick = (now - last_status_print_at >= STATUS_PRINT_INTERVAL_S)
            if is_status_tick:
                print_status(robot)
                last_status_print_at = now
            
            # --- CUSTOM NAVIGATION LOOP ---
            with robot._lock:
                obstacles = robot._obstacles_mm.copy()
                pose = robot._pose # (x_mm, y_mm, theta_rad)
            
            # USE RAW OBSTACLES (Confirmed in testing)
            processed_obstacles = obstacles
            
            # Detection Check (Synchronized with main.py)
            if processed_obstacles.size > 0:
                x, y, theta = pose
                view_angle = robot.planner.view_angle
                obs_range = robot.planner.obstacles_range
                lane_width = robot.planner.lane_width
                x_L = robot.planner.x_L
                
                # 1. FOV Filter
                angles = np.arctan2(processed_obstacles[:,1], processed_obstacles[:,0])
                in_fov = processed_obstacles[np.abs(angles) <= view_angle, :]
                
                # 2. Range Filter
                dists = np.linalg.norm(in_fov, axis=1) if in_fov.size > 0 else np.array([])
                in_range = in_fov[dists < obs_range, :] if in_fov.size > 0 else np.array([])
                
                # 3. Robust Lane Filter
                if in_range.size > 0:
                    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                    obs_world = (R @ in_range.T).T + np.array([[x, y],])
                    # Success condition: either in absolute world lane or robot-relative path
                    in_lane_mask = (np.abs(obs_world[:, 0] - x_L) < lane_width) | (np.abs(in_range[:, 1]) < 400.0)
                    in_lane = in_range[in_lane_mask, :]
                    
                    if in_lane.size > 0:
                        print(f"  ✅ OBSTACLE FOUND! count={len(in_lane)} closest={np.min(np.linalg.norm(in_lane, axis=1)):.1f}mm")

            # Compute velocity using raw obstacles
            v, w = robot.planner.compute_velocity(pose, processed_obstacles)
            robot.set_velocity(v, math.degrees(w))
            
            # Lane Detection Print
            current_lane = robot.planner.current_lane
            if current_lane != last_lane:
                print(f"*** LANE SWITCH! Switched to {current_lane} lane ***")
                last_lane = current_lane

            # Check if target reached
            if robot.planner.TargetReached(robot.planner.remaining_path, pose[0], pose[1]):
                print("[FSM] DONE — Goal reached")
                robot.stop()
                show_idle_leds(robot)
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
