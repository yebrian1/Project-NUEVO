from __future__ import annotations
import time
import math
import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    Stepper,
    ServoChannel,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.path_planner2 import PurePursuitPlanner, generate_maze_waypoints

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# LIDAR Helper Functions
# ---------------------------------------------------------------------------
def get_front_distance(points: np.ndarray, fov_deg: float = 5.0) -> tuple[float | None, int]:
    if points.size == 0: return None, 0
    x, y = points[:, 0], points[:, 1]
    front_mask = x > 0
    xf, yf = x[front_mask], y[front_mask]
    if xf.size == 0: return None, 0
    angles = np.arctan2(yf, xf)
    half_fov_rad = math.radians(fov_deg / 2.0)
    fov_mask = np.abs(angles) <= half_fov_rad
    final_points = xf[fov_mask]
    if final_points.size == 0: return None, 0
    return float(np.mean(final_points)), final_points.size

def get_wall_alignment(points: np.ndarray, fov_deg: float = 20.0) -> dict | None:
    if points.size == 0: return None
    x, y = points[:, 0], points[:, 1]
    angles = np.arctan2(y, x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    mask = (x > 0) & (np.abs(angles) <= half_fov_rad)
    xf, yf = x[mask], y[mask]
    if xf.size < 5: return None
    m, c = np.polyfit(yf, xf, 1)
    tilt_rad = math.atan(m)
    predicted_x = m * yf + c
    residuals = xf - predicted_x
    std_err = float(np.sqrt(np.sum(residuals**2) / xf.size))
    return {'tilt_deg': float(math.degrees(tilt_rad)), 'points': int(xf.size), 'error': std_err}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    robot.enable_imu()
    robot.set_lidar_filter(range_min_mm=0.0)
    
    # Fusion alpha set to 0.3
    robot.set_orientation_fusion_alpha(0.3) 

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER, 
        wheel_base=WHEEL_BASE, 
        initial_theta_deg=INITIAL_THETA_DEG, 
        left_motor_id=LEFT_WHEEL_MOTOR, 
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED, 
        right_motor_id=RIGHT_WHEEL_MOTOR, 
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED
    )
    robot.set_tracked_tag_id(TAG_ID)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR): robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    remaining_path = []
    planner1 = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        raw_lidar = robot.get_obstacles()
        points = np.asarray(raw_lidar)
        if state not in ["INIT", "IDLE"] and robot.get_button(Button.BTN_2):
            robot.stop() 
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            print("\nMASTER FSM LOADED\nBTN 1: DIST | BTN 2: TILT | BTN 9: START MAZE\n")
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            if robot.was_button_pressed(Button.BTN_1):
                avg_dist, count = get_front_distance(points, fov_deg=5.0)
                print(f"[DISTANCE] Avg: {avg_dist:.2f} mm" if avg_dist else "[DISTANCE] No points")
            elif robot.was_button_pressed(Button.BTN_2):
                res = get_wall_alignment(points)
                print(f"[TILT] {res['tilt_deg']:.2f} deg" if res else "[TILT] No wall")
            elif robot.get_button(Button.BTN_9):
                state = "CMD_MAZE_START"

        elif state == "CMD_MAZE_START":
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.GREEN, 200)
            
            print("[GPS] Checking for Tag 21 lock...")
            gps_wait = time.monotonic()
            while time.monotonic() - gps_wait < 3.0:
                if robot.is_gps_active(): break
                time.sleep(0.1)

            print("[ACTION] Alignment: Back 70mm -> Turn Right 90°")
            robot.move_backward(70.0, 100.0, 20.0, blocking=False).wait(timeout=10.0)
            robot.turn_by(-90.0, blocking=False).wait(timeout=10.0)
            
            print("[GPS] Stabilizing...")
            time.sleep(1.0)
            
            cur_x, cur_y, odom_theta_rad = robot.get_pose()
            cur_theta_fused = robot.get_fused_orientation()
            
            # Odometry correctly tracked the 90° turn; fused heading did not get
            # the initial offset. Use odom for waypoint generation.
            cur_theta_for_path = math.degrees(odom_theta_rad)  # ~0° after the turn
            
            print(f"[GPS] Origin: ({cur_x:.1f}, {cur_y:.1f})")
            print(f"[GPS] Using odom heading {cur_theta_for_path:.1f}° for path (fused was {cur_theta_fused:.1f}°)")
            remaining_path = generate_maze_waypoints(cur_x, cur_y, cur_theta_for_path)
            
            planner1 = PurePursuitPlanner(lookahead_dist=150.0, max_angular=0.8, goal_tolerance=50.0)
            state = "EXEC_MAZE_DRIVING"

        elif state == "EXEC_MAZE_DRIVING":
            current_x, current_y, odom_theta_rad = robot.get_pose()
            
            # Use odom heading — fused heading has a ~90° initialization offset
            # that corrupts the planner's robot-frame transform
            current_theta_rad = odom_theta_rad
            
            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y, advance_radius_mm=100.0
            )
            if not remaining_path:
                print("[DONE] Maze complete.")
                robot.stop()
                state = "IDLE"
                continue
            
            linear, angular = planner1.compute_velocity(
                (current_x, current_y, current_theta_rad), remaining_path, 110.0
            )
            robot.set_velocity(linear, math.degrees(angular))
            
            if planner1.CurrentTargetReached(
                remaining_path[-1][0], remaining_path[-1][1], current_x, current_y
            ):
                robot.stop()
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0: time.sleep(sleep_s)
        else: next_tick = time.monotonic()

if __name__ == "__main__":
    from robot.robot_node import RobotNode
    import rclpy
    rclpy.init()
    node = RobotNode()
    robot = Robot(node)
    try: run(robot)
    except KeyboardInterrupt: pass
    finally:
        robot.stop()
        rclpy.shutdown()
