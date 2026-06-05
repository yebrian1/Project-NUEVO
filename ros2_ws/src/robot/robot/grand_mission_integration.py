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
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.lidar_helpers import (
    get_left_distance,
    get_right_distance,
    get_front_distance,
    robot_align_left,
    robot_align_right,
    robot_align_front,
    robot_align_to_wall,
)

# ---------------------------------------------------------------------------
# 1. TUNED CONFIGURATION (Combined & Hardcoded)
# ---------------------------------------------------------------------------

# Robot Physicals
TAG_ID = 21 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 80.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# Actuators (Tuned for Burger Mission)
ARM_STEPPER        = Stepper.STEPPER_1
ARM_UP_STEPS       = -700    # Tuned height for burger clearance
ARM_MAX_VELOCITY   = 400    
ARM_ACCELERATION   = 200    

GRIPPER_CHANNEL    = ServoChannel.CH_1
GRIPPER_OPEN_DEG   = 120.0
GRIPPER_CLOSE_DEG_MEAT = 64 # Tuned for secure grip

CAMERA_CHANNEL     = ServoChannel.CH_2
CAMERA_DEFAULT_DEG = 60.0

# Navigation Constants (Tuned for Reliability)
VELOCITY_MM_S = 150.0
TOLERANCE_MM = 10.0
TURN_VELOCITY_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5

# Wall Following (Tuned for Burger Table)
TABLE_THRESHOLD_MM = 400.0
CONSISTENT_READINGS_REQ = 3
FOLLOW_TARGET_MM = 390.0
FOLLOW_DISTANCE_MM = 545.0
FOLLOW_KP = 0.3

# Pure Pursuit (Tuned for Maze)
LOOKAHEAD_MM       = 250.0  
PP_TOLERANCE_MM    = 80.0   
ADVANCE_RADIUS_MM  = 250.0  
MAX_ANGULAR_RAD_S  = 1.0    
PATH_CONTROL_POINTS = [
    (0, 1300),
    (0, 3900),  
]

# Obstacle Avoidance (LAPF Tuned)
LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LEASH_LENGTH_MM = 350.0 
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

# Vision/GPS
ENABLE_GPS = True
VISION_STALE_SEC = 3.0
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# 2. HELPER FUNCTIONS
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    
    # Lidar configuration for obstacle avoidance
    robot.set_lidar_mount(x_mm=LIDAR_MOUNT_X_MM, y_mm=LIDAR_MOUNT_Y_MM, theta_deg=LIDAR_MOUNT_THETA_DEG)
    robot.set_lidar_filter(range_min_mm=LIDAR_RANGE_MIN_MM, range_max_mm=LIDAR_RANGE_MAX_MM, fov_deg=LIDAR_FOV_DEG)
    robot.start_lidar_world_publisher()

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER, wheel_base=WHEEL_BASE, initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR, left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR, right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID)
    robot.enable_gps_tangent_heading(alpha=0.01, min_displacement_mm=200.0)
    robot.set_position_fusion_alpha(0.01)

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
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

def resolve_lapf_config() -> dict[str, float]:
    return {
        "leash_length_mm": float(LEASH_LENGTH_MM),
        "repulsion_range_mm": float(REPULSION_RANGE_MM),
        "target_speed_mm_s": float(TARGET_SPEED_MM_S),
        "repulsion_gain": float(REPULSION_GAIN),
        "attraction_gain": float(ATTRACTION_GAIN),
        "force_ema_alpha": float(FORCE_EMA_ALPHA),
        "inflation_margin_mm": float(INFLATION_MARGIN_MM),
        "leash_half_angle_deg": float(LEASH_HALF_ANGLE_DEG),
    }

# ---------------------------------------------------------------------------
# 3. MAIN FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    drive_handle = None
    table_detect_count = 0
    start_pose = None
    final_front_dist = 0.0
    seq_step = 0
    
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[GRAND MISSION] Initialized. Press BTN_9 to start.")

    while True:
        now = time.monotonic()

        # Global Kill Switch (BTN_2)
        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
            robot.stop() 
            state = "IDLE"
            show_idle_leds(robot)
            print("[FSM] Stopped. Returning to IDLE.")

        # --- STATE MACHINE ---

        if state == "INIT":
            start_robot(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            robot.enable_servo(GRIPPER_CHANNEL)
            robot.enable_servo(CAMERA_CHANNEL)
            robot.set_servo(CAMERA_CHANNEL, CAMERA_DEFAULT_DEG)
            show_idle_leds(robot)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_9):
                print("[ACTION] Mission Start! Lifting arm, opening gripper and starting search.")
                
                # Lift arm to clear obstacles (Non-blocking)
                robot.step_enable(ARM_STEPPER)
                robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=False)
                
                robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
                show_running_leds(robot)
                # Immediately drive forward to search for table (Skipping Traffic Light)
                drive_handle = robot.move_forward(2000.0, velocity=VELOCITY_MM_S, tolerance=TOLERANCE_MM, blocking=False)
                state = "BURGER_SEARCH_TABLE"

        # -------------------------------------------------------------------
        # PHASE 1: BURGER PICKUP (From FINAL_burger_assembly)
        # -------------------------------------------------------------------

        elif state == "BURGER_SEARCH_TABLE":
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            
            if dist is not None and dist < TABLE_THRESHOLD_MM:
                table_detect_count += 1
            else:
                table_detect_count = 0

            if table_detect_count >= CONSISTENT_READINGS_REQ:
                print("[SEARCH] Table confirmed! Starting wall-following.")
                if drive_handle is not None: drive_handle.cancel()
                robot.stop()
                drive_handle = None
                start_pose = robot.get_odometry_pose()
                state = "BURGER_WALL_FOLLOW"
            elif drive_handle.is_finished():
                print("[WARN] Table not found. Stopping.")
                robot.stop()
                state = "IDLE"

        elif state == "BURGER_WALL_FOLLOW":
            curr_pose = robot.get_odometry_pose()
            dist_traveled = math.hypot(curr_pose[0] - start_pose[0], curr_pose[1] - start_pose[1])
            
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            
            if dist is not None and count > 0:
                error_mm = dist - FOLLOW_TARGET_MM
                angular_cmd = error_mm * FOLLOW_KP
                angular_cmd = max(-20.0, min(20.0, angular_cmd))
                robot.set_velocity(VELOCITY_MM_S, angular_cmd)
            else:
                robot.set_velocity(VELOCITY_MM_S, 0.0)

            if dist_traveled >= FOLLOW_DISTANCE_MM:
                robot.stop()
                print(f"[FSM] Wall-follow complete. Turning 90° to face table.")
                state = "BURGER_TURN_TO_TABLE"

        elif state == "BURGER_TURN_TO_TABLE":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "BURGER_ALIGN_FRONT"

        elif state == "BURGER_ALIGN_FRONT":
            if robot_align_front(robot):
                points = np.asarray(robot.get_obstacles())
                f_dist, _ = get_front_distance(points)
                if f_dist is not None:
                    final_front_dist = f_dist
                    print(f"[FSM] Aligned. Front Dist: {final_front_dist:.1f}mm. Moving to pick.")
                    state = "BURGER_FINAL_APPROACH"
                else:
                    state = "IDLE"
            else:
                state = "IDLE"

        elif state == "BURGER_FINAL_APPROACH":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "BURGER_LOWER_ARM"

        elif state == "BURGER_LOWER_ARM":
            robot.step_enable(ARM_STEPPER)
            # Lower arm from home position (Assuming arm starts at 0 and UP is negative)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=10.0):
                state = "BURGER_CLOSE_GRIPPER"
            else:
                state = "IDLE"

        elif state == "BURGER_CLOSE_GRIPPER":
            print(f"[ACTION] Gripping burger at {GRIPPER_CLOSE_DEG_MEAT}°")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            time.sleep(2.0) # Ensure grip is secure
            state = "BURGER_RAISE_ARM"

        elif state == "BURGER_RAISE_ARM":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=True, timeout=10.0):
                print("[FSM] Burger secured. Transitioning to GPS Navigation.")
                state = "NAV_RETRACT_BACK"
            else:
                state = "IDLE"

        elif state == "NAV_RETRACT_BACK":
            if drive_handle is None:
                drive_handle = robot.move_backward(150.0, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "NAV_GPS_LOCK"

        # -------------------------------------------------------------------
        # PHASE 2: COMPLEX NAVIGATION (From integrated_obstacle_avoidance.py)
        # -------------------------------------------------------------------

        elif state == "NAV_GPS_LOCK":
            print("[GPS] Waiting for lock (Tag 21) while holding burger...")
            gps_wait = time.monotonic()
            gps_locked = False
            while time.monotonic() - gps_wait < 15.0:
                if robot.is_gps_active():
                    gps_locked = True
                    break
                time.sleep(0.1)
            
            if gps_locked:
                robot.set_position_fusion_alpha(1.0)
                time.sleep(0.5)
                robot.set_position_fusion_alpha(0.01)
                print("[GPS] Synced. Starting Maze Pursuit.")
            else:
                print("[WARN] GPS Lock failed, using Odom only.")
            
            drive_handle = robot.purepursuit_follow_path(
                waypoints=PATH_CONTROL_POINTS, velocity=VELOCITY_MM_S, lookahead=LOOKAHEAD_MM,
                tolerance=PP_TOLERANCE_MM, advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S, blocking=False,
            )
            state = "NAV_MAZE_DRIVING"

        elif state == "NAV_MAZE_DRIVING":
            # Constant Gripper Pressure
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            
            if drive_handle is not None and drive_handle.is_finished():
                robot.stop()
                print("[FSM] Maze complete. Aligning for Turn Sequence.")
                robot_align_to_wall(robot, 90.0, 0.0)
                seq_step = 0
                state = "NAV_TURN_SEQUENCE"

        elif state == "NAV_TURN_SEQUENCE":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            if seq_step == 0:
                drive_handle = robot.turn_by(-90.0, blocking=False)
                seq_step = 1
            elif seq_step == 1 and drive_handle.is_finished():
                drive_handle = robot.move_forward(600.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                seq_step = 2
            elif seq_step == 2 and drive_handle.is_finished():
                drive_handle = robot.turn_by(-90.0, blocking=False)
                seq_step = 3
            elif seq_step == 3 and drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                seq_step = 0
                state = "NAV_RAMP_SEQUENCE"

        elif state == "NAV_RAMP_SEQUENCE":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            if seq_step == 0:
                drive_handle = robot.move_forward(450.0, velocity=VELOCITY_MM_S, tolerance=10.0, blocking=False)
                seq_step = 1
            elif seq_step == 1 and drive_handle.is_finished():
                robot_align_to_wall(robot, 180.0, 90.0)
                seq_step = 2
            elif seq_step == 2:
                # LONG RAMP CLIMB
                drive_handle = robot.move_forward(2715.0, velocity=VELOCITY_MM_S, tolerance=50.0, blocking=False)
                seq_step = 3
            elif seq_step == 3 and drive_handle.is_finished():
                robot_align_to_wall(robot, 90.0, 0.0)
                robot.stop()
                seq_step = 0
                state = "NAV_TURN_SEQUENCE_2"

        elif state == "NAV_TURN_SEQUENCE_2":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            if seq_step == 0:
                drive_handle = robot.turn_by(90.0, blocking=False)
                seq_step = 1
            elif seq_step == 1 and drive_handle.is_finished():
                drive_handle = robot.move_forward(600.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                seq_step = 2
            elif seq_step == 2 and drive_handle.is_finished():
                drive_handle = robot.turn_by(90.0, blocking=False)
                seq_step = 3
            elif seq_step == 3 and drive_handle.is_finished():
                drive_handle = robot.move_forward(200.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                seq_step = 4
            elif seq_step == 4 and drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "NAV_OBSTACLE_AVOIDANCE"

        elif state == "NAV_OBSTACLE_AVOIDANCE":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            if drive_handle is None:
                cfg = resolve_lapf_config()
                # Assuming the goal is the same as in the integrated script
                drive_handle = robot.lapf_to_goal(
                    0.0, 3050.0, # 610*5
                    velocity=LAPF_VELOCITY_MM_S, tolerance=LAPF_TOLERANCE_MM,
                    leash_length_mm=cfg["leash_length_mm"], repulsion_range_mm=cfg["repulsion_range_mm"],
                    target_speed_mm_s=cfg["target_speed_mm_s"], max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    repulsion_gain=cfg["repulsion_gain"], attraction_gain=cfg["attraction_gain"],
                    force_ema_alpha=cfg["force_ema_alpha"], inflation_margin_mm=cfg["inflation_margin_mm"],
                    leash_half_angle_deg=cfg["leash_half_angle_deg"], blocking=False,
                )
            elif drive_handle.is_finished():
                print("[FSM] MISSION COMPLETE. Burger delivered!")
                robot.stop()
                show_idle_leds(robot)
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

if __name__ == "__main__":
    from robot.robot_node import RobotNode
    import rclpy
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
