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
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 80.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Actuator Configuration
# ---------------------------------------------------------------------------
ARM_STEPPER        = Stepper.STEPPER_1
ARM_UP_STEPS       = -700
ARM_MAX_VELOCITY   = 400    
ARM_ACCELERATION   = 200    

GRIPPER_CHANNEL    = ServoChannel.CH_1
GRIPPER_OPEN_DEG   = 120.0
GRIPPER_CLOSE_DEG  = 80.0
GRIPPER_CLOSE_DEG_MEAT = 64

CAMERA_CHANNEL     = ServoChannel.CH_2
CAMERA_SCAN_DEG    = 95.0
CAMERA_DEFAULT_DEG = 60.0

# ---------------------------------------------------------------------------
# Vision & Drive Constants
# ---------------------------------------------------------------------------
VELOCITY_MM_S = 150.0
TOLERANCE_MM = 50.0
TABLE_THRESHOLD_MM = 400.0
CONSISTENT_READINGS_REQ = 3

# P-Control Wall Following
FOLLOW_TARGET_MM = 390.0
FOLLOW_RIGHT_TARGET_MM = 370.0
FOLLOW_DISTANCE_MM = 550.0
FOLLOW_KP = 0.3  

FIRST_RIGHT_FORWARD = 140.0
FIRST_LEFT_FORWARD = 195.0
SECOND_RIGHT_FORWARD = 165.0

# Pure Pursuit tuning for straights
LOOKAHEAD_MM = 100.0
MAX_ANGULAR_RAD_S = 1.0

VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0  

def find_traffic_light_color(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best_color = None
    best_confidence = -1.0
    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue
        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)
    return best_color

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_lidar()
    robot.set_lidar_filter(range_min_mm=0.0)
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

def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    drive_handle = None
    last_shown_color = None
    lights_off_at = 0.0
    table_detect_count = 0
    start_pose = None
    final_front_dist = 0.0
    
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[STARTUP] Initialized. Press BTN_9 to start Burger Assembly V2 (Pure Pursuit Straights).")

    while True:
        now = time.monotonic()

        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
            robot.stop() 
            state = "IDLE"
            print("[FSM] Stopped. Returning to IDLE.")

        if state == "INIT":
            start_robot(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            robot.enable_servo(GRIPPER_CHANNEL)
            robot.enable_servo(CAMERA_CHANNEL)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_9):
                print(f"[ACTION] Opening gripper to {GRIPPER_OPEN_DEG}°")
                robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
                state = "CAMERA_SCAN"

        elif state == "CAMERA_SCAN":
            print(f"[ACTION] Turning camera to {CAMERA_SCAN_DEG}° to scan light")
            robot.set_servo(CAMERA_CHANNEL, CAMERA_SCAN_DEG)
            state = "WATCHING"

        elif state == "WATCHING":
            traffic_light_color = find_traffic_light_color(robot)
            if traffic_light_color in ("red", "green"):
                lights_off_at = now + LIGHT_HOLD_SEC  
                if traffic_light_color == "red":
                    robot.set_led(LED.RED, LED_BRIGHTNESS)
                    robot.set_led(LED.GREEN, 0)
                    robot.stop() 
                elif traffic_light_color == "green":
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    robot.set_servo(CAMERA_CHANNEL, CAMERA_DEFAULT_DEG)
                    robot.step_enable(ARM_STEPPER)
                    robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=False)
                    drive_handle = None 
                    table_detect_count = 0
                    state = "DRIVE_SEARCH_TABLE"
                last_shown_color = traffic_light_color
            elif lights_off_at > 0.0 and now >= lights_off_at:
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 0)
                lights_off_at = 0.0
                last_shown_color = None

        elif state == "DRIVE_SEARCH_TABLE":
            if drive_handle is None:
                # Use Pure Pursuit for long search drive
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                target_x = curr_x + 2000.0 * math.cos(math.radians(curr_theta))
                target_y = curr_y + 2000.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(target_x, target_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=TOLERANCE_MM,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            if dist is not None and dist < TABLE_THRESHOLD_MM:
                table_detect_count += 1
            else:
                table_detect_count = 0
            if table_detect_count >= CONSISTENT_READINGS_REQ:
                if drive_handle is not None:
                    drive_handle.cancel()
                robot.stop()
                drive_handle = None
                start_pose = robot.get_odometry_pose()
                state = "DRIVE_WALL_FOLLOW"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "IDLE"

        elif state == "DRIVE_WALL_FOLLOW":
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
                state = "TURN_LEFT_90"

        elif state == "TURN_LEFT_90":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_WALL"

        elif state == "ALIGN_FRONT_WALL":
            if robot_align_front(robot): pass
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                state = "FINAL_APPROACH"
            else:
                state = "IDLE"

        elif state == "FINAL_APPROACH":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    # Use Pure Pursuit for final approach
                    curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                    t_x = curr_x + target_fwd * math.cos(math.radians(curr_theta))
                    t_y = curr_y + target_fwd * math.sin(math.radians(curr_theta))
                    drive_handle = robot.purepursuit_follow_path(
                        waypoints=[(t_x, t_y)],
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=5.0,
                        max_angular_rad_s=MAX_ANGULAR_RAD_S,
                        blocking=False
                    )
                else:
                    state = "LOWER_ARM"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "LOWER_ARM"

        elif state == "LOWER_ARM":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=20.0):
                state = "CLOSE_GRIPPER"
            else:
                state = "IDLE"

        elif state == "CLOSE_GRIPPER":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            time.sleep(2.0)  
            state = "RAISE_ARM_FINAL"

        elif state == "RAISE_ARM_FINAL":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=True, timeout=20.0):
                state = "DRIVE_BACK_100"
            else:
                state = "IDLE"

        elif state == "DRIVE_BACK_100":
            if drive_handle is None:
                # Use Pure Pursuit for reverse
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x - 100.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y - 100.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90"

        elif state == "TURN_RIGHT_90":
            if drive_handle is None:
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_155"

        elif state == "DRIVE_FWD_155":
            if drive_handle is None:
                # Use Pure Pursuit
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x + FIRST_RIGHT_FORWARD * math.cos(math.radians(curr_theta))
                t_y = curr_y + FIRST_RIGHT_FORWARD * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_STAGE2"

        elif state == "TURN_LEFT_90_STAGE2":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_STAGE2"

        elif state == "ALIGN_FRONT_STAGE2":
            if robot_align_front(robot): pass
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                state = "FINAL_APPROACH_STAGE2"
            else:
                state = "OPEN_GRIPPER_POST"
            
        elif state == "FINAL_APPROACH_STAGE2":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                    t_x = curr_x + target_fwd * math.cos(math.radians(curr_theta))
                    t_y = curr_y + target_fwd * math.sin(math.radians(curr_theta))
                    drive_handle = robot.purepursuit_follow_path(
                        waypoints=[(t_x, t_y)],
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=5.0,
                        max_angular_rad_s=MAX_ANGULAR_RAD_S,
                        blocking=False
                    )
                else:
                    state = "OPEN_GRIPPER_POST"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "OPEN_GRIPPER_POST"

        elif state == "OPEN_GRIPPER_POST":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
            time.sleep(1.0)
            state = "DRIVE_BACK_120_POST"

        elif state == "DRIVE_BACK_120_POST":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x - 120.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y - 120.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_POST"

        elif state == "TURN_LEFT_90_POST":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_POST"

        elif state == "DRIVE_FWD_150_POST":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x + 105.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y + 105.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_2_POST"

        elif state == "DRIVE_FWD_150_2_POST":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x + FIRST_LEFT_FORWARD * math.cos(math.radians(curr_theta))
                t_y = curr_y + FIRST_LEFT_FORWARD * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_POST"

        elif state == "TURN_RIGHT_90_POST":
            if drive_handle is None:
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_POST"

        elif state == "ALIGN_FRONT_POST":
            if robot_align_front(robot): pass
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                state = "FINAL_APPROACH_POST"
            else:
                state = "LOWER_ARM_FINAL"

        elif state == "FINAL_APPROACH_POST":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                    t_x = curr_x + target_fwd * math.cos(math.radians(curr_theta))
                    t_y = curr_y + target_fwd * math.sin(math.radians(curr_theta))
                    drive_handle = robot.purepursuit_follow_path(
                        waypoints=[(t_x, t_y)],
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=5.0,
                        max_angular_rad_s=MAX_ANGULAR_RAD_S,
                        blocking=False
                    )
                else:
                    state = "LOWER_ARM_FINAL"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "LOWER_ARM_FINAL"

        elif state == "LOWER_ARM_FINAL":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=20.0):
                state = "CLOSE_GRIPPER_FINAL"
            else:
                state = "IDLE"

        elif state == "CLOSE_GRIPPER_FINAL":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
            time.sleep(2.0)
            state = "RAISE_ARM_DOUBLE_FINAL"

        elif state == "RAISE_ARM_DOUBLE_FINAL":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS * 2, blocking=True, timeout=30.0):
                state = "DRIVE_BACK_FINAL"
            else:
                state = "IDLE"

        elif state == "DRIVE_BACK_FINAL":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x - 100.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y - 100.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_END"

        elif state == "TURN_RIGHT_90_END":
            if drive_handle is None:
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_END"

        elif state == "DRIVE_FWD_150_END":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x + 150.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y + 150.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_2_END"

        elif state == "DRIVE_FWD_150_2_END":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x + SECOND_RIGHT_FORWARD * math.cos(math.radians(curr_theta))
                t_y = curr_y + SECOND_RIGHT_FORWARD * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_END"

        elif state == "TURN_LEFT_90_END":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_END"

        elif state == "ALIGN_FRONT_END":
            if robot_align_front(robot): pass
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                state = "FINAL_APPROACH_END"
            else:
                state = "LOWER_ARM_QUARTER_END"

        elif state == "FINAL_APPROACH_END":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                    t_x = curr_x + target_fwd * math.cos(math.radians(curr_theta))
                    t_y = curr_y + target_fwd * math.sin(math.radians(curr_theta))
                    drive_handle = robot.purepursuit_follow_path(
                        waypoints=[(t_x, t_y)],
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=5.0,
                        max_angular_rad_s=MAX_ANGULAR_RAD_S,
                        blocking=False
                    )
                else:
                    state = "LOWER_ARM_QUARTER_END"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "LOWER_ARM_QUARTER_END"

        elif state == "LOWER_ARM_QUARTER_END":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS // 4, blocking=True, timeout=10.0):
                state = "OPEN_GRIPPER_END"
            else:
                state = "IDLE"

        elif state == "OPEN_GRIPPER_END":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
            time.sleep(1.0)
            state = "LOWER_ARM_DOUBLE_END"

        elif state == "LOWER_ARM_DOUBLE_END":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS * 2, blocking=True, timeout=40.0):
                state = "CLOSE_GRIPPER_END"
            else:
                state = "IDLE"

        elif state == "CLOSE_GRIPPER_END":
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
            time.sleep(2.0)
            state = "RAISE_ARM_FINAL_END"

        elif state == "RAISE_ARM_FINAL_END":
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=True, timeout=20.0):
                state = "DRIVE_BACK_FINAL_END"
            else:
                state = "IDLE"

        elif state == "DRIVE_BACK_FINAL_END":
            if drive_handle is None:
                curr_x, curr_y, curr_theta = robot.get_odometry_pose()
                t_x = curr_x - 100.0 * math.cos(math.radians(curr_theta))
                t_y = curr_y - 100.0 * math.sin(math.radians(curr_theta))
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=[(t_x, t_y)],
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=5.0,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False
                )
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_FINAL"

        elif state == "TURN_RIGHT_90_FINAL":
            if drive_handle is None:
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Sequence complete. All tasks finished.")
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
