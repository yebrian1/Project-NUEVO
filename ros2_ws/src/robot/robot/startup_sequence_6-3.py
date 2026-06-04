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
)
from robot.robot import FirmwareState, Robot, Unit
from robot.lidar_helpers import (
    get_left_distance,
    get_front_distance,
    robot_align_left,
    robot_align_front,
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
ARM_UP_STEPS       = -2000
ARM_MAX_VELOCITY   = 400    
ARM_ACCELERATION   = 200    

# ---------------------------------------------------------------------------
# Vision & Drive Constants
# ---------------------------------------------------------------------------
VELOCITY_MM_S = 150.0
TOLERANCE_MM = 50.0
TABLE_THRESHOLD_MM = 400.0
CONSISTENT_READINGS_REQ = 3

# P-Control Wall Following
FOLLOW_TARGET_MM = 390.0
FOLLOW_DISTANCE_MM = 640.0
FOLLOW_KP = 0.3  # Angular speed (deg/s) per mm of distance error

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

    print("[STARTUP] Initialized. Press BTN_9 to start.")

    while True:
        now = time.monotonic()

        # BTN_2 Kill Switch
        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
            robot.stop() 
            state = "IDLE"
            print("[FSM] Stopped. Returning to IDLE.")

        if state == "INIT":
            start_robot(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_9):
                state = "INITIAL_TURN"

        elif state == "INITIAL_TURN":
            if drive_handle is None:
                print("[ACTION] Turning 20° Left to scan light")
                drive_handle = robot.turn_by(25.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                last_shown_color = None
                lights_off_at = 0.0
                state = "WATCHING"

        elif state == "WATCHING":
            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color in ("red", "green"):
                lights_off_at = now + LIGHT_HOLD_SEC  
                
                if traffic_light_color == "red":
                    robot.set_led(LED.RED, LED_BRIGHTNESS)
                    robot.set_led(LED.GREEN, 0)
                    robot.stop() 
                    if traffic_light_color != last_shown_color:
                        print("[VISION] Traffic Light: RED. Waiting...")
                
                elif traffic_light_color == "green":
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    print("[VISION] Traffic Light: GREEN. Lifting arm and straightening up.")
                    
                    # Start lifting arm while we turn (non-blocking)
                    robot.step_enable(ARM_STEPPER)
                    robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=False)
                    
                    drive_handle = None 
                    state = "TURN_BACK"
                
                last_shown_color = traffic_light_color

            elif lights_off_at > 0.0 and now >= lights_off_at:
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 0)
                lights_off_at = 0.0
                last_shown_color = None

        elif state == "TURN_BACK":
            if drive_handle is None:
                drive_handle = robot.turn_by(-25.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Orientation straightened. Searching for table...")
                table_detect_count = 0
                state = "DRIVE_SEARCH_TABLE"

        elif state == "DRIVE_SEARCH_TABLE":
            if drive_handle is None:
                # Long drive to find table
                drive_handle = robot.move_forward(2000.0, velocity=VELOCITY_MM_S, tolerance=TOLERANCE_MM, blocking=False)
            
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            
            if dist is not None and dist < TABLE_THRESHOLD_MM:
                table_detect_count += 1
                print(f"[SEARCH] Table detected ({table_detect_count}/{CONSISTENT_READINGS_REQ}): {dist:.1f} mm")
            else:
                table_detect_count = 0

            if table_detect_count >= CONSISTENT_READINGS_REQ:
                print("[SEARCH] Table confirmed! Starting wall-following drive.")
                if drive_handle is not None:
                    drive_handle.cancel()
                robot.stop()
                drive_handle = None
                start_pose = robot.get_odometry_pose()
                state = "DRIVE_WALL_FOLLOW"
            elif drive_handle.is_finished():
                print("[WARN] Reached end of search distance without finding table.")
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
                
                if np.random.rand() < 0.1:
                    print(f"[FOLLOW] Dist:{dist:.1f}mm | Err:{error_mm:+.1f}mm | Traveled:{dist_traveled:.0f}mm")
            else:
                robot.set_velocity(VELOCITY_MM_S, 0.0)

            if dist_traveled >= FOLLOW_DISTANCE_MM:
                robot.stop()
                print(f"[FSM] Wall-following complete ({dist_traveled:.0f}mm). Turning left 90°.")
                state = "TURN_LEFT_90"

        elif state == "TURN_LEFT_90":
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Turn complete. Aligning with front wall.")
                state = "ALIGN_FRONT_WALL"

        elif state == "ALIGN_FRONT_WALL":
            if robot_align_front(robot):
                print("[FSM] Front alignment successful.")
            else:
                print("[WARN] Front alignment failed.")
            
            # Final distance report
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                print(f"[REPORT] Final Front Distance: {final_front_dist:.1f} mm. Driving forward {final_front_dist - 20.0:.1f}mm.")
                state = "FINAL_APPROACH"
            else:
                print("[WARN] Front distance unknown. Returning to IDLE.")
                state = "IDLE"

        elif state == "FINAL_APPROACH":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
                else:
                    state = "LOWER_ARM"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Final approach complete. Lowering arm.")
                state = "LOWER_ARM"

        elif state == "LOWER_ARM":
            # Lower arm by the exact amount raised (-ARM_UP_STEPS)
            print("[ACTION] Lowering arm...")
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=10.0):
                print("[FSM] Arm lowered. Startup sequence complete.")
            else:
                print("[ERROR] Stepper error during arm lowering.")
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
