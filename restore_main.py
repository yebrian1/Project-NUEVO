import os

content = """from __future__ import annotations
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
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.path_planner2 import PurePursuitPlanner, generate_maze_waypoints
from robot.face_helpers import CustomerClassifier

from robot.lidar_helpers import (
    get_left_distance,
    get_right_distance,
    get_front_distance,
    robot_align_left,
    robot_align_right,
    robot_align_front,
    robot_align_to_wall,
    robot_approach_wall,
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
ARM_MAX_VELOCITY   = 400    # Reverted to fast version
ARM_ACCELERATION   = 200    # Reverted to fast version

GRIPPER_CHANNEL    = ServoChannel.CH_1
# Using UI-aligned degrees: User is tweaking these manually, leaving as-is.
GRIPPER_OPEN_DEG   = 120.0
GRIPPER_CLOSE_DEG  = 80.0
GRIPPER_CLOSE_DEG_MEAT = 67

CAMERA_CHANNEL     = ServoChannel.CH_2
CAMERA_SCAN_DEG    = 95.0
CAMERA_DEFAULT_DEG = 60.0

# ---------------------------------------------------------------------------
# Pure Pursuit Configuration
# ---------------------------------------------------------------------------
VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 250.0  
TOLERANCE_MM       = 80.0   
ADVANCE_RADIUS_MM  = 250.0  
MAX_ANGULAR_RAD_S  = 1.0    

STATUS_PRINT_INTERVAL_S = 0.5

# Simplified path to avoid tethering issues at corners
PATH_CONTROL_POINTS = [
    (0, 1300),
    (0, 3950),  
]

RUNBACK = False

# ---------------------------------------------------------------------------
# Obstacle Avoidance Configuration (from obstacleavoidance7final.py)
# ---------------------------------------------------------------------------
LAPF_GOAL_MM = (0, 2000)
LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 1.0

# LAPF behavior tuning
LEASH_LENGTH_MM = 350.0 
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 150.0
REPULSION_GAIN = 450.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.25
#increasing Force
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

# ---------------------------------------------------------------------------
# Vision & Drive Constants
# ---------------------------------------------------------------------------
VELOCITY_MM_S = 100.0
TOLERANCE_MM = 10.0
TURN_VELOCITY_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5
TABLE_THRESHOLD_MM = 400.0
CONSISTENT_READINGS_REQ = 3

# P-Control Wall Following
FOLLOW_TARGET_MM = 390.0
FOLLOW_RIGHT_TARGET_MM = 370.0
FOLLOW_DISTANCE_MM = 560.0
FOLLOW_KP = 0.3  # Angular speed (deg/s) per mm of distance error

FIRST_RIGHT_FORWARD = 140.0
FIRST_LEFT_FORWARD = 195.0
SECOND_RIGHT_FORWARD = 170.0

ENABLE_GPS = True
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0  

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
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
def configure_robot_for_lapf(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    
    # Lidar configuration for obstacle avoidance
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    robot.start_lidar_world_publisher()

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID)
    robot.enable_gps_tangent_heading(alpha=0.05, min_displacement_mm=200.0)
    robot.set_position_fusion_alpha(0.05)

def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")

def print_lapf_status(robot: Robot) -> None:
    if robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "

    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()
    if virtual_target is None:
        vt_summary = " vt=(none)"
    else:
        vt_summary = f" vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    if obstacle_tracks:
        nearest_boundary_mm = min(
            max(
                0.0,
                ((float(track["x"]) - x) ** 2 + (float(track["y"]) - y) ** 2) ** 0.5
                - float(track["radius"]),
            )
            for track in obstacle_tracks
        )
        track_summary = f" tracked={len(obstacle_tracks)} nearest_track={nearest_boundary_mm:.0f} mm"
    else:
        track_summary = " tracked=0"

    print(
        f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°"
        f"{vt_summary}{track_summary}"
    )

def start_lapf_goal(robot: Robot):
    cfg = resolve_lapf_config()
    return robot.lapf_to_goal(
        LAPF_GOAL_MM[0],
        LAPF_GOAL_MM[1],
        velocity=LAPF_VELOCITY_MM_S,
        tolerance=LAPF_TOLERANCE_MM,
        leash_length_mm=cfg["leash_length_mm"],
        repulsion_range_mm=cfg["repulsion_range_mm"],
        target_speed_mm_s=cfg["target_speed_mm_s"],
        max_angular_rad_s=LAPF_MAX_ANGULAR_RAD_S,
        repulsion_gain=cfg["repulsion_gain"],
        attraction_gain=cfg["attraction_gain"],
        force_ema_alpha=cfg["force_ema_alpha"],
        inflation_margin_mm=cfg["inflation_margin_mm"],
        leash_half_angle_deg=cfg["leash_half_angle_deg"],
        blocking=False,
    )

def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    #turn off gps
    robot.disable_gps()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

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
    # Initialize classifier
    classifier = CustomerClassifier(robot)
    
    state = "INIT"
    drive_handle = None
    last_shown_color = None
    seq_step = 0
    lights_off_at = 0.0
    table_detect_count = 0
    start_pose = None
    final_front_dist = 0.0
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[STARTUP] Initialized. Press BTN_9 to start.")

    while True:
        now = time.monotonic()
        global RUNBACK

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
            # Enable servos early
            robot.enable_servo(GRIPPER_CHANNEL)
            robot.enable_servo(CAMERA_CHANNEL)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_9):
                print(f"[ACTION] Opening gripper to {GRIPPER_OPEN_DEG}°")
                robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
                #state = "CAMERA_SCAN"
                # delete below later
                robot.step_enable(ARM_STEPPER)
                robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=False)
                    
                drive_handle = None 
                table_detect_count = 0
                state = "DRIVE_SEARCH_TABLE"
                state = "DRIVE_SEARCH_TABLE"  # Skip camera scan for now, go straight to driving and obstacle detection test

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
                    if traffic_light_color != last_shown_color:
                        print("[VISION] Traffic Light: RED. Waiting...")
                
                elif traffic_light_color == "green":
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    print("[VISION] Traffic Light: GREEN. Resetting camera, lifting arm, and driving forward.")
                    
                    # Reset camera
                    robot.set_servo(CAMERA_CHANNEL, CAMERA_DEFAULT_DEG)
                    
                    # Start lifting arm (non-blocking)
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

        elif state == "TURN_LEFT_90": #left is posotive
            if drive_handle is None:
                drive_handle = robot.turn_by(90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
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
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
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
            # Timeout increased to 20s as a safety measure
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=20.0):
                print("[FSM] Arm lowered. Closing gripper.")
                state = "CLOSE_GRIPPER"
            else:
                print("[ERROR] Stepper error during arm lowering.")
                state = "IDLE"

        elif state == "CLOSE_GRIPPER":
            # Note: Now using UI-aligned degrees (5.0) which maps to ~555us.
            print(f"[ACTION] Closing gripper to {GRIPPER_CLOSE_DEG_MEAT}° - Blocking...")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG_MEAT)
            time.sleep(2.0)  
            state = "RAISE_ARM_FINAL"

        elif state == "RAISE_ARM_FINAL":
            print("[ACTION] Raising arm with object...")
            robot.step_enable(ARM_STEPPER)
            # Timeout increased to 20s as a safety measure
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=True, timeout=20.0):
                print("[FSM] Arm raised. Starting first stage of reverse.")
                state = "DRIVE_BACK_100"
            else:
                print("[ERROR] Stepper error during arm raising.")
                state = "IDLE"

        elif state == "DRIVE_BACK_100":
            if drive_handle is None:
                print("[ACTION] Driving back 100mm (Stage 1/2)")
                drive_handle = robot.move_backward(100.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90"

        elif state == "TURN_RIGHT_90": #right is neg
            if drive_handle is None:
                print("[ACTION] Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_155"
        elif state == "DRIVE_FWD_155":
            if drive_handle is None:
                print("[ACTION] Driving forward 150mm")
                drive_handle = robot.move_forward(FIRST_RIGHT_FORWARD+5, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_STAGE2"

        elif state == "TURN_LEFT_90_STAGE2": #left is pos
            if drive_handle is None:
                print("[ACTION] Turning Left 90°")
                drive_handle = robot.turn_by(90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_STAGE2"

        elif state == "ALIGN_FRONT_STAGE2":
            print("[ACTION] Aligning with front wall (Stage 2)")
            if robot_align_front(robot):
                print("[FSM] Front alignment successful.")
            else:
                print("[WARN] Front alignment failed.")
            
            # Get front distance for final approach
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                print(f"[REPORT] Final Front Distance: {final_front_dist:.1f} mm.")
                state = "FINAL_APPROACH_STAGE2"
            else:
                print("[WARN] Front distance unknown. Ending sequence.")
                state = "OPEN_GRIPPER_POST"
            
        elif state == "FINAL_APPROACH_STAGE2":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                if target_fwd > 0:
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
                else:
                    state = "OPEN_GRIPPER_POST"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Phase 2 complete. Opening gripper.")
                state = "OPEN_GRIPPER_POST"

        elif state == "OPEN_GRIPPER_POST":
            print(f"[ACTION] Opening gripper to {GRIPPER_OPEN_DEG}°")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
            time.sleep(1.0)
            state = "DRIVE_BACK_120_POST"

        elif state == "DRIVE_BACK_120_POST":
            if drive_handle is None:
                print("[ACTION] Driving back 120mm")
                drive_handle = robot.move_backward(120.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_POST"

        elif state == "TURN_LEFT_90_POST": #left is positive in our setup, but using positive degrees for UI consistency
            if drive_handle is None:
                print("[ACTION] Turning Left 90°")
                drive_handle = robot.turn_by(90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_POST"

        elif state == "DRIVE_FWD_150_POST":
            if drive_handle is None:
                print("[ACTION] Driving forward 150mm")
                drive_handle = robot.move_forward(100.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_2_POST"

        elif state == "DRIVE_FWD_150_2_POST":
            if drive_handle is None:
                print("[ACTION] Driving forward another 150mm")
                drive_handle = robot.move_forward(FIRST_LEFT_FORWARD, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_POST"

        elif state == "TURN_RIGHT_90_POST":
            if drive_handle is None:
                print("[ACTION] Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_POST"

        elif state == "ALIGN_FRONT_POST":
            print("[ACTION] Aligning with front wall")
            if robot_align_front(robot):
                print("[FSM] Front alignment successful.")
            else:
                print("[WARN] Front alignment failed.")
            
            # Get front distance for final approach
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                print(f"[REPORT] Final Front Distance: {final_front_dist:.1f} mm.")
                state = "FINAL_APPROACH_POST"
            else:
                print("[WARN] Front distance unknown. Ending sequence.")
                state = "LOWER_ARM_FINAL"

        elif state == "FINAL_APPROACH_POST":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                print(f"[ACTION] Driving forward {target_fwd:.1f}mm")
                if target_fwd > 0:
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
                else:
                    state = "LOWER_ARM_FINAL"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Post-delivery approach complete. Lowering arm.")
                state = "LOWER_ARM_FINAL"

        elif state == "LOWER_ARM_FINAL":
            print("[ACTION] Lowering arm (Final)...")
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS, blocking=True, timeout=20.0):
                print("[FSM] Arm lowered. Closing gripper to 15.0°.")
                state = "CLOSE_GRIPPER_FINAL"
            else:
                print("[ERROR] Stepper error during arm lowering.")
                state = "IDLE"

        elif state == "CLOSE_GRIPPER_FINAL":
            print("[ACTION] Closing gripper to 15.0°")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
            time.sleep(2.0)
            state = "RAISE_ARM_DOUBLE_FINAL"

        elif state == "RAISE_ARM_DOUBLE_FINAL":
            print("[ACTION] Raising arm (Double distance)...")
            robot.step_enable(ARM_STEPPER)
            # Raise arm twice the normal amount
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS * 2, blocking=True, timeout=30.0):
                print("[FSM] Arm raised double distance. Driving back 100mm.")
                state = "DRIVE_BACK_FINAL"
            else:
                print("[ERROR] Stepper error during double arm raise.")
                state = "IDLE"

        elif state == "DRIVE_BACK_FINAL":
            if drive_handle is None:
                print("[ACTION] Final Reverse: 100mm")
                drive_handle = robot.move_backward(100.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_END"

        elif state == "TURN_RIGHT_90_END":
            if drive_handle is None:
                print("[ACTION] Turning Right 90° (End Phase)")
                drive_handle = robot.turn_by(-90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_END"

        elif state == "DRIVE_FWD_150_END":
            if drive_handle is None:
                print("[ACTION] Driving forward 150mm (End Phase)")
                drive_handle = robot.move_forward(125.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DRIVE_FWD_150_2_END"

        elif state == "DRIVE_FWD_150_2_END":
            if drive_handle is None:
                print("[ACTION] Driving forward another 150mm (End Phase)")
                drive_handle = robot.move_forward(SECOND_RIGHT_FORWARD, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_LEFT_90_END"

        elif state == "TURN_LEFT_90_END":
            if drive_handle is None:
                print("[ACTION] Turning Left 90° (End Phase)")
                drive_handle = robot.turn_by(90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "ALIGN_FRONT_END"

        elif state == "ALIGN_FRONT_END":
            print("[ACTION] Aligning with front wall (End Phase)")
            if robot_align_front(robot):
                print("[FSM] Front alignment successful.")
            else:
                print("[WARN] Front alignment failed.")
            
            # Get front distance for final approach
            points = np.asarray(robot.get_obstacles())
            f_dist, f_count = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                print(f"[REPORT] Final Front Distance: {final_front_dist:.1f} mm.")
                state = "FINAL_APPROACH_END"
            else:
                print("[WARN] Front distance unknown. Ending sequence.")
                state = "OPEN_GRIPPER_END"

        elif state == "FINAL_APPROACH_END":
            if drive_handle is None:
                target_fwd = final_front_dist - 40.0
                print(f"[ACTION] Driving forward {target_fwd:.1f}mm (End Phase)")
                if target_fwd > 0:
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
                else:
                    state = "LOWER_ARM_QUARTER_END"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "LOWER_ARM_QUARTER_END"

        elif state == "LOWER_ARM_QUARTER_END":
            print("[ACTION] Lowering arm (1/4 distance)...")
            robot.step_enable(ARM_STEPPER)
            # Lower by 1/4 of the usual distance
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS // 3, blocking=True, timeout=10.0):
                print("[FSM] Arm lowered 1/4. Opening gripper.")
                state = "OPEN_GRIPPER_END"
            else:
                print("[ERROR] Stepper error during quarter arm lowering.")
                state = "IDLE"

        elif state == "OPEN_GRIPPER_END":
            print(f"[ACTION] Opening gripper to {GRIPPER_OPEN_DEG}° (End Phase)")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
            time.sleep(1.0)
            state = "LOWER_ARM_DOUBLE_END"

        elif state == "LOWER_ARM_DOUBLE_END":
            print("[ACTION] Lowering arm (Double distance)...")
            robot.step_enable(ARM_STEPPER)
            # Lower arm by double the amount raised (-ARM_UP_STEPS * 2)
            if robot.step_move(ARM_STEPPER, steps=-ARM_UP_STEPS * 2, blocking=True, timeout=40.0):
                print("[FSM] Arm lowered double distance. Closing gripper.")
                state = "CLOSE_GRIPPER_END"
            else:
                print("[ERROR] Stepper error during double arm lowering.")
                state = "IDLE"

        elif state == "CLOSE_GRIPPER_END":
            print(f"[ACTION] Closing gripper to {GRIPPER_CLOSE_DEG}°")
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
            time.sleep(2.0)
            state = "RAISE_ARM_DOUBLE_FINAL"
            state = "RAISE_ARM_FINAL_END"

        elif state == "RAISE_ARM_FINAL_END":
            print("[ACTION] Raising arm (Final Stage)...")
            robot.step_enable(ARM_STEPPER)
            if robot.step_move(ARM_STEPPER, steps=ARM_UP_STEPS, blocking=True, timeout=20.0):
                print("[FSM] Arm raised. Final reverse.")
                state = "DRIVE_BACK_FINAL_END"
            else:
                print("[ERROR] Stepper error during final arm raise.")
                state = "IDLE"

        elif state == "DRIVE_BACK_FINAL_END":
            if drive_handle is None:
                print("[ACTION] Final Reverse: 100mm")
                drive_handle = robot.move_backward(70.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "TURN_RIGHT_90_FINAL"

        elif state == "TURN_RIGHT_90_FINAL":
            if drive_handle is None:
                print("[ACTION] Turning Right 90° (Final)")
                drive_handle = robot.turn_by(-87.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Sequence complete. Bgining MAZE phase.")
                state = "MAZETIME"

        elif state == "MAZETIME":
            configure_robot_for_lapf(robot)
            seq_step = 0
            print("[GPS] Checking for GPS lock (Tag 21)...")
            gps_wait = time.monotonic()
            gps_locked = False
            last_gps_print = 0.0
            while time.monotonic() - gps_wait < 30.0:
                if robot.is_gps_active():
                    gps_x, gps_y = robot._gps_x_mm, robot._gps_y_mm
                    if abs(gps_x) > 0.1 or abs(gps_y) > 0.1:
                        print(f"[GPS] Lock acquired! x={gps_x:.1f}mm y={gps_y:.1f}mm")
                        gps_locked = True
                        break
                if time.monotonic() - last_gps_print >= 2.0:
                    print(f"[GPS] Waiting... ({time.monotonic() - gps_wait:.0f}s elapsed)")
                    last_gps_print = time.monotonic()
                time.sleep(0.1)

            if not gps_locked:
                print("[WARN] GPS lock failed after 30s. Proceeding with Odometry only.")
            else:
                print("[GPS] Synchronizing Odometry to GPS (Alpha Snap)...")
                robot.set_position_fusion_alpha(1.0)
                time.sleep(0.5)
                robot.set_position_fusion_alpha(0.08)
                print("[GPS] Synchronization complete. Pose is now aligned.")
                time.sleep(0.2)

            drive_handle = robot.purepursuit_follow_path(
                waypoints=PATH_CONTROL_POINTS,
                velocity=VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=TOLERANCE_MM,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S,
                blocking=False,
            )
            print(f"[FSM] MOVING — Using built-in Pure Pursuit")
            last_status_print_at = now    
            state = "EXEC_MAZE_DRIVING"

        elif state == "EXEC_MAZE_DRIVING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] DONE — path complete")
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] Aligning with wall...")
                robot_align_to_wall(robot, 90.0, 0.0)
                
                seq_step = 0
                state = "EXEC_TURN_SEQUENCE"

        elif state == "EXEC_TURN_SEQUENCE":
            if seq_step == 0:
                print("[SEQ] Step 1/3: Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=True)
                robot_align_to_wall(robot, 180.0, 90.0)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 670 mm")
                    drive_handle = robot.move_forward(670.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 2
            elif seq_step == 2:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 3/3: Turning Right 90°")
                    drive_handle = robot.turn_by(-90.0, blocking=False)
                    seq_step = 3
            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    robot.stop()
                    show_idle_leds(robot)
                    print("[SEQ] Sequence complete! Transitioning to EXEC_RAMP_SEQUENCE.")
                    RUNBACK = True
                    seq_step = 0
                    state = "EXEC_RAMP_SEQUENCE"

        elif state == "EXEC_RAMP_SEQUENCE":
            if seq_step == 0:
                print("[RAMP] Step 1/4: Moving 450 mm forward to get closer to ramp")
                drive_handle = robot.move_forward(400.0, velocity=VELOCITY_MM_S, tolerance=10.0, blocking=False)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[RAMP] Step 2/4: Aligning with ramp (front)...")
                    robot_align_to_wall(robot, 180.0, 90.0)
                    seq_step = 2
            elif seq_step == 2:
                print("[RAMP] Step 3/4: Driving straight 2715 mm (Manual)")
                drive_handle = robot.move_forward(2800.0, velocity=VELOCITY_MM_S, tolerance=50.0, blocking=False)
                seq_step = 3
            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[RAMP] Step 4/4: Aligning with wall before turn (front)...")
                    robot_align_to_wall(robot, 90.0, 0.0)
                    robot.stop()
                    show_idle_leds(robot)
                    seq_step = 0
                    state = "EXEC_TURN_SEQUENCE_2"

        elif state == "EXEC_TURN_SEQUENCE_2":
            if seq_step == 0:
                print("[SEQ] Step 1/3: Turning Left 90°")
                drive_handle = robot.turn_by(90.0, blocking=False)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 700mm")
                    drive_handle = robot.move_forward(800.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 2
            elif seq_step == 2:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 3/3: Turning Left 90°")
                    drive_handle = robot.turn_by(80.0, blocking=False)
                    seq_step = 3
            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 200mm")
                    drive_handle = robot.move_forward(10.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 4
            elif seq_step == 4:
                if drive_handle is not None and drive_handle.is_finished():
                    robot.stop()
                    show_idle_leds(robot)
                    print("[SEQ] Maze Sequence complete! Transitioning to OBSTACLE_AVOIDANCE_START.")
                    RUNBACK = True
                    state = "OBSTACLE_AVOIDANCE_START"

        elif state == "OBSTACLE_AVOIDANCE_START":
            # Start of Obstacle Avoidance logic integrated from obstacleavoidance7final.py
            reset_mission_pose(robot)

            show_running_leds(robot)
            drive_handle = start_lapf_goal(robot)
            last_status_print_at = now
            print("[FSM] OBSTACLE AVOIDANCE — LAPF goal seeking started")
            state = "OBSTACLE_AVOIDANCE_MOVING"

        elif state == "OBSTACLE_AVOIDANCE_MOVING":
            # HARD THRESHOLD CHECK: Stop immediately if we pass the Y-goal
            curr_pose = robot.get_pose()
            if curr_pose[1] >= LAPF_GOAL_MM[1]:
                print(f"[MASTER] Hard threshold {LAPF_GOAL_MM[1]}mm reached (Current Y: {curr_pose[1]:.0f}mm).")
                # Important: robot.stop() stops the physical motors, 
                # but we must also null the drive_handle to stop tracking the old motion.
                robot.stop()
                drive_handle = None
                state = "POST_NAV_ALIGN"

            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_lapf_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] OBSTACLE AVOIDANCE — Goal reached!")
                print_lapf_status(robot)
                drive_handle = None
                robot.stop()
                state = "POST_NAV_ALIGN"

        elif state == "POST_NAV_ALIGN":
            # Ensure any background motion is dead before starting blocking alignment
            if drive_handle is not None:
                robot.stop()
                drive_handle = None

            print("[MASTER] Aligning with front wall...")
            if robot_align_front(robot):
                print("[MASTER] Aligned. Approaching to 75mm via LiDAR.")
                state = "POST_NAV_APPROACH_LIDAR"
            else:
                state = "IDLE"

        elif state == "POST_NAV_APPROACH_LIDAR":
            # Precision approach to exactly 75mm from wall
            if robot_approach_wall(robot, 75.0, tolerance_mm=2.0):
                print("[MASTER] Positioned 75mm from wall. Starting scan.")
                state = "POST_NAV_SCAN"
            else:
                print("[WARN] Approach failed. Retrying alignment.")
                state = "POST_NAV_ALIGN"

        elif state == "POST_NAV_SCAN":
            print("[MASTER] Initiating face scan (15s timeout)...")
            gender = classifier.get_gender(wait_for_face=15.0)
            if gender:
                print(f"[RESULT] CUSTOMER IDENTIFIED: {gender}")
                print("[MASTER] Face recognized. Starting final sequence.")
                state = "FINAL_MOVE_FWD_75"
                drive_handle = None
            else:
                print("[RESULT] No face detected during scan.")
                print("[MASTER] MISSION COMPLETE. Returning to IDLE.")
                show_idle_leds(robot)
                state = "IDLE"

        elif state == "FINAL_MOVE_FWD_75":
            if drive_handle is None:
                print("[ACTION] Driving forward 75mm")
                drive_handle = robot.move_forward(75.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "FINAL_TURN_RIGHT_90"

        elif state == "FINAL_TURN_RIGHT_90":
            if drive_handle is None:
                print("[ACTION] Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "FINAL_ALIGN_LEFT_WALL"

        elif state == "FINAL_ALIGN_LEFT_WALL":
            print("[ACTION] Aligning with left wall")
            # Use robot_align_to_wall with 90° (Left) and 0° (parallel target)
            if robot_align_to_wall(robot, 90.0, 0.0):
                print("[FSM] Left alignment successful.")
            else:
                print("[WARN] Left alignment failed. Proceeding anyway.")
            
            start_pose = robot.get_odometry_pose()
            state = "FINAL_WALL_FOLLOW_2800"

        elif state == "FINAL_WALL_FOLLOW_2800":
            curr_pose = robot.get_odometry_pose()
            dist_traveled = math.hypot(curr_pose[0] - start_pose[0], curr_pose[1] - start_pose[1])
            
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            
            if dist is not None and count > 0:
                error_mm = dist - 50.0 # Maintain 50mm from left wall
                angular_cmd = error_mm * FOLLOW_KP
                angular_cmd = max(-20.0, min(20.0, angular_cmd))
                robot.set_velocity(VELOCITY_MM_S, angular_cmd)
                
                if time.monotonic() % 1.0 < 0.1:
                    print(f"[FOLLOW] Dist:{dist:.1f}mm | Traveled:{dist_traveled:.0f}mm")
            else:
                # If wall lost, drive straight
                robot.set_velocity(VELOCITY_MM_S, 0.0)

            if dist_traveled >= 2800.0:
                robot.stop()
                drive_handle = None
                print(f"[FSM] Final wall-following complete ({dist_traveled:.0f}mm). MISSION COMPLETE.")
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

with open('/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py', 'w') as f:
    f.write(content)
"""

with open('restore_main.py', 'w') as f:
    f.write(content)
