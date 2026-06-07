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
# Obstacle Avoidance Configuration
# ---------------------------------------------------------------------------
LAPF_GOAL_MM = (100, 2525)
LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 1.0

# LAPF behavior tuning
LEASH_LENGTH_MM = 300.0 
REPULSION_RANGE_MM = 400.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

# ---------------------------------------------------------------------------
# Pure Pursuit & GPS Configuration
# ---------------------------------------------------------------------------
LOOKAHEAD_MM       = 250.0  
TOLERANCE_MM       = 80.0   
ADVANCE_RADIUS_MM  = 250.0  
MAX_ANGULAR_RAD_S  = 1.0    

# Post-obstacle GPS waypoints to get close to the front wall consistently
POST_OBSTACLE_GPS_WAYPOINTS = [
    
   
   
    (1750, 3500),
    (1750, 3600),
    (1750, 3700),
    (1750, 3800),
    (1750, 3850),

]

# ---------------------------------------------------------------------------
# Vision & Drive Constants
# ---------------------------------------------------------------------------
VELOCITY_MM_S = 100.0
TURN_VELOCITY_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5
FOLLOW_KP = 0.3
STATUS_PRINT_INTERVAL_S = 0.5

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

def configure_robot_for_lapf(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    
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
    if robot.has_fused_pose():
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

    print(f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°{vt_summary}{track_summary}")

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
    robot.disable_gps()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s")
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
    global LAPF_GOAL_MM
    print("[STARTUP] Standalone GPS-Enabled Obstacle Scan Delivery Initialized.")
    configure_robot_for_lapf(robot)
    classifier = CustomerClassifier(robot)
    
    state = "INIT"
    final_front_dist = 0.0
    stop_sign_timer = 0.0
    drive_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("Press BTN_9 to start the obstacle avoidance sequence.")

    while True:
        now = time.monotonic()

        # BTN_2 Kill Switch
        if state != "IDLE" and robot.was_button_pressed(Button.BTN_2):
            robot.cancel_motion()
            drive_handle = None
            state = "IDLE"
            print("[FSM] Stopped. Returning to IDLE.")

        if state == "INIT":
            start_robot(robot)
            # Configure actuators
            robot.step_set_config(Stepper.STEPPER_1, max_velocity=400, acceleration=200)
            robot.enable_servo(ServoChannel.CH_1)
            robot.enable_servo(ServoChannel.CH_2)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_9):
                print("[FSM] Starting Obstacle Avoidance...")
                reset_mission_pose(robot)
                show_running_leds(robot)
                drive_handle = start_lapf_goal(robot)
                last_status_print_at = now
                state = "OBSTACLE_AVOIDANCE_MOVING"

        elif state == "OBSTACLE_AVOIDANCE_MOVING":
            curr_pose = robot.get_pose()
            if curr_pose[1] >= LAPF_GOAL_MM[1]:
                print(f"[MASTER] Goal Y {LAPF_GOAL_MM[1]}mm reached.")
                robot.cancel_motion()
                drive_handle = None
                state = "POST_NAV_ALIGN"

            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_lapf_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] Goal reached!")
                drive_handle = None
                robot.cancel_motion()
                state = "POST_NAV_ALIGN"

        # elif state == "GPS_WAYPOINT_START":
        #     configure_robot_for_lapf(robot)
        #     print("[GPS] Checking for GPS lock (Tag 21) for post-obstacle navigation...")
        #     gps_wait = time.monotonic()
        #     gps_locked = False
        #     last_gps_print = 0.0
        #     while time.monotonic() - gps_wait < 30.0:
        #         if robot.is_gps_active():
        #             gps_x, gps_y = robot._gps_x_mm, robot._gps_y_mm
        #             if abs(gps_x) > 0.1 or abs(gps_y) > 0.1:
        #                 print(f"[GPS] Lock acquired! x={gps_x:.1f}mm y={gps_y:.1f}mm")
        #                 gps_locked = True
        #                 break
        #         if time.monotonic() - last_gps_print >= 2.0:
        #             print(f"[GPS] Waiting... ({time.monotonic() - gps_wait:.0f}s elapsed)")
        #             last_gps_print = time.monotonic()
        #         time.sleep(0.1)

        #     if not gps_locked:
        #         print("[WARN] GPS lock failed after 30s. Proceeding with Odometry only.")
        #     else:
        #         print("[GPS] Synchronizing Odometry to GPS (Alpha Snap)...")
        #         robot.set_position_fusion_alpha(1.0)
        #         time.sleep(0.5)
        #         robot.set_position_fusion_alpha(0.08)
        #         print("[GPS] Synchronization complete. Pose is now aligned.")
        #         time.sleep(0.2)

        #     drive_handle = robot.purepursuit_follow_path(
        #         waypoints=POST_OBSTACLE_GPS_WAYPOINTS,
        #         velocity=VELOCITY_MM_S,
        #         lookahead=LOOKAHEAD_MM,
        #         tolerance=TOLERANCE_MM,
        #         advance_radius=ADVANCE_RADIUS_MM,
        #         max_angular_rad_s=MAX_ANGULAR_RAD_S,
        #         blocking=False,
        #     )
        #     print(f"[FSM] MOVING to GPS waypoints")
        #     last_status_print_at = now
        #     state = "GPS_WAYPOINT_NAV"

        # elif state == "GPS_WAYPOINT_NAV":
        #     if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
        #         print_status(robot)
        #         last_status_print_at = now
            
        #     if drive_handle is not None and drive_handle.is_finished():
        #         print("[FSM] GPS waypoints reached!")
        #         robot.stop()
        #         drive_handle = None
        #         state = "POST_NAV_ALIGN"

        elif state == "POST_NAV_ALIGN":
            if drive_handle is not None:
                robot.cancel_motion()
                drive_handle = None

            print("[MASTER] Aligning with front wall...")
            if robot_align_front(robot):
                print("[MASTER] Aligned. Measuring distance.")
                points = np.asarray(robot.get_obstacles())
                f_dist, _ = get_front_distance(points)
                if f_dist is not None:
                    final_front_dist = f_dist
                    state = "POST_NAV_APPROACH"
                else:
                    state = "POST_NAV_APPROACH"
            else:
                state = "POST_NAV_APPROACH"

        elif state == "POST_NAV_APPROACH":
            if drive_handle is None:
                target_fwd = final_front_dist - 100.0
                print(f"[MASTER] Precision Approach: {target_fwd:.1f}mm")
                drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.cancel_motion()
                drive_handle = None
                state = "POST_NAV_TURN_RIGHT_90"

        elif state == "POST_NAV_TURN_RIGHT_90":
            if drive_handle is None:
                print("[MASTER] Turning right 90° for scan.")
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.cancel_motion()
                drive_handle = None
                state = "SCAN_LEFT_POST_NAV"
        elif state == "SCAN_LEFT_POST_NAV":
            if drive_handle is not None:
                robot.cancel_motion()
                drive_handle = None

            print("[MASTER] Aligning left for scan.")
            robot_align_left(robot)
            state = "POST_NAV_DRIVE"

        elif state == "POST_NAV_DRIVE":
            if drive_handle is None:
                print("[MASTER] Driving forward 350mm to scan point.")
                drive_handle = robot.move_forward(350.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.cancel_motion()
                drive_handle = None
                print("[MASTER] Aligned. Measuring distance.")
                points = np.asarray(robot.get_obstacles())
                f_dist, _ = get_front_distance(points)
                if f_dist is not None:
                    final_front_dist = f_dist
                    state = "POST_NAV_DRIVE_PRE_FACE_SCAN"
                else:
                    state = "POST_NAV_DRIVE_PRE_FACE_SCAN"

        elif state == "POST_NAV_DRIVE_PRE_FACE_SCAN":
            if drive_handle is None:
                target_fwd = final_front_dist - 450.0
                print(f"[MASTER] Drive Approach Pre Face: {target_fwd:.1f}mm")
                drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.cancel_motion()
                drive_handle = None
                state = "POST_NAV_SCAN"


        elif state == "POST_NAV_SCAN":
            print("[MASTER] Initiating robust face scan (Majority Vote)...")
            results = []
            max_attempts = 10  # Try up to 10 times to get 5 detections
            attempts = 0
            
            # Stabilization wait
            time.sleep(1.0)
            
            while len(results) < 5 and attempts < max_attempts:
                attempts += 1
                # Short wait for face detection per attempt
                res = classifier.get_gender(wait_for_face=3.0)
                if res:
                    results.append(res)
                    print(f"  [SCAN] Detection {len(results)}/5: {res}")
                else:
                    print("  [SCAN] No face seen in this attempt.")
            
            if results:
                # Majority vote
                gender = max(set(results), key=results.count)
                print(f"[RESULT] CUSTOMER IDENTIFIED (Voted): {gender} from {results}")
                
                # Define and use variables within the FSM context
                detected_gender = gender
                follow_distance_mm = 1550
                dropoff_distance_mm = 1100.0 if "boy" in gender.lower() else 600.0
                state = "POST_SCAN_GET_FRONT_DIST"
                drive_handle = None
            else:
                print("[RESULT] No face detected after multiple attempts.")
                print("[MASTER] MISSION COMPLETE. Returning to IDLE.")
                show_idle_leds(robot)
                state = "IDLE"

        elif state == "POST_SCAN_GET_FRONT_DIST":
            points = np.asarray(robot.get_obstacles())
            f_dist, _ = get_front_distance(points)
            if f_dist is not None:
                final_front_dist = f_dist
                print(f"[MASTER] Front distance: {final_front_dist:.1f}mm. Driving forward {final_front_dist - 100.0:.1f}mm.")
                state = "POST_SCAN_DRIVE_FWD"
            else:
                print("[WARN] Could not get front distance. Retrying...")

        elif state == "POST_SCAN_DRIVE_FWD":
            if drive_handle is None:
                target_fwd = final_front_dist - 150.0
                if target_fwd > 0:
                    drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
                else:
                    state = "POST_SCAN_TURN_RIGHT_90"
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "POST_SCAN_TURN_RIGHT_90"

        elif state == "POST_SCAN_TURN_RIGHT_90":
            if drive_handle is None:
                print("[MASTER] Turning right 90° for wall follow.")
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                start_pose = robot.get_odometry_pose()
                print(f"[MASTER] Starting wall follow for {follow_distance_mm}mm.")
                state = "POST_SCAN_WALL_FOLLOW"

        elif state == "POST_SCAN_WALL_FOLLOW":
            curr_pose = robot.get_odometry_pose()
            dist_traveled = math.hypot(curr_pose[0] - start_pose[0], curr_pose[1] - start_pose[1])
            
            points = np.asarray(robot.get_obstacles())
            # Maintain 300mm from the left wall
            dist, count = get_left_distance(points)
            
            if dist is not None and count > 0:
                WALL_FOLLOW_TARGET_MM = 300.0
                error_mm = dist - WALL_FOLLOW_TARGET_MM
                angular_cmd = error_mm * FOLLOW_KP
                # Clamp angular speed
                angular_cmd = max(-20.0, min(20.0, angular_cmd))
                robot.set_velocity(VELOCITY_MM_S, angular_cmd)
                
                if np.random.rand() < 0.05:
                    print(f"[WALL_FOLLOW] Dist:{dist:.1f}mm | Err:{error_mm:+.1f}mm | Traveled:{dist_traveled:.0f}/{follow_distance_mm}mm")
            else:
                robot.set_velocity(VELOCITY_MM_S, 0.0)

            if dist_traveled >= follow_distance_mm:
                robot.stop()
                print(f"[MASTER] Wall follow complete. Transitioning to delivery sequence.")
                state = "DROPOFF_MOVE_FORWARD"

        elif state == "DROPOFF_MOVE_FORWARD":
            if drive_handle is None:
                print(f"[ACTION] Driving forward {dropoff_distance_mm:.1f}mm for dropoff.")
                drive_handle = robot.move_forward(dropoff_distance_mm, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DELIVERY_TURN_LEFT"

        elif state == "DELIVERY_TURN_LEFT":
            if drive_handle is None:
                print("[ACTION] Turning Left 90° for delivery.")
                drive_handle = robot.turn_by(90.0, blocking=False)
                
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                #get front distance for final approach
                points = np.asarray(robot.get_obstacles())
                f_dist, _ = get_front_distance(points)
                if f_dist is not None:
                    final_front_dist = f_dist
                    state = "DELIVERY_FORWARD_APPROACH"
                else:
                    state = "DELIVERY_FORWARD_APPROACH"
        
        elif state == "DELIVERY_FORWARD_APPROACH":
            if drive_handle is None:
                target_fwd = 10.0
                print(f"[ACTION] Driving forward {target_fwd:.1f}mm for delivery approach.")
                drive_handle = robot.move_forward(target_fwd, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
                
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DELIVERY_OPEN_GRIPPER"

        elif state == "DELIVERY_OPEN_GRIPPER":
            # Using GRIPPER_OPEN_DEG = 120.0 from startup sequence
            print(f"[ACTION] Opening gripper to 120.0°")
            robot.set_servo(ServoChannel.CH_1, 120.0)
            time.sleep(1.0)
            state = "DELIVERY_RAISE_ARM"
        
        elif state == "DELIVERY_RAISE_ARM":
            print("[ACTION] Raising arm (2100 steps)...")
            robot.step_enable(Stepper.STEPPER_1)
            # Raise arm by 700 * 3 = 2100 steps. Negative is up in this setup.
            if robot.step_move(Stepper.STEPPER_1, steps=-2100, blocking=True, timeout=20.0):
                state = "DELIVERY_DRIVE_BACK"
            else:
                print("[ERROR] Stepper error during arm raise.")
                state = "IDLE"
        
        elif state == "DELIVERY_DRIVE_BACK":
            if drive_handle is None:
                print("[ACTION] Driving back 100mm")
                drive_handle = robot.move_backward(100.0, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "DELIVERY_TURN_RIGHT_90"

        elif state == "DELIVERY_TURN_RIGHT_90":
            if drive_handle is None:
                print("[ACTION] Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "LOWER_ARM_POST_DELIVERY"
        
        elif state == "LOWER_ARM_POST_DELIVERY":
            print("[ACTION] Lowering arm (2800 steps)...")
            robot.step_enable(Stepper.STEPPER_1)
            if robot.step_move(Stepper.STEPPER_1, steps=2800, blocking=True, timeout=20.0):
                state = "WATCH_FOR_STOP_SIGN"
            else:
                print("[ERROR] Stepper error during arm lower.")
                state = "IDLE"

        elif state == "WATCH_FOR_STOP_SIGN":
            # Robot starts moving forward immediately
            if drive_handle is None:
                print("[ACTION] Starting forward movement. Waiting 3s before turning camera.")
                robot.set_velocity(VELOCITY_MM_S, 0.0)
                stop_sign_timer = now
                drive_handle = "STARTED"
                state = "WAIT_TO_SCAN"

        elif state == "WAIT_TO_SCAN":
            # Wait 3 seconds while moving before turning camera
            if now - stop_sign_timer >= 3.0:
                print("[ACTION] 3 seconds elapsed. Turning camera to 25° while moving.")
                robot.set_servo(ServoChannel.CH_2, 25.0)
                state = "SCAN_FOR_STOP_SIGN"

        elif state == "SCAN_FOR_STOP_SIGN":
            # Watch for stop sign detection
            if robot.has_detection("stop sign", min_confidence=0.4):
                print("[VISION] STOP SIGN DETECTED! Stopping for 2s.")
                robot.stop()
                stop_sign_timer = now
                state = "STOPPED_AT_SIGN"

        elif state == "STOPPED_AT_SIGN":
            # Stop for 2 seconds
            if now - stop_sign_timer >= 2.0:
                print("[ACTION] 2 seconds elapsed. Resuming forward motion.")
                robot.set_velocity(VELOCITY_MM_S, 0.0)
                stop_sign_timer = now
                state = "CONTINUE_FORWARD"

        elif state == "CONTINUE_FORWARD":
            # Continue forward motion for 3 seconds
            if now - stop_sign_timer >= 3.0:
                print("[ACTION] 3s forward complete. Turning left 20°.")
                robot.stop()
                drive_handle = None
                state = "FINAL_TURN_LEFT_20"

        elif state == "FINAL_TURN_LEFT_20":
            if drive_handle is None:
                drive_handle = robot.turn_by(20.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[ACTION] Turn complete. Moving forward 300mm.")
                state = "FINAL_MOVE_FWD_300"

        elif state == "FINAL_MOVE_FWD_300":
            if drive_handle is None:
                drive_handle = robot.move_forward(300.0, velocity=VELOCITY_MM_S, tolerance=5.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Mission complete. Returning to IDLE.")
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
        robot.cancel_motion()
        rclpy.shutdown()
