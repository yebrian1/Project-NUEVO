from __future__ import annotations
import time
import math
import numpy as np
import matplotlib.pyplot as plt

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
from robot.util import densify_polyline

# Import LiDAR helpers instead of defining them locally
from robot.lidar_helpers import (
    get_front_distance,
    robot_align_front,
    robot_align_right,
    robot_align_left,
    robot_approach_front,
    save_lidar_plot,
)

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
# Actuator Configuration
# ---------------------------------------------------------------------------
GRIPPER_CHANNEL   = ServoChannel.CH_1
GRIPPER_OPEN_DEG  = 150.0   
GRIPPER_CLOSE_DEG = 50.0  

ARM_STEPPER        = Stepper.STEPPER_1
arm_up_steps       = -2000
arm_down_steps     = 1600
ARM_MAX_VELOCITY   = 400    
ARM_ACCELERATION   = 200    
ARM_HOME_VELOCITY  = 300    

# ---------------------------------------------------------------------------
# Pure Pursuit Configuration
# ---------------------------------------------------------------------------
VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 250.0  
TOLERANCE_MM       = 80.0   
ADVANCE_RADIUS_MM  = 250.0  
MAX_ANGULAR_RAD_S  = 1.0    

STATUS_PRINT_INTERVAL_S = 0.5

# Start at 0,0 and go straight forward 1000mm
PATH_CONTROL_POINTS = [
    (180, 900),           
]

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
ENABLE_GPS = True
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

# ---------------------------------------------------------------------------
# Robot Control Setup Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
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
    
    # LIDAR Health Check
    print("[INIT] Checking LIDAR health...")
    timeout = time.monotonic() + 3.0
    while time.monotonic() < timeout:
        points = robot.get_obstacles()
        if points and len(points) > 0:
            print(f"[INIT] LIDAR OK: {len(points)} points detected.")
            break
        time.sleep(0.1)
    else:
        print("[WARN] LIDAR timeout: No points received. Check connection/permissions.")

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)

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

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    
    drive_handle = None
    target_theta = None
    
    last_status_print_at = 0.0
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # Shared state variables
    last_shown_color = None
    lights_off_at = 0.0 
    alignment_attempts = 0
    target_dist = 25.0

    while True:
        now = time.monotonic()

        # Global Kill Switch (BTN_2 cancels active driving configurations)
        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
                drive_handle = None
            robot.stop() 
            show_idle_leds(robot)
            print("[FSM] IDLE — Execution canceled manually via BTN_2.")
            state = "IDLE"

        # ---------------------------------------------------------
        # FSM STATES
        # ---------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            print("[FSM] IDLE — press BTN_9 to start Execution Sequence")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.was_button_pressed(Button.BTN_9):
                drive_handle = None 
                state = "INITIAL_TURN"

        elif state == "INITIAL_TURN":
            if drive_handle is None:
                print("[ACTION] Starting 20° Left Turn to scan light")
                drive_handle = robot.turn_by(20.0, blocking=False)
            
            elif drive_handle.is_finished():
                robot.stop()
                dim_all_leds(robot)
                print("[FSM] Turn complete. Entering WATCHING state.")
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
                        print("[VISION] Traffic Light: RED. Stopping.")
                
                elif traffic_light_color == "green":
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    print("[VISION] Traffic Light: GREEN. Correcting heading orientation.")
                    drive_handle = None 
                    state = "TURN_BACK"
                
                last_shown_color = traffic_light_color

            elif lights_off_at > 0.0 and now >= lights_off_at:
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 0)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[VISION] No recent red/green light - LEDs off. Waiting.")
                last_shown_color = None

        elif state == "TURN_BACK":
            if drive_handle is None:
                print("[ACTION] Returning 20° Right to straighten back up")
                drive_handle = robot.turn_by(-20.0, blocking=False)
            
            elif drive_handle.is_finished():
                robot.stop()
                print("[FSM] Orientation straightened. Advancing 200 mm.")
                drive_handle = None
                state = "INITIAL_FORWARD"
                
        elif state == "INITIAL_FORWARD":
            if drive_handle is None:
                print("[ACTION] Hardcoded drive: Moving forward 200 mm")
                drive_handle = robot.move_forward(200.0, velocity=VELOCITY_MM_S, tolerance=TOLERANCE_MM, blocking=False)
                
            elif drive_handle.is_finished():
                robot.stop()
                print("[FSM] Initial drive complete. Transitioning to GPS tracking.")
                drive_handle = None
                state = "START_DRIVING"

        elif state == "START_DRIVING":
            show_running_leds(robot)
            
            print("[INIT] Waiting for firmware bridge...")
            bridge_wait = now
            while time.monotonic() - bridge_wait < 5.0:
                if robot.get_state() == FirmwareState.RUNNING:
                    break
                time.sleep(0.2)

            print("[GPS] Checking for GPS lock (Tag 21)...")
            gps_wait = time.monotonic()
            gps_locked = False
            last_gps_print = 0.0
            
            while time.monotonic() - gps_wait < 5.0:
                if robot.is_gps_active():
                    gps_x, gps_y = robot._gps_x_mm, robot._gps_y_mm
                    if abs(gps_x) > 0.1 or abs(gps_y) > 0.1:
                        print(f"[GPS] Lock acquired! x={gps_x:.1f}mm y={gps_y:.1f}mm")
                        gps_locked = True
                        break
                
                if time.monotonic() - last_gps_print >= 1.0:
                    print(f"[GPS] Waiting... ({time.monotonic() - gps_wait:.0f}s elapsed)")
                    last_gps_print = time.monotonic()
                time.sleep(0.1)

            if not gps_locked:
                print("[WARN] GPS lock failed. Tracking with Odometry.")
            else:
                time.sleep(0.5)

            # Generating the missing waypoints so Lookahead doesn't fail
            dense_path = densify_polyline(PATH_CONTROL_POINTS, spacing=20.0)

            drive_handle = robot.purepursuit_follow_path(
                waypoints=dense_path,
                velocity=VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=TOLERANCE_MM,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S,
                blocking=False,
            )

            print("[FSM] MOVING — Tracking dense path via Pure Pursuit")
            last_status_print_at = time.monotonic()    
            state = "EXEC_MAZE_DRIVING"

        elif state == "EXEC_MAZE_DRIVING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                print("[FSM] Targeted path coordinates complete. Pivoting to terminal orientation.")
                target_theta = None
                state = "FINAL_TURN"

        elif state == "FINAL_TURN":
            if drive_handle is None:
                print("[ACTION] Initiating 90° Left Turn.")
                # Reset alignment attempts before entering the alignment phase
                alignment_attempts = 0
                drive_handle = robot.turn_by(90.0, blocking=False)

            elif drive_handle.is_finished():
                robot.stop()
                print("[FSM] Turn complete. Running front LiDAR regression calibration.")
                drive_handle = None
                state = "ALIGN_WALL"

        elif state == "ALIGN_WALL":
            alignment_attempts += 1
            print(f"[FSM] Starting front wall alignment (Attempt {alignment_attempts}/3)...")
            
            if robot_align_front(robot):
                # Calculate target distance: current distance minus 10mm
                points = np.asarray(robot.get_obstacles())
                current_dist, _ = get_front_distance(points)
                
                if current_dist is not None:
                    target_dist = current_dist - 10.0
                    # Safety floor of 10mm
                    target_dist = max(10.0, target_dist) 
                    print(f"[FSM] Alignment successful. Current dist: {current_dist:.1f}mm. Target approach: {target_dist:.1f}mm.")
                    
                    # Save diagnostic plot on success
                    save_lidar_plot(robot, filename="lidar_alignment_success.png")
                    
                    state = "APPROACH_WALL"
                else:
                    print("[WARN] Could not determine distance for approach. Returning to IDLE.")
                    state = "IDLE"
            elif alignment_attempts < 3:
                print(f"[WARN] Alignment attempt {alignment_attempts} failed. Retrying...")
                # Brief pause before retry
                time.sleep(0.5)
            else:
                print("[WARN] All alignment attempts failed. Saving diagnostic plot and returning to IDLE.")
                save_lidar_plot(robot, filename="lidar_alignment_failed.png")
                robot.stop()
                show_idle_leds(robot)
                state = "IDLE"

        elif state == "APPROACH_WALL":
            # target_dist is captured from the ALIGN_WALL successful transition
            if robot_approach_front(robot, target_dist_mm=target_dist):
                print(f"[FSM] SUCCESS! Relative approach complete. Systems down.")
                show_idle_leds(robot)
                state = "IDLE"
            else:
                print("[WARN] Wall approach failed or aborted. Returning to IDLE.")
                robot.stop()
                show_idle_leds(robot)
                state = "IDLE"

        # Sleep to maintain loop frequency
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
