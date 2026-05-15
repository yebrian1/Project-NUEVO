from __future__ import annotations
import time
import math

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
from robot.path_planner import PurePursuitPlanner
from robot.util import densify_polyline

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 11 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Actuator Configuration (Arm & Gripper)
# ---------------------------------------------------------------------------
GRIPPER_CHANNEL   = ServoChannel.CH_1
GRIPPER_OPEN_DEG  = 90.0   
GRIPPER_CLOSE_DEG = 180.0  

ARM_STEPPER        = Stepper.STEPPER_1
arm_up_steps       = -2000
arm_down_steps     = 1000
ARM_MAX_VELOCITY   = 800    
ARM_ACCELERATION   = 400    
ARM_HOME_VELOCITY  = 300    

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
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

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

def home_arm(robot: Robot) -> None:
    robot.step_set_config(
        ARM_STEPPER,
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    ok = robot.step_home(
        ARM_STEPPER,
        direction=-1,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[WARN] arm homing timed out — check limit switch")

# --- LED Helpers ---
def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

def show_error_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 200)

def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)

# --- Vision Helpers ---
def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
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

# --- Placeholder Sensors ---
def sensor_confirms_grip() -> bool:
    return True 

def sensor_disk_is_ready_for_retry() -> bool:
    return True

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    timer_start = 0.0  
    
    # Drive control variables
    path_control_points = []
    obstacle_path_control_points = []
    remaining_path = []
    planner1 = None
    LOOKAHEAD_DIST = 50.0
    
    # Vision variables
    lights_off_at = 0.0
    last_shown_color = None
    
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        
        # --- Cancellation / Safety Check ---
        if state not in ["INIT", "IDLE", "ERROR_STEPPER", "ERROR_GRIPPER"] and robot.get_button(Button.BTN_2):
            print("[FSM] OPERATION CANCELLED! Stopping motors and returning to IDLE.")
            robot.stop() 
            show_idle_leds(robot)
            state = "IDLE"

        # ── INIT ──────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            
            print("[FSM] HOMING arm — do not obstruct the arm")
            home_arm(robot)
            
            print("[FSM] Loading path coordinates into memory...")
            path_control_points = [
                (0.0, 0.0), (0.0, 10.0), (0.0, 20.0), (0.0, 30.0), (0.0, 40.0),
                (0.0, 50.0), (0.0, 60.0), (0.0, 70.0), (0.0, 80.0), (0.0, 90.0),
                (0.0, 100.0), (0.0, 110.0), (0.0, 120.0), (0.0, 130.0), (0.0, 140.0),
                (0.0, 150.0), (0.0, 160.0), (0.0, 170.0), (0.0, 180.0), (0.0, 190.0),
                (0.0, 200.0), (0.0, 210.0), (0.0, 220.0), (0.0, 230.0), (0.0, 240.0),
                (0.0, 250.0), (0.0, 260.0), (0.0, 270.0), (0.0, 280.0), (0.0, 290.0),
                (0.0, 300.0), (0.0, 310.0), (0.0, 320.0), (0.0, 330.0), (0.0, 340.0),
                (0.0, 350.0), (0.0, 360.0), (0.0, 370.0), (0.0, 380.0), (0.0, 390.0),
                (0.0, 400.0), (0.0, 410.0), (0.0, 420.0), (0.0, 430.0), (0.0, 440.0),
                (0.0, 450.0), (0.0, 460.0), (0.0, 470.0), (0.0, 480.0), (0.0, 490.0),
                (0.0, 500.0), (0.1, 500.0), (10.0, 500.0), (20.0, 500.0), (30.0, 500.0),
                (40.0, 500.0), (50.0, 500.0), (60.0, 500.0), (70.0, 500.0), (80.0, 500.0),
                (90.0, 500.0), (100.0, 500.0), (110.0, 500.0), (120.0, 500.0), (130.0, 500.0),
                (140.0, 500.0), (150.0, 500.0), (160.0, 500.0), (170.0, 500.0), (180.0, 500.0),
                (190.0, 500.0), (200.0, 500.0), (210.0, 500.0), (220.0, 500.0), (230.0, 500.0),
                (240.0, 500.0), (250.0, 500.0), (260.0, 500.0), (270.0, 500.0), (280.0, 500.0),
                (290.0, 500.0), (300.0, 500.0), (310.0, 500.0), (320.0, 500.0), (330.0, 500.0),
                (340.0, 500.0), (350.0, 500.0), (360.0, 500.0), (370.0, 500.0), (380.0, 500.0),
                (390.0, 500.0), (400.0, 500.0), (410.0, 500.0), (420.0, 500.0), (430.0, 500.0),
                (440.0, 500.0), (450.0, 500.0), (460.0, 500.0), (470.0, 500.0), (480.0, 500.0),
                (490.0, 500.0), (500.0, 500.0), (500.0, 499.9), (500.0, 490.0), (500.0, 480.0),
                (500.0, 470.0), (500.0, 460.0), (500.0, 450.0), (500.0, 440.0), (500.0, 430.0),
                (500.0, 420.0), (500.0, 410.0), (500.0, 400.0), (500.0, 390.0), (500.0, 380.0),
                (500.0, 370.0), (500.0, 360.0), (500.0, 350.0), (500.0, 340.0), (500.0, 330.0),
                (500.0, 320.0), (500.0, 310.0), (500.0, 300.0), (500.0, 290.0), (500.0, 280.0),
                (500.0, 270.0), (500.0, 260.0), (500.0, 250.0), (500.0, 240.0), (500.0, 230.0),
                (500.0, 220.0), (500.0, 210.0), (500.0, 200.0), (500.0, 190.0), (500.0, 180.0),
                (500.0, 170.0), (500.0, 160.0), (500.0, 150.0), (500.0, 140.0), (500.0, 130.0),
                (500.0, 120.0), (500.0, 110.0), (500.0, 100.0), (500.0, 90.0), (500.0, 80.0),
                (500.0, 70.0), (500.0, 60.0), (500.0, 50.0), (500.0, 40.0), (500.0, 30.0),
                (500.0, 20.0), (500.0, 10.0), (500.0, 0.0), (499.9, 0.0), (490.0, 0.0),
                (480.0, 0.0), (470.0, 0.0), (460.0, 0.0), (450.0, 0.0), (440.0, 0.0),
                (430.0, 0.0), (420.0, 0.0), (410.0, 0.0), (400.0, 0.0), (390.0, 0.0),
                (380.0, 0.0), (370.0, 0.0), (360.0, 0.0), (350.0, 0.0), (340.0, 0.0),
                (330.0, 0.0), (320.0, 0.0), (310.0, 0.0), (300.0, 0.0), (290.0, 0.0),
                (280.0, 0.0), (270.0, 0.0), (260.0, 0.0), (250.0, 0.0), (240.0, 0.0),
                (230.0, 0.0), (220.0, 0.0), (210.0, 0.0), (200.0, 0.0), (190.0, 0.0),
                (180.0, 0.0), (170.0, 0.0), (160.0, 0.0), (150.0, 0.0), (140.0, 0.0),
                (130.0, 0.0), (120.0, 0.0), (110.0, 0.0), (100.0, 0.0), (90.0, 0.0),
                (80.0, 0.0), (70.0, 0.0), (60.0, 0.0), (50.0, 0.0), (40.0, 0.0),
                (30.0, 0.0), (20.0, 0.0), (10.0, 0.0), (0.0, 0.0)
            ]
            
            obstacle_path_control_points = [
               (300.0, 0.0), (300.0, 250.0), (300.0, 500.0), (300.0, 750.0),
               (300.0, 1000.0), (300.0, 1250.0), (300.0, 1500.0), (300.0, 1750.0),
               (300.0, 2000.0), (300.0, 2250.0), (300.0, 2500.0),
            ]
            
            show_idle_leds(robot)
            print("[FSM] IDLE — Waiting for sensor commands...")
            state = "IDLE"

        # ── IDLE (Hub State) ──────────────────────────────────────────────
        elif state == "IDLE":
            show_idle_leds(robot)
            
            # --- Event Triggers ---
            if robot.get_button(Button.BTN_1):
                state = "CMD_START_DRIVING"
            elif robot.get_button(Button.BTN_3):
                state = "CMD_LIFT_ARM"
            elif robot.get_button(Button.BTN_4):
                state = "CMD_LOWER_ARM"
            elif robot.get_button(Button.BTN_5):
                state = "CMD_OPEN_GRIPPER"
            elif robot.get_button(Button.BTN_6):
                state = "CMD_CLOSE_GRIPPER"
                
            # New integration triggers
            elif robot.get_button(Button.BTN_7):
                state = "WATCHING"
            elif robot.get_button(Button.BTN_8):
                state = "CMD_START_OBSTACLE_AVOIDANCE"

        # ── CAMERA WATCHING (Traffic Light) ───────────────────────────────
        elif state == "WATCHING":
            now = time.monotonic()
            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color == "red":
                robot.set_led(LED.RED, LED_BRIGHTNESS)
                robot.set_led(LED.GREEN, 0)
                robot.stop() # Ensure motors halt immediately
                if traffic_light_color != last_shown_color:
                    print("[VISION] Traffic Light: RED. Stopping.")
                last_shown_color = traffic_light_color

            elif traffic_light_color == "green":
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                print("[VISION] Traffic Light: GREEN. Commencing Drive Sequence.")
                state = "CMD_START_DRIVING" 

        # ── DISCRETE COMMAND: START DRIVING ───────────────────────────────
        elif state == "CMD_START_DRIVING":
            remaining_path = path_control_points.copy()
            planner1 = PurePursuitPlanner(
                lookahead_dist=LOOKAHEAD_DIST, 
                max_angular=1.5, 
                goal_tolerance=20.0, 
            )
            print("[ACTION] Path planned. Engaging pursuit algorithms...")
            show_running_leds(robot)
            state = "EXEC_DRIVING"

        # ── EXECUTION LOOP: DRIVING (Pure Pursuit) ────────────────────────
        elif state == "EXEC_DRIVING":
            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)
            
            remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, advance_radius_mm=LOOKAHEAD_DIST)
            current_pursuit_x, current_pursuit_y = planner1._lookahead_point(current_x, current_y, waypoints=remaining_path)
            
            linear_velocity_cmd, angular_velocity_cmd = planner1.compute_velocity(
                pose = (current_x, current_y, current_theta_rad),
                waypoints = remaining_path,
                max_linear = 80.0, 
            )
            robot.set_velocity(linear_velocity_cmd, math.degrees(angular_velocity_cmd)) 
            
            if planner1.CurrentTargetReached(current_pursuit_x, current_pursuit_y, current_x, current_y): 
                print("[ACTION] Destination reached! Stopping motors.")
                robot.stop()
                print("[FSM] Returned to IDLE")
                state = "IDLE"         

        # ── DISCRETE COMMAND: START OBSTACLE AVOIDANCE ────────────────────
        elif state == "CMD_START_OBSTACLE_AVOIDANCE":
            path = densify_polyline(obstacle_path_control_points, spacing=400.0)
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
                lane_width=350.0,
                obstacle_avoidance=True,
                x_L=300.0,
            )
            robot.planner.set_path(path)
            show_running_leds(robot)
            print("[ACTION] Obstacle Avoidance Engaged.")
            state = "EXEC_OBSTACLE_AVOIDANCE"

        # ── EXECUTION LOOP: OBSTACLE AVOIDANCE ────────────────────────────
        elif state == "EXEC_OBSTACLE_AVOIDANCE":
            show_moving_leds(robot)
            robot._draw_lidar_obstacles()
            
            next_state = robot._nav_follow_pp_path_loop()
            
            # Keep routing back to this execution state if the loop confirms it is still moving.
            if next_state == "MOVING":
                state = "EXEC_OBSTACLE_AVOIDANCE"
            else:
                state = next_state 

        # ── DISCRETE ACTION: LIFT ARM ─────────────────────────────────────
        elif state == "CMD_LIFT_ARM":
            show_running_leds(robot)
            print("[ACTION] Lifting arm...")
            robot.step_enable(ARM_STEPPER)
            success = robot.step_move(
                ARM_STEPPER, steps=arm_up_steps, move_type=StepMoveType.RELATIVE,
                blocking=True, timeout=10.0
            )
            if success:
                print("[ACTION] Lift complete. Returning to IDLE.")
                state = "IDLE"
            else:
                print("[ERROR] Stepper stalled or timed out during LIFT!")
                state = "ERROR_STEPPER"

        # ── DISCRETE ACTION: LOWER ARM ────────────────────────────────────
        elif state == "CMD_LOWER_ARM":
            show_running_leds(robot)
            print("[ACTION] Lowering arm...")
            robot.step_enable(ARM_STEPPER)
            success = robot.step_move(
                ARM_STEPPER, steps=arm_down_steps, move_type=StepMoveType.RELATIVE,
                blocking=True, timeout=10.0
            )
            if success:
                print("[ACTION] Lower complete. Returning to IDLE.")
                state = "IDLE"
            else:
                print("[ERROR] Stepper stalled or timed out during LOWER!")
                state = "ERROR_STEPPER"

        # ── DISCRETE ACTION: OPEN GRIPPER ─────────────────────────────────
        elif state == "CMD_OPEN_GRIPPER":
            show_running_leds(robot)
            print("[ACTION] Opening gripper...")
            robot.enable_servo(GRIPPER_CHANNEL)
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
            timer_start = time.monotonic() 
            state = "WAIT_GRIPPER_OPEN"

        # ── SERVO TIMING: WAIT FOR OPEN ───────────────────────────────────
        elif state == "WAIT_GRIPPER_OPEN":
            if time.monotonic() - timer_start >= 0.4:
                print("[ACTION] Gripper open. Returning to IDLE.")
                state = "IDLE"

        # ── DISCRETE ACTION: CLOSE GRIPPER ────────────────────────────────
        elif state == "CMD_CLOSE_GRIPPER":
            show_running_leds(robot)
            print("[ACTION] Closing gripper...")
            robot.enable_servo(GRIPPER_CHANNEL)
            robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
            timer_start = time.monotonic() 
            state = "WAIT_GRIPPER_CLOSE"

        # ── SERVO TIMING: WAIT FOR CLOSE (WITH SENSOR CHECK) ──────────────
        elif state == "WAIT_GRIPPER_CLOSE":
            if time.monotonic() - timer_start >= 0.5:
                if sensor_confirms_grip():
                    print("[ACTION] Grip successful. Returning to IDLE.")
                    state = "IDLE"
                else:
                    print("[ERROR] Grip failed! No disk detected in jaw.")
                    state = "ERROR_GRIPPER"

        # ── ERROR: STEPPER ────────────────────────────────────────────────
        elif state == "ERROR_STEPPER":
            show_error_leds(robot)
            if robot.get_button(Button.BTN_2):
                print("[RECOVERY] Clearing stepper error. Homing arm...")
                home_arm(robot) 
                state = "IDLE"

        # ── ERROR: GRIPPER ────────────────────────────────────────────────
        elif state == "ERROR_GRIPPER":
            show_error_leds(robot)
            if sensor_disk_is_ready_for_retry():
                print("[RECOVERY] Disk detected at station. Retrying grip...")
                state = "CMD_OPEN_GRIPPER" 
            elif robot.get_button(Button.BTN_2):
                print("[RECOVERY] Gripper error manually cleared.")
                state = "IDLE"

        # ── Tick-rate control ─────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()