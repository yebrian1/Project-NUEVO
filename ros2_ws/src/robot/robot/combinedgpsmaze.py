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
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
)
from robot.lidar_helpers import (
    get_front_distance,
    get_wall_alignment,
    save_lidar_plot,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.path_planner2 import PurePursuitPlanner, generate_maze_waypoints
from robot.util import densify_polyline
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 # Updated to user's GPS ArUco tag
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 80.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG_CFG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Actuator Configuration (Arm & Gripper)
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
LOOKAHEAD_MM       = 250.0  # Increased for smoother turns
TOLERANCE_MM       = 80.0   # Relaxed tolerance for better arrival
ADVANCE_RADIUS_MM  = 250.0  # Allows skipping overshot points
MAX_ANGULAR_RAD_S  = 1.0    # Stability improvement

STATUS_PRINT_INTERVAL_S = 0.5

# ---------------------------------------------------------------------------
# OBSTACLE AVOIDANCE CONFIGURATION (from obstacleavoidance7final.py)
# ---------------------------------------------------------------------------
OA_GOAL_MM = (610.0, 3050.0)
OA_VELOCITY_MM_S = 150.0
OA_TOLERANCE_MM = 50.0
OA_MAX_ANGULAR_RAD_S = 1.0

# LAPF Tuning
LEASH_LENGTH_MM = 350.0 
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

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
def get_left_wall_alignment(points: np.ndarray, fov_deg: float = 30.0) -> dict | None:
    """Fits a line to the left wall (points at ~90° left of robot heading)."""
    if points.size == 0:
        return None

    x = points[:, 0]
    y = points[:, 1]

    # Left wall points: y is negative (left side), x is near zero
    # Select points in a cone pointing left: angle near -90°
    angles = np.arctan2(y, x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    target_angle = -math.pi / 2.0  # -90° = directly left

    mask = np.abs(angles - target_angle) <= half_fov_rad
    xf = x[mask]
    yf = y[mask]

    if xf.size < 5:
        return None

    # Fit line: x as a function of y (wall runs along Y axis in robot frame)
    m, c = np.polyfit(yf, xf, 1)

    predicted_x = m * yf + c
    ss_res = np.sum((xf - predicted_x) ** 2)
    ss_tot = np.sum((xf - np.mean(xf)) ** 2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0.0001 else 0.0

    # tilt_deg: how much the robot is rotated relative to the wall
    # m is the slope of x vs y; zero slope = robot parallel to wall
    tilt_deg = math.degrees(math.atan(m))

    return {
        'tilt_deg': float(tilt_deg),
        'dist_mm': float(c),          # perpendicular distance to wall
        'points': int(xf.size),
        'error': float(np.sqrt(ss_res / xf.size)),
        'r2': float(r2),
    }

# Simplified path to avoid tethering issues at corners
PATH_CONTROL_POINTS = [
    (335, 1300),
    (345, 3750),  # Straight
]

PATH_CONTROL_POINTS_2 = [(1150,3400),
    (1150, 3300),
    (1150, 3150),
    (1150,2000),
    (1200,1500),
    (1200, 1000), 
    (1250, 600), # Corner 3: Turn Right
]

RUNBACK = False

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
ENABLE_GPS = True
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    
    # Lidar configuration - Start with no filter for maximum data availability
    # We will only enable the specific filter right before the turn.
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(range_min_mm=0.0)
    # robot.start_lidar_world_publisher() # Keep this disabled to rely on GPS

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG_CFG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
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

def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "

    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()
    if virtual_target is None:
        vt_summary = ""
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
        track_summary = f" tracked={len(obstacle_tracks)} nearest={nearest_boundary_mm:.0f}mm"
    else:
        track_summary = " tracked=0"

    print(
        f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°"
        f"{vt_summary}{track_summary}"
        f" gps={'fresh' if robot.is_gps_active() else 'stale'}"
    )


# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    global RUNBACK
    configure_robot(robot)
    state = "INIT"
    seq_step = 0
    state_timer_end = 0.0
    drive_handle = None
    last_status_print_at = 0.0
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()
        
        # --- LIDAR DEBUG BUTTONS ---
        if robot.was_button_pressed(Button.BTN_1):
            points = np.asarray(robot.get_obstacles())
            avg_dist, count = get_front_distance(points, fov_deg=5.0)
            if avg_dist is not None:
                print(f"[DEBUG DIST] Avg: {avg_dist:.2f} mm | Points: {count}")
            else:
                print("[DEBUG DIST] No points in 5 deg cone.")

        if robot.was_button_pressed(Button.BTN_2):
            points = np.asarray(robot.get_obstacles())
            ref_dist, _ = get_front_distance(points, fov_deg=5.0)
            result = get_wall_alignment(points, fov_deg=15.0, ref_dist=ref_dist)
            if result:
                tilt = result['tilt_deg']
                print(f"[DEBUG ALIGN] Tilt: {tilt:+.2f} deg")
            else:
                print("[DEBUG ALIGN] Not enough points for tilt.")

        if robot.was_button_pressed(Button.BTN_3):
            save_lidar_plot(robot)

        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
            robot.stop() 
            show_idle_leds(robot)
            print("[FSM] IDLE — path cancelled/stopped")
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            print("[FSM] IDLE — press BTN_9 to start Maze")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.was_button_pressed(Button.BTN_9):
                state = "CMD_MAZE_START"

        elif state == "CMD_MAZE_START":
            show_running_leds(robot)
            print("[INIT] Waiting for firmware bridge...")
            bridge_wait = time.monotonic()
            while time.monotonic() - bridge_wait < 5.0:
                if robot.get_state() == FirmwareState.RUNNING:
                    break
                time.sleep(0.2)
            
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
                time.sleep(0.5)

            if not RUNBACK:
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=PATH_CONTROL_POINTS,
                    velocity=VELOCITY_MM_S,
                    lookahead=LOOKAHEAD_MM,
                    tolerance=TOLERANCE_MM,
                    advance_radius=ADVANCE_RADIUS_MM,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False,
                )
            else:
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=PATH_CONTROL_POINTS_2,
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
            if RUNBACK:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    curr_x, curr_y, _ = robot.get_pose()
                    target_wp = PATH_CONTROL_POINTS_2[-1]
                    for wp in PATH_CONTROL_POINTS_2:
                        if math.hypot(wp[0] - curr_x, wp[1] - curr_y) > LOOKAHEAD_MM:
                            target_wp = wp
                            break
                    print(f"  target_wp=({target_wp[0]:.0f}, {target_wp[1]:.0f})")
                    last_status_print_at = now
                
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    robot.stop()
                    show_idle_leds(robot)
                    seq_step = 0
                    state = "EXEC_TURN_SEQUENCE_2"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    curr_x, curr_y, _ = robot.get_pose()
                    target_wp = PATH_CONTROL_POINTS[-1]
                    for wp in PATH_CONTROL_POINTS:
                        if math.hypot(wp[0] - curr_x, wp[1] - curr_y) > LOOKAHEAD_MM:
                            target_wp = wp
                            break
                    print(f"  target_wp=({target_wp[0]:.0f}, {target_wp[1]:.0f})")
                    last_status_print_at = now
                
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    robot.stop()
                    show_idle_leds(robot)
                    seq_step = 0
                    state = "ALIGN_BEFORE_TURN"

        elif state == "ALIGN_BEFORE_TURN":
            points = np.asarray(robot.get_obstacles())
            align_data = get_left_wall_alignment(points, fov_deg=40.0)

            if align_data and align_data['points'] >= 5 and align_data['r2'] >= 0.80:
                tilt = align_data['tilt_deg']
                print(f"[ALIGN] Left wall tilt={tilt:+.2f}° dist={align_data['dist_mm']:.0f}mm r²={align_data['r2']:.2f}")
                if abs(tilt) <= 1.0:
                    robot.stop()
                    print(f"[FSM] Alignment complete. Starting sequence.")
                    seq_step = 0
                    state = "EXEC_TURN_SEQUENCE"
                else:
                    Kp_align = 0.03
                    angular_cmd = max(-0.6, min(0.6, Kp_align * tilt))
                    robot.set_velocity(0.0, angular_cmd)
            else:
                print(f"[WARN] Left wall not visible ")
                print(f"({'no data' if align_data is None else f'pts={align_data['points']} r²={align_data['r2']:.2f}'}).")
                print(f"Proceeding without alignment.")
                robot.stop()
                seq_step = 0
                state = "EXEC_TURN_SEQUENCE"

        elif state == "EXEC_TURN_SEQUENCE":
            if seq_step == 0:
                print("[SEQ] Step 1/3: Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 600 mm")
                    drive_handle = robot.move_forward(600.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
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
                    print("[SEQ] Sequence complete! Transitioning to CMD_MAZE_START.")
                    RUNBACK = True
                    state = "CMD_MAZE_START"

        elif state == "EXEC_TURN_SEQUENCE_2":
            if seq_step == 0:
                print("[SEQ2] Step 1/4: Turning Left 90°")
                drive_handle = robot.turn_by(90.0, blocking=False)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ2] Step 2/4: Driving Straight 700mm")
                    drive_handle = robot.move_forward(700.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 2
            elif seq_step == 2:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ2] Step 3/4: Turning Left 90°")
                    drive_handle = robot.turn_by(90.0, blocking=False)
                    seq_step = 3
            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ2] Step 4/4: Driving Straight 200mm")
                    drive_handle = robot.move_forward(200.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 4
            elif seq_step == 4:
                if drive_handle is not None and drive_handle.is_finished():
                    robot.stop()
                    show_idle_leds(robot)
                    print("[SEQ2] Sequence complete! Transitioning to OBSTACLE_AVOIDANCE.")
                    state = "CMD_OBSTACLE_AVOIDANCE_START"

        elif state == "CMD_OBSTACLE_AVOIDANCE_START":
            show_running_leds(robot)
            print("[FSM] STARTING OBSTACLE AVOIDANCE (LAPF)")
            cfg = resolve_lapf_config()
            drive_handle = robot.lapf_to_goal(
                OA_GOAL_MM[0],
                OA_GOAL_MM[1],
                velocity=OA_VELOCITY_MM_S,
                tolerance=OA_TOLERANCE_MM,
                leash_length_mm=cfg["leash_length_mm"],
                repulsion_range_mm=cfg["repulsion_range_mm"],
                target_speed_mm_s=cfg["target_speed_mm_s"],
                max_angular_rad_s=OA_MAX_ANGULAR_RAD_S,
                repulsion_gain=cfg["repulsion_gain"],
                attraction_gain=cfg["attraction_gain"],
                force_ema_alpha=cfg["force_ema_alpha"],
                inflation_margin_mm=cfg["inflation_margin_mm"],
                leash_half_angle_deg=cfg["leash_half_angle_deg"],
                blocking=False,
            )
            last_status_print_at = now
            state = "EXEC_OBSTACLE_AVOIDANCE"

        elif state == "EXEC_OBSTACLE_AVOIDANCE":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] DONE — Obstacle avoidance complete")
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
