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
from robot.path_planner2 import PurePursuitPlanner, generate_maze_waypoints
from robot.util import densify_polyline
import matplotlib.pyplot as plt
from robot.lidar_helpers import (
    get_front_distance,
    get_wall_alignment,
    save_lidar_plot,
    robot_align_to_wall,
)

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration (from main.py)
# ---------------------------------------------------------------------------
TAG_ID = 21 
WHEEL_DIAMETER_MAIN = 80.0 # main.py uses 80.0, hardware_map uses 74.0
# We will use the ones from main.py for the maze part, but they seem to be the same variables.
# Actually, main.py redefines them.

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
# Pure Pursuit Configuration (from main.py)
# ---------------------------------------------------------------------------
VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 250.0  
TOLERANCE_MM       = 80.0   
ADVANCE_RADIUS_MM  = 250.0  
MAX_ANGULAR_RAD_S  = 1.0    

STATUS_PRINT_INTERVAL_S = 0.5

PATH_CONTROL_POINTS = [
    (0, 1300),
    (0, 3750),
]

PATH_CONTROL_POINTS_2 = [(1150,3400),
    (800, 3300),
    (800, 3150),
    (800,2000),
    (850,1500),
    (850, 1000), 
    (850, 600), 
]

RUNBACK = False

# ---------------------------------------------------------------------------
# Obstacle Avoidance Configuration (from obstacleavoidance7final.py)
# ---------------------------------------------------------------------------
LAPF_GOAL_MM = (610.0, 610.0*5)
LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 1.0

LEASH_LENGTH_MM = 350.0 
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

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

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    
    # Combined Lidar setup
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
        wheel_diameter=WHEEL_DIAMETER_MAIN,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
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
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    global RUNBACK
    configure_robot(robot)
    state = "INIT"
    seq_step = 0
    drive_handle = None
    last_status_print_at = 0.0
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[STARTUP] Initialized. Press BTN_9 to start Maze, then Obstacle Avoidance.")

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
                print("[GPS] Synchronizing Odometry to GPS (Alpha Snap)...")
                robot.set_position_fusion_alpha(1.0)
                time.sleep(0.5)
                robot.set_position_fusion_alpha(0.01)
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
                drive_handle = robot.turn_by(-90.0, blocking=False)
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
                print("[RAMP] Step 1/4: Moving 150 mm forward to get closer to ramp")
                drive_handle = robot.move_forward(150.0, velocity=VELOCITY_MM_S, tolerance=10.0, blocking=False)
                seq_step = 1
            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[RAMP] Step 2/4: Aligning with ramp (front)...")
                    robot_align_to_wall(robot, 90.0, 0.0)
                    seq_step = 2
            elif seq_step == 2:
                print("[RAMP] Step 3/4: Driving straight 2715 mm (Manual)")
                drive_handle = robot.move_forward(2715.0, velocity=VELOCITY_MM_S, tolerance=50.0, blocking=False)
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
                    drive_handle = robot.move_forward(700.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 2
            elif seq_step == 2:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 3/3: Turning Left 90°")
                    drive_handle = robot.turn_by(90.0, blocking=False)
                    seq_step = 3
            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 200mm")
                    drive_handle = robot.move_forward(200.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 4
            elif seq_step == 4:
                if drive_handle is not None and drive_handle.is_finished():
                    robot.stop()
                    show_idle_leds(robot)
                    print("[SEQ] Maze Sequence complete! Transitioning to OBSTACLE_AVOIDANCE_START.")
                    RUNBACK = True
                    state = "OBSTACLE_AVOIDANCE_START"

        elif state == "OBSTACLE_AVOIDANCE_START":
            # Transition logic from obstacleavoidance7final.py
            reset_mission_pose(robot)
            show_running_leds(robot)
            drive_handle = start_lapf_goal(robot)
            last_status_print_at = now
            print("[FSM] OBSTACLE AVOIDANCE — LAPF goal started")
            state = "OBSTACLE_AVOIDANCE_MOVING"

        elif state == "OBSTACLE_AVOIDANCE_MOVING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_lapf_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] OBSTACLE AVOIDANCE — goal complete")
                print_lapf_status(robot)
                drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] ALL SEQUENCES COMPLETE. Returning to IDLE.")
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
