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
from robot.util import densify_polyline
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 # Updated to user's GPS ArUco tag
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

# Simplified path to avoid tethering issues at corners
PATH_CONTROL_POINTS = [
    (325, 1300),
    (325, 3700),  # Straight
    (520, 3850),  # Midpoint for smoother turn

    (700, 3800),  # Corner 2: Turn Right
    (750, 3750),
    (808, 3700),  # Corner 2: Turn Right
    (813,3400),
    (825, 2500),
    (825,2000),
    (825, 1800),  # Corner 3: Turn Right
    (827, 800),
    (1240, 1000),  # Corner 4: Turn Left
    (1240, 1500),  # Final Goal
]

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
ENABLE_GPS = True
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# LIDAR Helper Functions
# ---------------------------------------------------------------------------

def get_front_distance(points: np.ndarray, fov_deg: float = 5.0) -> tuple[float | None, int]:
    """Calculates the average distance to objects in a narrow front cone."""
    if points.size == 0:
        return None, 0
    x = points[:, 0]
    y = points[:, 1]
    front_mask = x > 0
    xf = x[front_mask]
    yf = y[front_mask]
    if xf.size == 0:
        return None, 0
    angles = np.arctan2(yf, xf)
    half_fov_rad = math.radians(fov_deg / 2.0)
    fov_mask = np.abs(angles) <= half_fov_rad
    final_points = xf[fov_mask]
    if final_points.size == 0:
        return None, 0
    return float(np.mean(final_points)), final_points.size

def get_wall_alignment(points: np.ndarray, fov_deg: float = 15.0, ref_dist: float | None = None) -> dict | None:
    """Fits a line to determine the tilt of a flat wall."""
    if points.size == 0:
        return None
    x = points[:, 0]
    y = points[:, 1]
    angles = np.arctan2(y, x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    mask = (x > 0) & (np.abs(angles) <= half_fov_rad)
    xf = x[mask]
    yf = y[mask]
    if xf.size < 5:
        return None
    target_dist = ref_dist if ref_dist is not None else np.median(xf)
    dist_mask = (xf >= target_dist * 0.7) & (xf <= target_dist * 1.3)
    xf = xf[dist_mask]
    yf = yf[dist_mask]
    if xf.size < 5:
        return None
    m, c = np.polyfit(yf, xf, 1)
    predicted_x = m * yf + c
    ss_res = np.sum((xf - predicted_x)**2)
    ss_tot = np.sum((xf - np.mean(xf))**2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0.0001 else 0.0
    tilt_deg = -math.degrees(math.atan(m))
    return {
        'tilt_deg': float(tilt_deg),
        'points': int(xf.size),
        'error': float(np.sqrt(ss_res / xf.size)),
        'r2': float(r2)
    }

def save_lidar_plot(robot: Robot, filename: str = "lidar_temp.png") -> None:
    """Saves a plot of the current LiDAR scan to a PNG file."""
    points_list = robot.get_obstacles()
    if not points_list:
        print("[LIDAR] No points to plot.")
        return
    points = np.asarray(points_list)
    plot_x = -points[:, 1]
    plot_y = points[:, 0]
    plt.figure(figsize=(8, 8))
    plt.scatter(plot_x, plot_y, s=2, c='red')
    plt.axhline(0, color='black', lw=1, alpha=0.5)
    plt.axvline(0, color='black', lw=1, alpha=0.5)
    robot_body = plt.Circle((0, 0), 100, color='blue', fill=False, label='Robot Center')
    plt.gca().add_patch(robot_body)
    plt.title(f"LiDAR Scan (Front is UP) - {len(points)} points")
    plt.xlabel("Left <--- (mm) ---> Right")
    plt.ylabel("Back <--- (mm) ---> Front")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.7)
    import os
    save_path = os.path.join("/runtime_output", filename)
    try:
        plt.savefig(save_path)
        print(f"[LIDAR] Plot saved to {save_path}")
    except Exception as e:
        print(f"[LIDAR] Failed to save plot: {e}")
    finally:
        plt.close()

# ---------------------------------------------------------------------------
# Helpers
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

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
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

            # Wait for firmware bridge
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
            
            # Increased timeout to 30 seconds to allow for camera stabilization/detection
            while time.monotonic() - gps_wait < 30.0:
                if robot.is_gps_active():
                    # Double check if coordinates are non-zero/valid
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
                # Give the fusion filter a moment to settle on the first GPS fix
                time.sleep(0.5)

            # Use built-in pure-pursuit with simplified path and robust parameters
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
                
                # Simple target waypoint estimation for logging
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
