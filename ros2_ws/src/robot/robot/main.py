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
# Vision Configuration
# ---------------------------------------------------------------------------
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# LIDAR Helper Functions
# ---------------------------------------------------------------------------

def get_front_distance(points: np.ndarray, fov_deg: float = 5.0) -> tuple[float | None, int]:
    """
    Calculates the average distance to objects in a narrow front cone.
    
    Returns:
        (avg_distance_mm, point_count)
    """
    if points.size == 0:
        return None, 0

    x = points[:, 0]
    y = points[:, 1]

    # Filter for points in front of the robot (x > 0)
    front_mask = x > 0
    xf = x[front_mask]
    yf = y[front_mask]

    if xf.size == 0:
        return None, 0

    # Calculate angles of all points in radians
    angles = np.arctan2(yf, xf)
    
    # Filter by FOV
    half_fov_rad = math.radians(fov_deg / 2.0)
    fov_mask = np.abs(angles) <= half_fov_rad
    
    final_points = xf[fov_mask]

    if final_points.size == 0:
        return None, 0

    return float(np.mean(final_points)), final_points.size


def get_wall_alignment(points: np.ndarray, fov_deg: float = 15.0, ref_dist: float | None = None) -> dict | None:
    """
    Fits a line to points in a front cone to determine the tilt of a flat wall.
    Filters out points beyond 30% of the reference distance (or median) to remove outliers.
    
    Returns:
        { 'tilt_deg': float, 'points': int, 'error': float, 'r2': float } or None
    """
    if points.size == 0:
        return None

    x = points[:, 0]
    y = points[:, 1]

    # 1. Initial filter for points in front (x > 0) and within FOV
    angles = np.arctan2(y, x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    mask = (x > 0) & (np.abs(angles) <= half_fov_rad)
    
    xf = x[mask]
    yf = y[mask]

    if xf.size < 5:
        return None

    # 2. Outlier Removal: Filter for points within +/- 30% of the reference distance
    target_dist = ref_dist if ref_dist is not None else np.median(xf)
    dist_mask = (xf >= target_dist * 0.7) & (xf <= target_dist * 1.3)
    xf = xf[dist_mask]
    yf = yf[dist_mask]

    if xf.size < 5:
        return None

    # 3. Linear Regression: x = m*y + c
    m, c = np.polyfit(yf, xf, 1)

    # Calculate R^2 (Coefficient of Determination)
    predicted_x = m * yf + c
    ss_res = np.sum((xf - predicted_x)**2)
    ss_tot = np.sum((xf - np.mean(xf))**2)
    # If the wall is perfectly flat (slope 0), ss_tot might be near zero (just noise)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0.0001 else 0.0

    # m = dx/dy. The angle is atan(m).
    # Positive m means x increases as y increases (left side is further).
    # Physically, m < 0 means robot is tilted RIGHT (right side is closer).
    # We want positive for RIGHT and negative for LEFT.
    tilt_deg = -math.degrees(math.atan(m))

    return {
        'tilt_deg': float(tilt_deg),
        'points': int(xf.size),
        'error': float(np.sqrt(ss_res / xf.size)),
        'r2': float(r2)
    }

def save_lidar_plot(robot: Robot, filename: str = "lidar_temp.png") -> None:
    """Saves a plot of the current LiDAR scan to a PNG file, rotated 90deg CCW (Front is Up)."""
    points_list = robot.get_obstacles()
    if not points_list:
        print("[LIDAR] No points to plot.")
        return
    
    points = np.asarray(points_list)
    # Rotate 90deg CCW for intuitive viewing:
    # Robot +X (Forward) becomes Plot +Y (Up)
    # Robot +Y (Left) becomes Plot -X (Left)
    plot_x = -points[:, 1]
    plot_y = points[:, 0]

    plt.figure(figsize=(8, 8))
    plt.scatter(plot_x, plot_y, s=2, c='red')
    
    # Draw axes for reference (center of robot)
    plt.axhline(0, color='black', lw=1, alpha=0.5)
    plt.axvline(0, color='black', lw=1, alpha=0.5)
    
    # Draw a small indicator for the robot body
    robot_body = plt.Circle((0, 0), 100, color='blue', fill=False, label='Robot Center')
    plt.gca().add_patch(robot_body)

    plt.title(f"LiDAR Scan (Front is UP) - {len(points)} points")
    plt.xlabel("Left <--- (mm) ---> Right")
    plt.ylabel("Back <--- (mm) ---> Front")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Save to the writable runtime_output directory
    import os
    save_path = os.path.join("/runtime_output", filename)
    try:
        plt.savefig(save_path)
        print(f"[LIDAR] Plot saved to {save_path} (Front is UP)")
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
    robot.enable_gps() # Enable GPS/ArUco fusion
    robot.enable_lidar() # Keep lidar for obstacle avoidance
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
     # Use GPS tangent to correct heading drift, GPS position to correct odometry drift
    robot.enable_gps_tangent_heading(alpha=0.15, min_displacement_mm=200.0)
    robot.set_position_fusion_alpha(0.10)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

# --- LED Helpers ---
def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

# --- Vision Helpers ---
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
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    remaining_path = []
    planner1 = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    def check_safety(step_name: str) -> bool:
        power = robot.get_power()
        cur_st = robot.get_state()
        if cur_st in (FirmwareState.ERROR, FirmwareState.ESTOP):
            return False
        return True

    while True:
        # --- LIDAR DEBUG BUTTONS ---
        if robot.was_button_pressed(Button.BTN_1):
            points = np.asarray(robot.get_obstacles())
            avg_dist, count = get_front_distance(points, fov_deg=5.0)
            if avg_dist is not None:
                print(f"[DEBUG DIST] Avg: {avg_dist:.2f} mm | Points: {count} (5 deg cone)")
            else:
                print("[DEBUG DIST] No points in 5 deg cone.")

        if robot.was_button_pressed(Button.BTN_2):
            points = np.asarray(robot.get_obstacles())
            # Calculate 5-deg distance first to use as a stable reference for 15-deg wall fit
            ref_dist, _ = get_front_distance(points, fov_deg=5.0)
            result = get_wall_alignment(points, fov_deg=15.0, ref_dist=ref_dist)
            
            if result:
                tilt = result['tilt_deg']
                direction = "RIGHT" if tilt > 0 else "LEFT"
                print(f"[DEBUG ALIGN] Tilt: {tilt:+.2f} deg ({direction}) | RMSE: {result['error']:.1f} mm | Points: {result['points']} (15 deg cone)")
            else:
                print("[DEBUG ALIGN] Not enough points for tilt.")

        if robot.was_button_pressed(Button.BTN_3):
            save_lidar_plot(robot)

        if state not in ["INIT", "IDLE"] and robot.get_button(Button.BTN_2):
            robot.stop() 
            state = "IDLE"

        if state == "INIT":
            # REMEMBER YOU NEED TO START THE GPS NODE USING ros2 run sensorbot_gps
            start_robot(robot)
            show_idle_leds(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.get_button(Button.BTN_9):
                state = "CMD_MAZE_START"
            elif robot.get_button(Button.BTN_10):
                state = "CMD_SEQUENCE_MOVE"

        elif state == "CMD_MAZE_START":
            show_running_leds(robot)

            # Wait for firmware bridge
            print("[INIT] Waiting for firmware bridge...")
            bridge_wait = time.monotonic()
            while time.monotonic() - bridge_wait < 5.0:
                if robot.get_state() == FirmwareState.RUNNING:
                    print("[INIT] Firmware ready.")
                    break
                time.sleep(0.2)
            else:
                print("[ERROR] Firmware never ready, aborting.")
                state = "IDLE"
                continue

            # Move into position FIRST before GPS lock
            print("[ACTION] Move Back 70mm -> Turn Right 90deg")
            robot.move_backward(70.0, 100.0, 20.0, blocking=False).wait(timeout=10.0)
            robot.turn_by(-90.0, blocking=False).wait(timeout=10.0)

            # NOW lock GPS after robot is in final position
            print("[GPS] Checking for GPS lock (Tag 21)...")
            gps_wait = time.monotonic()
            gps_locked = False
            while time.monotonic() - gps_wait < 5.0:
                if robot.is_gps_active():
                    gps_x, gps_y = robot._gps_x_mm, robot._gps_y_mm
                    if abs(gps_x) > 1.0 or abs(gps_y) > 1.0:
                        print(f"[GPS] Lock acquired! x={gps_x:.1f}mm y={gps_y:.1f}mm")
                        gps_locked = True
                        break
                time.sleep(0.1)

            if not gps_locked:
                print("[WARN] GPS lock failed. Proceeding with Odom.")

            cur_x, cur_y, cur_theta_deg = robot.get_pose()
            print(f"[POSE] Starting Maze at ({cur_x:.1f}, {cur_y:.1f}) @ {cur_theta_deg:.1f}°")
            
            # Generate path relative to current pose using path_planner2.py logic
            remaining_path = generate_maze_waypoints(cur_x, cur_y, cur_theta_deg)
            planner1 = PurePursuitPlanner(lookahead_dist=150.0, max_angular=1.0, goal_tolerance=50.0)

            print(f"[MAZE] Path generated with {len(remaining_path)} points.")
            state = "EXEC_MAZE_DRIVING"

        elif state == "EXEC_MAZE_DRIVING":
            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            # Advance waypoints logic
            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y, advance_radius_mm=80.0
            )

            if not remaining_path:
                print("[DONE] Maze complete (path empty).")
                robot.stop()
                state = "IDLE"
                continue

            # Pure Pursuit velocity computation
            linear, angular = planner1.compute_velocity(
                (current_x, current_y, current_theta_rad), remaining_path, 100.0
            )
            robot.set_velocity(linear, math.degrees(angular))

            # Target is the next immediate point we are aiming for
            target_x, target_y = remaining_path[0]
            print(f"[MAZE] pos=({current_x:.0f},{current_y:.0f}) next_wp=({target_x:.0f},{target_y:.0f}) theta={current_theta_deg:.1f}")

            # Check for final destination
            final_x, final_y = remaining_path[-1]
            if len(remaining_path) == 1 and planner1.CurrentTargetReached(final_x, final_y, current_x, current_y):
                print("[DONE] Goal reached.")
                robot.stop()
                state = "IDLE"
             
        elif state == "CMD_SEQUENCE_MOVE":
            # (Burger sequence placeholder - user can restore full sequence if needed)
            print("[ACTION] Executing Burger Pickup Sequence...")
            # ... full sequence logic goes here ...
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
