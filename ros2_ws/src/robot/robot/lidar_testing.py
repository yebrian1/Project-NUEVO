from __future__ import annotations
import time
import math
import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
)
from robot.robot import FirmwareState, Robot, Unit

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

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


def get_wall_alignment(points: np.ndarray, fov_deg: float = 20.0) -> dict | None:
    """
    Fits a line to points in a front cone to determine the tilt of a flat wall.
    
    Returns:
        { 'tilt_deg': float, 'points': int, 'error': float } or None
    """
    if points.size == 0:
        return None

    x = points[:, 0]
    y = points[:, 1]

    # Filter for points in front (x > 0) and within FOV
    angles = np.arctan2(y, x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    mask = (x > 0) & (np.abs(angles) <= half_fov_rad)
    
    xf = x[mask]
    yf = y[mask]

    if xf.size < 5:
        return None

    # Linear Regression: x = m*y + c
    m, c = np.polyfit(yf, xf, 1)

    # m = dx/dy. The angle is atan(m).
    tilt_rad = math.atan(m)
    tilt_deg = math.degrees(tilt_rad)

    # Quality of fit (Standard Error of the estimate)
    predicted_x = m * yf + c
    residuals = xf - predicted_x
    std_err = float(np.sqrt(np.sum(residuals**2) / xf.size))

    return {
        'tilt_deg': float(tilt_deg),
        'points': int(xf.size),
        'error': std_err
    }

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
    )
    
    print("[LIDAR TEST] Enabling LIDAR...")
    robot.enable_lidar()

    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    print("\n" + "="*40)
    print("LIDAR TESTING TOOL LOADED")
    print("Button 1: Print Front Distance (5 deg cone)")
    print("Button 2: Print Wall Alignment (20 deg cone)")
    print("="*40 + "\n")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        points = robot.get_obstacles()

        if robot.was_button_pressed(Button.BTN_1):
            avg_dist, count = get_front_distance(points, fov_deg=5.0)
            if avg_dist is not None:
                print(f"[DISTANCE] Avg: {avg_dist:.2f} mm | Points: {count}")
            else:
                print("[DISTANCE] No points detected in 5 degree front cone.")

        if robot.was_button_pressed(Button.BTN_2):
            result = get_wall_alignment(points, fov_deg=20.0)
            if result:
                tilt = result['tilt_deg']
                direction = "RIGHT" if tilt > 0 else "LEFT"
                print(f"[ALIGNMENT] Tilt: {tilt:+.2f} deg (Skewed {direction}) | "
                      f"Points: {result['points']} | StdErr: {result['error']:.2f}mm")
            else:
                print("[ALIGNMENT] Not enough points to determine wall tilt.")

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
    node = RobotNode("lidar_testing_tool")
    robot = Robot(node)
    
    try:
        run(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        rclpy.shutdown()