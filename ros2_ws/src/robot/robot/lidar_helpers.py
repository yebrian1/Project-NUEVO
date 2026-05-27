import math
import numpy as np
import matplotlib.pyplot as plt
from robot.robot import Robot

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
    # Positive m means x increases as y increases (left side is further)
    # We want negative tilt for LEFT (left is further) and positive for RIGHT (right is further)
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

def print_lidar_stats(robot: Robot, step_name: str) -> None:
    """Helper to quickly print distance and alignment."""
    points = np.asarray(robot.get_obstacles())
    if points.size == 0:
        print(f"[{step_name}] No LiDAR points available.")
        return

    ref_dist, count = get_front_distance(points, 5.0)
    if ref_dist is not None:
        print(f"[{step_name}] Distance: {ref_dist:.1f} mm ({count} pts)")
    else:
        print(f"[{step_name}] Distance: None")
        
    res = get_wall_alignment(points, 15.0, ref_dist)
    if res:
        direction = "RIGHT" if res['tilt_deg'] > 0 else "LEFT"
        print(f"[{step_name}] Alignment: {res['tilt_deg']:+.2f} deg ({direction}) | RMSE: {res['error']:.1f} mm | R2: {res['r2']:.2f}")
    else:
        print(f"[{step_name}] Alignment: Not enough points for tilt.")
