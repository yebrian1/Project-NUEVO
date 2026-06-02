import math
import numpy as np
import matplotlib.pyplot as plt
from robot.robot import Robot

def get_distance_at_angle(points: np.ndarray, angle_deg: float, fov_deg: float = 10.0) -> tuple[float | None, int]:
    """
    Generic distance check at any angle (0=Front, 90=Left, -90=Right, 180=Back).
    Rotates the points so the target angle is 'Front' and reuses front logic.
    """
    if points.size == 0:
        return None, 0

    # Rotate points so the target angle is at 0 (Front)
    rad = math.radians(-angle_deg)
    c, s = math.cos(rad), math.sin(rad)
    rot_x = points[:, 0] * c - points[:, 1] * s
    rot_y = points[:, 0] * s + points[:, 1] * c
    
    # Filter for points in "front" of the rotated frame
    mask = rot_x > 0
    xf = rot_x[mask]
    yf = rot_y[mask]

    if xf.size == 0:
        return None, 0

    # Filter by FOV
    angles = np.arctan2(yf, xf)
    half_fov_rad = math.radians(fov_deg / 2.0)
    fov_mask = np.abs(angles) <= half_fov_rad
    
    final_points = xf[fov_mask]

    if final_points.size == 0:
        return None, 0

    return float(np.mean(final_points)), final_points.size


def get_alignment_at_angle(points: np.ndarray, angle_deg: float, fov_deg: float = 30.0, ref_dist: float | None = None) -> dict | None:
    """
    Fits a line to points at a specific angle to determine tilt relative to the robot's axis.
    Uses a larger 30deg FOV by default for better stability.
    """
    if points.size == 0:
        return None

    # Rotate points so the target angle is at 0 (Front)
    rad = math.radians(-angle_deg)
    c, s = math.cos(rad), math.sin(rad)
    rot_x = points[:, 0] * c - points[:, 1] * s
    rot_y = points[:, 0] * s + points[:, 1] * c

    # Filter for points in "front" of the rotated frame and within FOV
    angles = np.arctan2(rot_y, rot_x)
    half_fov_rad = math.radians(fov_deg / 2.0)
    mask = (rot_x > 0) & (np.abs(angles) <= half_fov_rad)
    
    xf = rot_x[mask]
    yf = rot_y[mask]

    if xf.size < 5:
        return None

    # Outlier Removal: Filter for points within +/- 15% of the reference distance
    # We tightened this from 30% to 15% to ignore objects behind the wall more effectively.
    target_dist = ref_dist if ref_dist is not None else np.median(xf)
    dist_mask = (xf >= target_dist * 0.85) & (xf <= target_dist * 1.15)
    xf = xf[dist_mask]
    yf = yf[dist_mask]

    if xf.size < 8: # Require slightly more points for a 30deg fit
        return None

    # Linear Regression: x = m*y + c
    try:
        m, c = np.polyfit(yf, xf, 1)
    except:
        return None

    predicted_x = m * yf + c
    ss_res = np.sum((xf - predicted_x)**2)
    ss_tot = np.sum((xf - np.mean(xf))**2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0.0001 else 0.0

    # m = dx/dy. The angle is atan(m).
    tilt_deg = -math.degrees(math.atan(m))

    return {
        'tilt_deg': float(tilt_deg),
        'points': int(xf.size),
        'error': float(np.sqrt(ss_res / xf.size)), # RMSE in mm
        'r2': float(r2)
    }

def get_front_distance(points: np.ndarray, fov_deg: float = 10.0) -> tuple[float | None, int]:
    return get_distance_at_angle(points, 0.0, fov_deg)

def get_left_distance(points: np.ndarray, fov_deg: float = 10.0) -> tuple[float | None, int]:
    return get_distance_at_angle(points, 90.0, fov_deg)

def get_right_distance(points: np.ndarray, fov_deg: float = 10.0) -> tuple[float | None, int]:
    return get_distance_at_angle(points, -90.0, fov_deg)

def get_wall_alignment(points: np.ndarray, fov_deg: float = 30.0, ref_dist: float | None = None) -> dict | None:
    return get_alignment_at_angle(points, 0.0, fov_deg, ref_dist)

def get_left_alignment(points: np.ndarray, fov_deg: float = 30.0, ref_dist: float | None = None) -> dict | None:
    return get_alignment_at_angle(points, 90.0, fov_deg, ref_dist)

def get_right_alignment(points: np.ndarray, fov_deg: float = 30.0, ref_dist: float | None = None) -> dict | None:
    return get_alignment_at_angle(points, -90.0, fov_deg, ref_dist)

def print_all_sides_stats(robot: Robot, step_name: str) -> None:
    """Helper to print distance and alignment for Front, Left, and Right sides with diagnostics."""
    points = np.asarray(robot.get_obstacles())
    if points.size == 0:
        print(f"[{step_name}] No LiDAR points available.")
        return

    # Get distances (using 10deg cone for more samples than 5deg)
    f_dist, f_count = get_front_distance(points)
    l_dist, l_count = get_left_distance(points)
    r_dist, r_count = get_right_distance(points)
    
    # Get alignments (using 30deg cone for stability)
    f_res = get_wall_alignment(points, ref_dist=f_dist)
    l_res = get_left_alignment(points, ref_dist=l_dist)
    r_res = get_right_alignment(points, ref_dist=r_dist)

    def format_align(res):
        if not res: return "None"
        # Print Tilt, RMSE (error), and R2 (fit quality)
        return f"{res['tilt_deg']:+5.1f}° [RMSE:{res['error']:4.1f}mm, R2:{res['r2']:.2f}]"

    print(f"\n--- LiDAR Report: {step_name} ---")
    print(f"FRONT: {f_dist if f_dist else 0.0:6.1f} mm ({f_count:3} pts) | {format_align(f_res)}")
    print(f"LEFT : {l_dist if l_dist else 0.0:6.1f} mm ({l_count:3} pts) | {format_align(l_res)}")
    print(f"RIGHT: {r_dist if r_dist else 0.0:6.1f} mm ({r_count:3} pts) | {format_align(r_res)}")
    print("------------------------------------------")

def save_lidar_plot(robot: Robot, filename: str = "lidar_temp.png") -> None:
    """Saves a plot of the current LiDAR scan to a PNG file, rotated 90deg CCW (Front is Up)."""
    points_list = robot.get_obstacles()
    if not points_list:
        print("[LIDAR] No points to plot.")
        return
    
    points = np.asarray(points_list)
    # Rotate 90deg CCW for intuitive viewing:
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
        print(f"[LIDAR] Plot saved to {save_path} (Front is UP)")
    except Exception as e:
        print(f"[LIDAR] Failed to save plot: {e}")
    finally:
        plt.close()
