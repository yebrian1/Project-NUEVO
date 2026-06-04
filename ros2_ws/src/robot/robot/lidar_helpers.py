from __future__ import annotations
import time
import math
import numpy as np
import matplotlib.pyplot as plt

from robot.hardware_map import Button
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

    # Use median for better outlier rejection (e.g. seeing through gaps)
    return float(np.median(final_points)), final_points.size


def get_alignment_at_angle(points: np.ndarray, angle_deg: float, fov_deg: float = 40.0, ref_dist: float | None = None) -> dict | None:
    """
    Fits a line to points at a specific angle to determine tilt relative to the robot's axis.
    Returns tilt_deg: positive means the wall is tilted CCW (receding on the left).
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

    # Outlier Removal: Filter for points within a distance window.
    # We tightened this from 50% to 15% (0.85 to 1.15) to ignore objects behind the wall.
    target_dist = ref_dist if ref_dist is not None else np.median(xf)
    dist_mask = (xf >= target_dist * 0.85) & (xf <= target_dist * 1.15)
    xf = xf[dist_mask]
    yf = yf[dist_mask]

    if xf.size < 8:
        return None

    # Linear Regression: x = m*y + c
    try:
        m, c = np.polyfit(yf, xf, 1)
    except:
        return None

    predicted_x = m * yf + c
    ss_res = np.sum((xf - predicted_x)**2)
    ss_tot = np.sum((xf - np.mean(xf))**2)
    
    # Robustness Check: If the wall is perfectly aligned, variance (ss_tot) approaches zero.
    dist_std = np.std(xf)
    if dist_std < 5.0:
        r2 = 1.0
    else:
        r2 = 1 - (ss_res / ss_tot) if ss_tot > 0.0001 else 0.0

    # m = dx/dy. The angle is atan(m).
    # If m > 0, wall recedes on left (+y), so we must turn left (+) to align.
    tilt_deg = math.degrees(math.atan(m))

    return {
        'tilt_deg': float(tilt_deg),
        'points': int(xf.size),
        'error': float(np.sqrt(ss_res / xf.size)), # RMSE in mm
        'r2': float(r2),
        'm': float(m),
        'b': float(c),
        'xf': xf,
        'yf': yf
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

def getWallAlignment(robot: Robot, input_angle: float) -> dict | None:
    """
    Finds the nearest wall in a specific direction.
    input_angle: 90=Front, 180=Left, 0=Right (Unit Circle convention).
    Returns a dict with heading (standardized), distance, and fit quality metrics.
    """
    robot_angle = input_angle - 90.0
    points_list = robot.get_obstacles()
    if not points_list: return None
    points = np.asarray(points_list)

    # 1. Distance Check (seeded distance) - Consistent FOV
    ref_dist, _ = get_distance_at_angle(points, robot_angle, fov_deg=30.0)
    
    # 2. Alignment Check (using the reference distance)
    res = get_alignment_at_angle(points, robot_angle, fov_deg=30.0, ref_dist=ref_dist)
    if not res: return None

    # target_nom: For Right(0) and Left(180), nominal is 90. For Front(90), nominal is 0.
    target_nom = 90.0 if input_angle in [0.0, 180.0, 360.0] else 0.0
    heading_final = target_nom + res['tilt_deg']

    return {
        'heading': float(heading_final),
        'dist': float(ref_dist if ref_dist else np.median(res['xf'])),
        'points_count': res['points'],
        'r2': res['r2'],
        'rmse': res['error'],
        'm': res['m'],
        'b': res['b'],
        'xf': res['xf'],
        'yf': res['yf'],
        'robot_angle': float(robot_angle)
    }

# ---------------------------------------------------------------------------
# High-Level Sequence Helpers
# ---------------------------------------------------------------------------

def robot_align_to_wall(robot: Robot, poll_angle: float, target_heading: float, timeout_s: float = 15.0) -> bool:
    """
    Blocking helper that performs smooth P-control alignment to a wall.
    Returns True if success, False if aborted or wall lost.
    """
    start_time = time.monotonic()
    align_max_speed = 10.0 # deg/s (increased for better response)
    align_tolerance = 0.5 # Tightened for "perfect" straightness
    align_r2_threshold = 0.8 # Very strict for high confidence
    align_fail_count = 0
    align_fail_limit = 20 # Be patient with the sensor
    last_known_heading = 999.0
    
    Kp_align = 1.5 # P-gain for continuous alignment
    
    print(f"[ALIGN] Target: {target_heading}° | Sector: {poll_angle}°")

    while (time.monotonic() - start_time) < timeout_s:
        # Global Cancel Check
        if robot.was_button_pressed(Button.BTN_10):
            robot.stop()
            print("[ALIGN] Aborted via BTN_10.")
            return False

        # 1. Fetch LIDAR
        res = getWallAlignment(robot, poll_angle)
        
        if res and res['r2'] > align_r2_threshold:
            align_fail_count = 0
            heading = res['heading']
            last_known_heading = heading
            error = target_heading - heading
            
            # 2. Check for completion
            if abs(error) <= align_tolerance:
                robot.stop()
                print(f"[ALIGN] Success! Heading: {heading:+.2f}° (Err: {error:+.1f}°)")
                return True
            
            # 3. Continuous P-Control (Smoother than turn_by)
            angular_vel = error * Kp_align
            angular_vel = max(-align_max_speed, min(align_max_speed, angular_vel))
            
            robot.set_velocity(0.0, angular_vel)
            
            if np.random.rand() < 0.10:
                print(f"  [ALIGN] Head:{heading:+.1f}° | Err:{error:+.1f}° | R2:{res['r2']:.2f}")
        else:
            # 4. Handle lost wall / finishing
            robot.stop() # Stop if we lose the wall to avoid drifting
            align_fail_count += 1
            
            if align_fail_count % 5 == 0:
                points = robot.get_obstacles()
                n_pts = len(points) if points else 0
                if n_pts == 0:
                    print(f"  [ALIGN] No LIDAR points received from sensor.")
                elif not res:
                    print(f"  [ALIGN] No wall found in {poll_angle}° sector.")
                else:
                    print(f"  [ALIGN] Wall fit too noisy: R2={res['r2']:.2f} (need >{align_r2_threshold})")

            if align_fail_count >= align_fail_limit:
                error_last = target_heading - (last_known_heading if last_known_heading != 999.0 else 0.0)
                # Tightened: Only return True if we are actually within the real tolerance
                if last_known_heading != 999.0 and abs(error_last) <= align_tolerance:
                    print(f"[ALIGN] Complete (Last error: {error_last:+.1f}°).")
                    return True
                else:
                    print(f"[ALIGN] Abort: Wall lost or fit too noisy.")
                    return False
                
                if np.random.rand() < 0.05:
                    print("[ALIGN] Searching for wall...")
        
        time.sleep(0.05) # 20Hz loop for smooth control

    robot.stop()
    print("[ALIGN] Abort: Timeout reached.")
    return False

def robot_align_front(robot: Robot) -> bool:
    return robot_align_to_wall(robot, 90.0, 0.0)

def robot_align_left(robot: Robot) -> bool:
    return robot_align_to_wall(robot, 180.0, 90.0)

def robot_align_right(robot: Robot) -> bool:
    return robot_align_to_wall(robot, 0.0, 90.0)

def robot_approach_wall(robot: Robot, target_dist_mm: float = 25.0, tolerance_mm: float = 2.0, timeout_s: float = 15.0) -> bool:
    """
    Blocking helper that performs smooth P-control to approach a wall.
    Returns True if success, False if aborted or wall lost.
    """
    start_time = time.monotonic()
    Kp_dist = 1.2
    max_linear = 60.0
    
    print(f"[APPROACH] Target: {target_dist_mm}mm")
    lost_wall_count = 0
    lost_wall_limit = 20  # ~1 second of no detection

    while (time.monotonic() - start_time) < timeout_s:
        # Global Cancel Check
        if robot.was_button_pressed(Button.BTN_10):
            robot.stop()
            print("[APPROACH] Aborted via BTN_10.")
            return False

        points = np.asarray(robot.get_obstacles())
        current_dist, count = get_front_distance(points, fov_deg=10.0)

        if current_dist is not None and count > 0:
            lost_wall_count = 0
            error_mm = current_dist - target_dist_mm
            
            if abs(error_mm) <= tolerance_mm:
                robot.stop()
                print(f"[APPROACH] Success! Distance: {current_dist:.1f}mm")
                return True
            
            # P-Control for distance
            linear_cmd = Kp_dist * error_mm
            linear_cmd = max(-max_linear, min(max_linear, linear_cmd))
            
            robot.set_velocity(linear_cmd, 0.0)
            
            if np.random.rand() < 0.05:
                print(f"  [APPROACH] Dist:{current_dist:.1f}mm | Err:{error_mm:.1f}mm")
        else:
            robot.stop()
            lost_wall_count += 1
            if lost_wall_count >= lost_wall_limit:
                print("[APPROACH] Abort: Wall not detected.")
                return False
            
            if np.random.rand() < 0.05:
                print("[APPROACH] Searching for wall...")
        
        time.sleep(0.05)

    robot.stop()
    print("[APPROACH] Abort: Timeout reached.")
    return False

def robot_approach_front(robot: Robot, target_dist_mm: float = 25.0) -> bool:
    return robot_approach_wall(robot, target_dist_mm=target_dist_mm)

def print_all_sides_stats(robot: Robot, step_name: str) -> None:
    """Helper to print distance and alignment for Front, Left, and Right sides with diagnostics."""
    points = np.asarray(robot.get_obstacles())
    if points.size == 0:
        print(f"[{step_name}] No LiDAR points available.")
        return

    # Get distances
    f_dist, f_count = get_front_distance(points)
    l_dist, l_count = get_left_distance(points)
    r_dist, r_count = get_right_distance(points)
    
    # Get alignments
    f_res = get_wall_alignment(points, ref_dist=f_dist)
    l_res = get_left_alignment(points, ref_dist=l_dist)
    r_res = get_right_alignment(points, ref_dist=r_dist)

    def format_align(res):
        if not res: return "None"
        return f"{res['tilt_deg']:+5.1f}° [RMSE:{res['error']:4.1f}mm, R2:{res['r2']:.2f}]"

    print(f"\n--- LiDAR Report: {step_name} ---")
    print(f"FRONT: {f_dist if f_dist else 0.0:6.1f} mm ({f_count:3} pts) | {format_align(f_res)}")
    print(f"LEFT : {l_dist if l_dist else 0.0:6.1f} mm ({l_count:3} pts) | {format_align(l_res)}")
    print(f"RIGHT: {r_dist if r_dist else 0.0:6.1f} mm ({r_count:3} pts) | {format_align(r_res)}")
    print("------------------------------------------")

def save_lidar_plot(robot: Robot, filename: str = "lidar_temp.png", align_results: list[dict] | None = None) -> None:
    """Saves a plot of the current LiDAR scan, optionally highlighting alignment fits."""
    points_list = robot.get_obstacles()
    if not points_list:
        print("[LIDAR] No points to plot.")
        return
    
    points = np.asarray(points_list)
    plot_x = -points[:, 1]
    plot_y = points[:, 0]

    plt.figure(figsize=(10, 10))
    plt.scatter(plot_x, plot_y, s=2, c='gray', alpha=0.5, label='All Points')
    
    if align_results:
        colors = ['red', 'green', 'blue', 'orange', 'purple']
        for i, res in enumerate(align_results):
            color = colors[i % len(colors)]
            xf = res['xf']
            yf = res['yf']
            angle_rad = math.radians(res['robot_angle'])
            c, s = math.cos(angle_rad), math.sin(angle_rad)
            robot_x = xf * c - yf * s
            robot_y = xf * s + yf * c
            px = -robot_y
            py = robot_x
            plt.scatter(px, py, s=15, c=color, label=f'Fit {i+1} Points')
            yf_range = np.array([np.min(yf), np.max(yf)])
            xf_fit = res['m'] * yf_range + res['b']
            line_x = xf_fit * c - yf_range * s
            line_y = xf_fit * s + yf_range * c
            plt.plot(-line_y, line_x, color=color, lw=2, label=f'Fit {i+1} Line')

    plt.axhline(0, color='black', lw=1, alpha=0.3)
    plt.axvline(0, color='black', lw=1, alpha=0.3)
    robot_body = plt.Circle((0, 0), 100, color='blue', fill=False, label='Robot (100mm)')
    plt.gca().add_patch(robot_body)
    plt.title(f"LiDAR Scan (Front is UP) - Alignment Visualization")
    plt.xlabel("Left <--- (mm) ---> Right")
    plt.ylabel("Back <--- (mm) ---> Front")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(loc='upper right', fontsize='small')
    
    import os
    save_path = os.path.join("/runtime_output", filename)
    try:
        plt.savefig(save_path)
        print(f"[LIDAR] Debug plot saved to {save_path}")
    except Exception as e:
        print(f"[LIDAR] Failed to save plot: {e}")
    finally:
        plt.close()
