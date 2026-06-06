import os

file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'

# Read the file
with open(file_path, 'r') as f:
    lines = f.readlines()

# Fix the import
for i, line in enumerate(lines):
    if 'robot_align_to_wall,' in line and 'robot_approach_wall' not in line:
        lines[i] = line.replace('robot_align_to_wall,', 'robot_align_to_wall,\n    robot_approach_wall,')
        break

# Find the start of POST_NAV_ALIGN
start_idx = -1
for i, line in enumerate(lines):
    if 'elif state == "POST_NAV_ALIGN":' in line:
        start_idx = i
        break

# Find the end of run() or POST_NAV_SCAN
end_idx = -1
for i, line in enumerate(lines):
    if 'next_tick += period' in line:
        end_idx = i
        break

if start_idx != -1 and end_idx != -1:
    new_logic = """        elif state == "POST_NAV_ALIGN":
            # Ensure any background motion is dead before starting blocking alignment
            if drive_handle is not None:
                robot.stop()
                drive_handle = None

            print("[MASTER] Aligning with front wall...")
            if robot_align_front(robot):
                print("[MASTER] Aligned. Approaching to 75mm via LiDAR.")
                state = "POST_NAV_APPROACH_LIDAR"
            else:
                state = "IDLE"

        elif state == "POST_NAV_APPROACH_LIDAR":
            # Precision approach to exactly 75mm from wall
            if robot_approach_wall(robot, 75.0, tolerance_mm=2.0):
                print("[MASTER] Positioned 75mm from wall. Starting scan.")
                state = "POST_NAV_SCAN"
            else:
                print("[WARN] Approach failed. Retrying alignment.")
                state = "POST_NAV_ALIGN"

        elif state == "POST_NAV_SCAN":
            print("[MASTER] Initiating face scan (15s timeout)...")
            gender = classifier.get_gender(wait_for_face=15.0)
            if gender:
                print(f"[RESULT] CUSTOMER IDENTIFIED: {gender}")
                print("[MASTER] Face recognized. Starting final sequence.")
                state = "FINAL_MOVE_FWD_75"
                drive_handle = None
            else:
                print("[RESULT] No face detected during scan.")
                print("[MASTER] MISSION COMPLETE. Returning to IDLE.")
                show_idle_leds(robot)
                state = "IDLE"

        elif state == "FINAL_MOVE_FWD_75":
            if drive_handle is None:
                print("[ACTION] Driving forward 75mm")
                drive_handle = robot.move_forward(75.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "FINAL_TURN_RIGHT_90"

        elif state == "FINAL_TURN_RIGHT_90":
            if drive_handle is None:
                print("[ACTION] Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False, max_angular_speed=math.radians(TURN_VELOCITY_DEG_S), tolerance_deg=TURN_TOLERANCE_DEG)
            elif drive_handle.is_finished():
                robot.stop()
                drive_handle = None
                state = "FINAL_ALIGN_LEFT_WALL"

        elif state == "FINAL_ALIGN_LEFT_WALL":
            print("[ACTION] Aligning with left wall")
            # Use robot_align_to_wall with 90° (Left) and 0° (parallel target)
            if robot_align_to_wall(robot, 90.0, 0.0):
                print("[FSM] Left alignment successful.")
            else:
                print("[WARN] Left alignment failed. Proceeding anyway.")
            
            start_pose = robot.get_odometry_pose()
            state = "FINAL_WALL_FOLLOW_2800"

        elif state == "FINAL_WALL_FOLLOW_2800":
            curr_pose = robot.get_odometry_pose()
            dist_traveled = math.hypot(curr_pose[0] - start_pose[0], curr_pose[1] - start_pose[1])
            
            points = np.asarray(robot.get_obstacles())
            dist, count = get_left_distance(points)
            
            if dist is not None and count > 0:
                error_mm = dist - 50.0 # Maintain 50mm from left wall
                angular_cmd = error_mm * FOLLOW_KP
                angular_cmd = max(-20.0, min(20.0, angular_cmd))
                robot.set_velocity(VELOCITY_MM_S, angular_cmd)
                
                if time.monotonic() % 1.0 < 0.1:
                    print(f"[FOLLOW] Dist:{dist:.1f}mm | Traveled:{dist_traveled:.0f}mm")
            else:
                # If wall lost, drive straight
                robot.set_velocity(VELOCITY_MM_S, 0.0)

            if dist_traveled >= 2800.0:
                robot.stop()
                drive_handle = None
                print(f"[FSM] Final wall-following complete ({dist_traveled:.0f}mm). MISSION COMPLETE.")
                show_idle_leds(robot)
                state = "IDLE"
"""
    # Build the final content
    final_lines = lines[:start_idx] + [new_logic] + lines[end_idx:]
    with open(file_path, 'w') as f:
        f.writelines(final_lines)
    print("Success")
else:
    print(f"Failed to find indices: {start_idx}, {end_idx}")
