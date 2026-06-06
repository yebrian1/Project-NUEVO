import os

file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'

with open(file_path, 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    # 1. Update POST_NAV_ALIGN transition
    if 'print("[MASTER] Aligned. Turning right 90° before approach.")' in line:
        lines[i] = '                print("[MASTER] Aligned. Moving forward 50mm before turn.")\n'
        lines[i+1] = '                state = "POST_NAV_MOVE_FWD_50"\n'
    
    # 2. Insert POST_NAV_MOVE_FWD_50 before POST_NAV_TURN_RIGHT_90
    if '        elif state == "POST_NAV_TURN_RIGHT_90":' in line:
        new_state = [
            '        elif state == "POST_NAV_MOVE_FWD_50":\n',
            '            if drive_handle is None:\n',
            '                drive_handle = robot.move_forward(50.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)\n',
            '            elif drive_handle.is_finished():\n',
            '                robot.stop()\n',
            '                drive_handle = None\n',
            '                state = "POST_NAV_TURN_RIGHT_90"\n',
            '\n'
        ]
        lines[i:i] = new_state
    
    # 3. Replace LiDAR approach with hardcoded 100mm move
    if '        elif state == "POST_NAV_APPROACH_LIDAR":' in line:
        # Replace the entire block until the next elif
        j = i + 1
        while j < len(lines) and '        elif state == ' not in lines[j]:
            j += 1
        
        replacement = [
            '        elif state == "POST_NAV_APPROACH_LIDAR":\n',
            '            if drive_handle is None:\n',
            '                print("[MASTER] Hardcoded approach: Driving forward 100mm.")\n',
            '                drive_handle = robot.move_forward(100.0, velocity=VELOCITY_MM_S, tolerance=2.0, blocking=False)\n',
            '            elif drive_handle.is_finished():\n',
            '                robot.stop()\n',
            '                drive_handle = None\n',
            '                state = "POST_NAV_SCAN"\n',
            '\n'
        ]
        lines[i:j] = replacement
        break

with open(file_path, 'w') as f:
    f.writelines(lines)
print("Successfully updated precision move sequence.")
