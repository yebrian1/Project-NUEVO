import os

file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'

with open(file_path, 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    # Update transition in POST_NAV_ALIGN
    if 'print("[MASTER] Aligned. Approaching to 75mm via LiDAR.")' in line:
        lines[i] = '                print("[MASTER] Aligned. Turning right 90° before approach.")\n'
        lines[i+1] = '                state = "POST_NAV_TURN_RIGHT_90"\n'
    
    # Insert new state before POST_NAV_APPROACH_LIDAR
    if 'elif state == "POST_NAV_APPROACH_LIDAR":' in line:
        new_state = [
            '        elif state == "POST_NAV_TURN_RIGHT_90":\n',
            '            if drive_handle is None:\n',
            '                drive_handle = robot.turn_by(-90.0, blocking=False)\n',
            '            elif drive_handle.is_finished():\n',
            '                robot.stop()\n',
            '                drive_handle = None\n',
            '                state = "POST_NAV_APPROACH_LIDAR"\n',
            '\n'
        ]
        lines[i:i] = new_state
        break

with open(file_path, 'w') as f:
    f.writelines(lines)
print("Successfully inserted POST_NAV_TURN_RIGHT_90")
