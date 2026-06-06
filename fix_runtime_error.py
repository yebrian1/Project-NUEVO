import os

file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'

with open(file_path, 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    # Ensure any background motion is dead before starting blocking alignment
    if 'elif state == "POST_NAV_ALIGN":' in line:
        # Check if the next line already has the fix
        if 'if drive_handle is not None:' not in lines[i+1]:
            fix = [
                '        elif state == "POST_NAV_ALIGN":\n',
                '            # Ensure any background motion is dead before starting blocking alignment\n',
                '            if drive_handle is not None:\n',
                '                drive_handle.cancel()\n',
                '                drive_handle = None\n',
                '            robot.stop()\n'
            ]
            # Replace the single elif line with the fix block
            lines[i:i+1] = fix
            break

with open(file_path, 'w') as f:
    f.writelines(lines)
print("Applied fix to POST_NAV_ALIGN")
