file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'
with open(file_path, 'r') as f:
    lines = f.readlines()

new_lines = []
seen_imports = set()
in_lidar_helpers = False

for line in lines:
    stripped = line.strip()
    if 'from robot.lidar_helpers import (' in line:
        in_lidar_helpers = True
        new_lines.append(line)
        continue
    
    if in_lidar_helpers:
        if ')' in line:
            in_lidar_helpers = False
            new_lines.append(line)
            continue
        
        # Inside the tuple, clean up duplicates
        func_name = stripped.replace(',', '').strip()
        if func_name and func_name not in seen_imports:
            seen_imports.add(func_name)
            new_lines.append(line)
    else:
        new_lines.append(line)

with open(file_path, 'w') as f:
    f.writelines(new_lines)
