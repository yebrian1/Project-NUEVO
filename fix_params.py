import os

files = [
    '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py',
    '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/obstacle_scan_delivery2.py'
]

for file_path in files:
    with open(file_path, 'r') as f:
        content = f.read()
    
    content = content.replace('WHEEL_DIAMETER = 80.0', 'WHEEL_DIAMETER = 74.0')
    content = content.replace('TARGET_SPEED_MM_S = 150.0', 'TARGET_SPEED_MM_S = 200.0')
    content = content.replace('REPULSION_GAIN = 450.0', 'REPULSION_GAIN = 550.0')
    content = content.replace('FORCE_EMA_ALPHA = 0.25', 'FORCE_EMA_ALPHA = 0.35')
    
    with open(file_path, 'w') as f:
        f.write(content)
    print(f"Updated {file_path}")
