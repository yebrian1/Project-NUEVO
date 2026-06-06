import os

file_path = '/home/raspberrypig4/Project-NUEVO/ros2_ws/src/robot/robot/main.py'

with open(file_path, 'r') as f:
    lines = f.readlines()

start_idx = -1
for i, line in enumerate(lines):
    if 'elif state == "POST_NAV_SCAN":' in line:
        start_idx = i
        break

end_idx = -1
for i, line in enumerate(lines):
    if 'if dist_traveled >= 2800.0:' in line:
        # Go down to the end of that elif block
        for j in range(i, len(lines)):
            if 'state = "IDLE"' in lines[j] and 'show_idle_leds(robot)' in lines[j-1]:
                end_idx = j + 1
                break
        if end_idx != -1:
            break

if start_idx != -1 and end_idx != -1:
    new_logic = """        elif state == "POST_NAV_SCAN":
            print("[MASTER] Initiating face scan (15s timeout)...")
            gender = classifier.get_gender(wait_for_face=15.0)
            if gender:
                print(f"[RESULT] CUSTOMER IDENTIFIED: {gender}")
                global LAPF_GOAL_MM
                if "girl" in gender.lower():
                    LAPF_GOAL_MM = (800.0, 400.0)
                else: # boy
                    LAPF_GOAL_MM = (800.0, 300.0)
                
                print(f"[MASTER] Gender recognized. Final goal set to {LAPF_GOAL_MM}. Starting LAPF.")
                state = "FINAL_LAPF_START"
                drive_handle = None
            else:
                print("[RESULT] No face detected during scan.")
                print("[MASTER] MISSION COMPLETE. Returning to IDLE.")
                show_idle_leds(robot)
                state = "IDLE"

        elif state == "FINAL_LAPF_START":
            # Start of the final mission leg using obstacle avoidance
            show_running_leds(robot)
            drive_handle = start_lapf_goal(robot)
            last_status_print_at = now
            print(f"[FSM] FINAL LAPF — Moving toward goal {LAPF_GOAL_MM}")
            state = "FINAL_LAPF_MOVING"

        elif state == "FINAL_LAPF_MOVING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_lapf_status(robot)
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] Final mission goal reached!")
                drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                state = "IDLE"

"""
    # Replace the block
    lines[start_idx:end_idx] = [new_logic]
    
    with open(file_path, 'w') as f:
        f.writelines(lines)
    print("Successfully implemented gender branching and final LAPF.")
else:
    print(f"Failed to find indices: {start_idx}, {end_idx}")
