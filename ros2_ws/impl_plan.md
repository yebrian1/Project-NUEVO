# Plan: Fix EXEC_MAZE_DRIVING Logic and NameError

## Goal
Resolve the `NameError` in `main.py` and ensure robust waypoint following that doesn't stop prematurely or crash on empty paths.

## Proposed Changes

### `ros2_ws/src/robot/robot/main.py`

Replace the existing `EXEC_MAZE_DRIVING` block with refined logic:

1.  **Advance Waypoints:** Use `robot._advance_remaining_path` with a 100mm radius.
2.  **Safety Check:** Return to `IDLE` if the path becomes empty.
3.  **Compute Velocity:** Use `planner1.compute_velocity` with the full `remaining_path` list (internally handles lookahead).
4.  **Logging:** Define `target_x` and `target_y` from the **final** waypoint for the status print.
5.  **Arrival Check:** Use `CurrentTargetReached` against the **final** waypoint to trigger mission completion.

```python
        elif state == "EXEC_MAZE_DRIVING":
            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            # 1. Update waypoints (advance radius should be smaller than lookahead)
            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y, advance_radius_mm=100.0
            )
            
            # 2. Safety: Ensure path isn't finished already
            if not remaining_path:
                print("[DONE] Maze complete.")
                robot.stop()
                state = "IDLE"
                continue

            # 3. Compute and set velocities
            linear, angular = planner1.compute_velocity(
                pose=(current_x, current_y, current_theta_rad), 
                waypoints=remaining_path, 
                max_linear=100.0
            )
            robot.set_velocity(linear, math.degrees(angular))

            # 4. Check for final destination (the last point in the path)
            target_x, target_y = remaining_path[-1]
            # print(f"[MAZE] pos=({current_x:.0f},{current_y:.0f}) target=({target_x:.0f},{target_y:.0f}) theta={current_theta_deg:.1f}")
            
            if planner1.CurrentTargetReached(target_x, target_y, current_x, current_y):
                print("[ACTION] Destination reached! Stopping motors.")
                robot.stop()
                print("[FSM] Returned to IDLE")
                state = "IDLE"
```

## Verification
1.  Run the robot and start the maze.
2.  Observe the console for the `[POSE]` and `[MAZE]` debug prints.
3.  Confirm waypoints advance as the robot moves.
4.  Confirm the robot completes the entire maze before stopping.
