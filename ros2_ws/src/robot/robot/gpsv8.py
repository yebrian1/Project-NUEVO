from __future__ import annotations
import time
import math
import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    Stepper,
    ServoChannel,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.path_planner2 import PurePursuitPlanner, generate_maze_waypoints
from robot.util import densify_polyline
import matplotlib.pyplot as plt
from robot.lidar_helpers import (
    get_front_distance,
    get_wall_alignment,
    save_lidar_plot,
    robot_align_to_wall,
)

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 # Updated to user's GPS ArUco tag
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 80.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Actuator Configuration (Arm & Gripper)
# ---------------------------------------------------------------------------
GRIPPER_CHANNEL   = ServoChannel.CH_1
GRIPPER_OPEN_DEG  = 150.0   
GRIPPER_CLOSE_DEG = 50.0  

ARM_STEPPER        = Stepper.STEPPER_1
arm_up_steps       = -2000
arm_down_steps     = 1600
ARM_MAX_VELOCITY   = 400    
ARM_ACCELERATION   = 200    
ARM_HOME_VELOCITY  = 300    

# ---------------------------------------------------------------------------
# Pure Pursuit Configuration
# ---------------------------------------------------------------------------
VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 250.0  # Increased for smoother turns
TOLERANCE_MM       = 80.0   # Relaxed tolerance for better arrival
ADVANCE_RADIUS_MM  = 250.0  # Allows skipping overshot points
MAX_ANGULAR_RAD_S  = 1.0    # Stability improvement

STATUS_PRINT_INTERVAL_S = 0.5

# Simplified path to avoid tethering issues at corners
PATH_CONTROL_POINTS = [
    (0, 1300),
    (0, 3750),  # Straight
    # (520, 3650),  # Midpoint for smoother turn

    # (730, 3750),  # Corner 2: Turn Right
    # (765, 3650),
    # (900, 3600),  # Corner 2: Turn Right
    # (950,3500),
    # (1050, 3450),
    # (1150,3418),
    # (1150,3400),

    # (1150, 3300),
    # (1150, 3225),
    # (1150, 3250),
    # (1150, 3275),
    # (1150,3200),
    # (1150, 3175),
    # (1150, 3150),
    # (1150, 3125),
    # (1150, 3100),
    # (1150,3000),
    # (1150, 2500),
    # (1200,2000),
    # (1200, 1800),  # Corner 3: Turn Right
    # (1200, 800),
    # (1200, 1000),  # Corner 4: Turn Left
    # (1340, 1500),  # Final Goal
]

PATH_CONTROL_POINTS_2 = [(1150,3400),
    (800, 3300),
    (800, 3150),
    (800,2000),
    (850,1500),
    (850, 1000), 
    (850, 600), # Corner 3: Turn Right
    
]

RUNBACK = False

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
ENABLE_GPS = True
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.20
LED_BRIGHTNESS = 255

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision() 
    robot.enable_gps()
    robot.enable_lidar()
    robot.set_lidar_filter(range_min_mm=0.0)

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID)
    robot.enable_gps_tangent_heading(alpha=0.01, min_displacement_mm=200.0)
    robot.set_position_fusion_alpha(0.01)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")









# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    global RUNBACK
    configure_robot(robot)
    state = "INIT"
    seq_step = 0
    state_timer_end = 0.0
    drive_handle = None
    last_status_print_at = 0.0
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()








    while True:
        now = time.monotonic()
        
        # --- LIDAR DEBUG BUTTONS ---
        if robot.was_button_pressed(Button.BTN_1):
            points = np.asarray(robot.get_obstacles())
            avg_dist, count = get_front_distance(points, fov_deg=5.0)
            if avg_dist is not None:
                print(f"[DEBUG DIST] Avg: {avg_dist:.2f} mm | Points: {count}")
            else:
                print("[DEBUG DIST] No points in 5 deg cone.")

        if robot.was_button_pressed(Button.BTN_2):
            points = np.asarray(robot.get_obstacles())

            ref_dist, _ = get_front_distance(points, fov_deg=5.0)
            result = get_wall_alignment(points, fov_deg=15.0, ref_dist=ref_dist)

            if result:
                tilt = result['tilt_deg']
                print(f"[DEBUG ALIGN] Tilt: {tilt:+.2f} deg")

            else:
                print("[DEBUG ALIGN] Not enough points for tilt.")

        if robot.was_button_pressed(Button.BTN_3):
            save_lidar_plot(robot)

        if state not in ["INIT", "IDLE"] and robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
            robot.stop() 
            show_idle_leds(robot)
            print("[FSM] IDLE — path cancelled/stopped")
            state = "IDLE"

        if state == "INIT":

            start_robot(robot)
            show_idle_leds(robot)
            robot.step_set_config(ARM_STEPPER, max_velocity=ARM_MAX_VELOCITY, acceleration=ARM_ACCELERATION)
            print("[FSM] IDLE — press BTN_9 to start Maze")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.was_button_pressed(Button.BTN_9):
                state = "CMD_MAZE_START"



        elif state == "CMD_MAZE_START":
            show_running_leds(robot)

            # Wait for firmware bridge
            print("[INIT] Waiting for firmware bridge...")
            bridge_wait = time.monotonic()
            while time.monotonic() - bridge_wait < 5.0:
                if robot.get_state() == FirmwareState.RUNNING:

                    break
                time.sleep(0.2)
            










            print("[GPS] Checking for GPS lock (Tag 21)...")
            gps_wait = time.monotonic()
            gps_locked = False
            last_gps_print = 0.0
            
            # Increased timeout to 30 seconds to allow for camera stabilization/detection
            while time.monotonic() - gps_wait < 30.0:
                if robot.is_gps_active():
                    # Double check if coordinates are non-zero/valid
                    gps_x, gps_y = robot._gps_x_mm, robot._gps_y_mm
                    if abs(gps_x) > 0.1 or abs(gps_y) > 0.1:
                        print(f"[GPS] Lock acquired! x={gps_x:.1f}mm y={gps_y:.1f}mm")
                        gps_locked = True
                        break
                
                if time.monotonic() - last_gps_print >= 2.0:
                    print(f"[GPS] Waiting... ({time.monotonic() - gps_wait:.0f}s elapsed)")
                    last_gps_print = time.monotonic()
                time.sleep(0.1)

            if not gps_locked:
                print("[WARN] GPS lock failed after 30s. Proceeding with Odometry only.")
            else:
                print("[GPS] Synchronizing Odometry to GPS (Alpha Snap)...")
                robot.set_position_fusion_alpha(1.0)
                time.sleep(0.5) # Wait for fusion loop to process
                robot.set_position_fusion_alpha(0.01) # Restore to original tracking alpha
                print("[GPS] Synchronization complete. Pose is now aligned.")
                time.sleep(0.2)

            # Use built-in pure-pursuit with simplified path and robust parameters
            drive_handle = robot.purepursuit_follow_path(
                waypoints=PATH_CONTROL_POINTS,
                velocity=VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=TOLERANCE_MM,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S,
                blocking=False,
            )

            print(f"[FSM] MOVING — Using built-in Pure Pursuit")
            last_status_print_at = now    



            state = "EXEC_MAZE_DRIVING"

        elif state == "EXEC_MAZE_DRIVING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot)
                
                # Simple target waypoint estimation for logging
                curr_x, curr_y, _ = robot.get_pose()
                target_wp = PATH_CONTROL_POINTS[-1]
                for wp in PATH_CONTROL_POINTS:
                    if math.hypot(wp[0] - curr_x, wp[1] - curr_y) > LOOKAHEAD_MM:
                        target_wp = wp
                        break
                print(f"  target_wp=({target_wp[0]:.0f}, {target_wp[1]:.0f})")
                
                last_status_print_at = now
            
            if drive_handle is not None and drive_handle.is_finished():
                print("[FSM] DONE — path complete")
                robot.stop()
                show_idle_leds(robot)
                
                # Align with the wall before starting the turn sequence
                print("[FSM] Aligning with wall...")
                # 90.0 is Front in Unit Circle (face the wall directly)
                robot_align_to_wall(robot, 90.0, 0.0)

                seq_step = 0
                state = "EXEC_TURN_SEQUENCE"




        elif state == "EXEC_TURN_SEQUENCE":
            # Performs a U-turn sequence: Turn 90° Right, Drive 825mm, Turn 90° Right
            if seq_step == 0:
                print("[SEQ] Step 1/3: Turning Right 90°")
                drive_handle = robot.turn_by(-90.0, blocking=False)
                seq_step = 1

            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 2/3: Driving Straight 600 mm")
                    drive_handle = robot.move_forward(670.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                    seq_step = 2

            elif seq_step == 2:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[SEQ] Step 3/3: Turning Right 90°")
                    drive_handle = robot.turn_by(-90.0, blocking=False)
                    seq_step = 3

            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    robot.stop()
                    show_idle_leds(robot)
                    print("[SEQ] Sequence complete! Transitioning to EXEC_RAMP_SEQUENCE.")
                    RUNBACK = True
                    seq_step = 0
                    state = "EXEC_RAMP_SEQUENCE"

        elif state == "EXEC_RAMP_SEQUENCE":
            # Manually move forward 2715mm without GPS
            if seq_step == 0:
                print("[RAMP] Step 1/4: Moving 75 mm forward to get closer to ramp")
                drive_handle = robot.move_forward(150.0, velocity=VELOCITY_MM_S, tolerance=10.0, blocking=False)
                seq_step = 1

            elif seq_step == 1:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[RAMP] Step 2/4: Aligning with ramp (front)...")
                    # Using 90.0 to face the ramp/wall directly
                    robot_align_to_wall(robot, 90.0, 0.0)
                    seq_step = 2

            elif seq_step == 2:
                print("[RAMP] Step 3/4: Driving straight 2715 mm (Manual)")
                drive_handle = robot.move_forward(2715.0, velocity=VELOCITY_MM_S, tolerance=50.0, blocking=False)
                seq_step = 3

            elif seq_step == 3:
                if drive_handle is not None and drive_handle.is_finished():
                    print("[RAMP] Step 4/4: Aligning with wall before turn (front)...")
                    robot_align_to_wall(robot, 90.0, 0.0)
                    robot.stop()
                    show_idle_leds(robot)
                    seq_step = 0
                    state = "EXEC_TURN_SEQUENCE_2"

        elif state == "EXEC_TURN_SEQUENCE_2":
                    # Performs a U-turn sequence: Turn 90° Left, Drive 825mm, Turn 90° Left
                    if seq_step == 0:
                        print("[SEQ] Step 1/3: Turning Left 90°")
                        drive_handle = robot.turn_by(90.0, blocking=False)
                        seq_step = 1

                    elif seq_step == 1:
                        if drive_handle is not None and drive_handle.is_finished():
                            print("[SEQ] Step 2/3: Driving Straight 700mm")
                            drive_handle = robot.move_forward(700.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                            seq_step = 2

                    elif seq_step == 2:
                        if drive_handle is not None and drive_handle.is_finished():
                            print("[SEQ] Step 3/3: Turning Left 90°")
                            drive_handle = robot.turn_by(90.0, blocking=False)
                            seq_step = 3

                    elif seq_step == 3:
                        if drive_handle is not None and drive_handle.is_finished():
                            print("[SEQ] Step 2/3: Driving Straight 200mm")
                            drive_handle = robot.move_forward(200.0, velocity=VELOCITY_MM_S, tolerance=20.0, blocking=False)
                            seq_step = 4

                    elif seq_step == 4:
                        if drive_handle is not None and drive_handle.is_finished():
                            robot.stop()
                            show_idle_leds(robot)
                            print("[SEQ] Sequence complete! Transitioning to IDLE.")
                            RUNBACK = True
                            state = "IDLE"


        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

if __name__ == "__main__":
    from robot.robot_node import RobotNode
    import rclpy
    rclpy.init()
    node = RobotNode()
    robot = Robot(node)
    try:
        run(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        rclpy.shutdown()