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

from robot.lidar_helpers import (
    getWallAlignment,
    save_lidar_plot,
)

# ---------------------------------------------------------------------------
# Robot Build & Drive Configuration
# ---------------------------------------------------------------------------
TAG_ID = 21 # Updated to user's GPS ArUco tag
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

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
    robot.enable_gps_tangent_heading(alpha=0.15, min_displacement_mm=200.0)
    robot.set_position_fusion_alpha(0.10)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_motor(LEFT_WHEEL_MOTOR)
    robot.enable_motor(RIGHT_WHEEL_MOTOR)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

# --- LED Helpers ---
def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # Alignment state variables
    align_max_speed = 5 # deg/s
    align_min_speed = 1 # deg/s (to overcome friction)
    align_kp = 0.5 # P-gain: speed = error * kp
    align_tolerance = 2.0 # dega
    last_w_cmd = 0.0
    last_sent_w = -999.0 # Forces initial update
    last_cmd_time = 0.0
    cmd_min_interval = 0.05 # Max 20Hz motor commands to keep UART healthy

    while True:
        loop_start = time.monotonic()
        
        # --- GLOBAL CANCEL BUTTON ---
        if state != "IDLE" and robot.was_button_pressed(Button.BTN_10):
            robot.stop() 
            show_idle_leds(robot)
            print("[FSM] Reset to IDLE.")
            state = "IDLE"

        # --- FSM STATES ---
        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            
            # --- ALIGNMENT TRIGGER ---
            if robot.was_button_pressed(Button.BTN_1):
                print("[ACTION] Starting smooth P-control FRONT wall alignment. Goal: 0.0°")
                show_running_leds(robot)
                last_w_cmd = 0.0
                last_sent_w = -999.0
                state = "ALIGN_P_SMOOTH"

            elif robot.was_button_pressed(Button.BTN_9):
                state = "CMD_MAZE_START"

        elif state == "ALIGN_P_SMOOTH":
            # 1. Fetch latest LIDAR result
            res = getWallAlignment(robot, 90.0) # Check front
            
            if res and res['r2'] > 0.4:
                heading = res['heading']
                
                # 2. Check for completion
                if abs(heading) <= align_tolerance:
                    if robot.is_moving():
                        robot.cancel_motion()
                    robot.stop()
                    print(f"[ALIGN] Complete! Final Heading: {heading:+.2f}°")
                    show_idle_leds(robot)
                    state = "IDLE"
                    continue
                
                # 3. Non-Blocking Turn
                # If we are not already moving, start a turn toward the detected wall angle.
                # This uses the built-in P-controller in the NavigationMixin background thread.
                if not robot.is_moving():
                    robot.turn_by(heading, blocking=False, 
                                  max_angular_speed=align_max_speed,
                                  tolerance_deg=align_tolerance)
                
                if np.random.rand() < 0.1: # 10% logging
                    print(f"  [ALIGN] Head:{heading:+.1f}° | R2:{res['r2']:.2f} | Moving:{robot.is_moving()}")
            else:
                # If LIDAR frame is bad/missing, we just wait.
                # If we were turning, the background thread continues until it hits its target.
                if not robot.is_moving() and np.random.rand() < 0.05:
                    print("[WARN] Searching for front wall...")

        elif state == "CMD_MAZE_START":
            state = "IDLE"

        # Loop Timing Control
        loop_time = time.monotonic() - loop_start
        if loop_time > period:
            # We are lagging - don't sleep, just move to next tick
            # print(f"[LAG] Loop took {loop_time*1000:.1f}ms (target {period*1000:.1f}ms)")
            next_tick = time.monotonic()
        else:
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
