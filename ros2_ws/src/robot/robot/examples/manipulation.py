"""
manipulation.py — blocking pick sequence example
================================================
Demonstrates one simple manipulator sequence using:

1. Servo 1 as a gripper
2. Stepper 1 as a horizontal arm
3. DC Motor 3 in POSITION mode as a vertical lift

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to run one pick sequence:

  1. Raise the lift
  2. Extend the arm
  3. Open the gripper
  4. Lower the lift
  5. Close the gripper
  6. Raise the lift
  7. Retract the arm

This example uses a blocking sequence on purpose, so the code is easy to read.
During the sequence, the FSM stays inside one state until the whole sequence
finishes.

WHAT THIS TEACHES
-----------------
1. `step_home()` for stepper homing at startup
2. `set_servo()` for gripper angle control
3. `enable_motor(..., DCMotorMode.POSITION)` for M3 position control
4. `set_motor_position(..., max_vel_ticks=...)` to apply a velocity limit
5. Writing a simple blocking actuator sequence with `time.sleep(...)`
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    POSITION_UNIT,
    ServoChannel,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Actuator configuration — edit these to match your build
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM


# ---------------------------------------------------------------------------
# Actuator configuration — edit to match your build
# ---------------------------------------------------------------------------

# Servo — gripper jaw
GRIPPER_CHANNEL   = ServoChannel.CH_1
GRIPPER_OPEN_DEG  = 20.0   # degrees — jaw fully open
GRIPPER_CLOSE_DEG = 90.0   # degrees — jaw gripping

# Stepper — horizontal arm extension
ARM_STEPPER        = Stepper.STEPPER_1
ARM_EXTEND_STEPS   = 2000   # steps from home to over the pick station
ARM_MAX_VELOCITY   = 800    # steps/s
ARM_ACCELERATION   = 400    # steps/s²
ARM_HOME_VELOCITY  = 300    # steps/s for homing

# DC Motor — vertical lift (position control mode)
LIFT_MOTOR       = Motor.DC_M3
LIFT_UP_TICKS    = 3000   # encoder ticks — raised/travel position
LIFT_DOWN_TICKS  = 0      # encoder ticks — lowered to disk height
LIFT_MAX_VEL     = 250    # ticks/s
LIFT_TOLERANCE   = 30     # ticks


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def home_arm(robot: Robot) -> bool:
    """Home the arm stepper against its limit switch."""
    print("[FSM] HOMING — press BTN_3 to trigger the shared LIM1 input for stepper 1")
    robot.step_enable(ARM_STEPPER)
    ok = robot.step_home(
        ARM_STEPPER,
        direction=-1,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[warn] arm homing timed out — check LIM1 or use BTN_3 to simulate it")
        robot.step_disable(ARM_STEPPER)
        return False
    robot.step_disable(ARM_STEPPER)
    return True


def restore_idle_lift(robot: Robot) -> bool:
    """
    Pick a disk from the fixed pick station.

    Returns a TaskHandle when blocking=False so the FSM can poll
    is_finished() while staying responsive to button presses.
    """
    def worker(task: TaskHandle) -> None:
        # 1. Raise lift so the arm clears any obstacles while extending
        print("[SEQ] raising lift")
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_UP_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 2. Extend arm over pick station
        print("[SEQ] extending arm")
        robot.step_enable(ARM_STEPPER)
        robot.step_move(
            ARM_STEPPER,
            steps=ARM_EXTEND_STEPS,
            move_type=StepMoveType.RELATIVE,
            blocking=True,
            timeout=10.0,
        )
        if task.cancelled():
            return

        # 3. Open gripper before lowering
        print("[SEQ] opening gripper")
        robot.enable_servo(GRIPPER_CHANNEL)
        robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
        if not task.sleep(0.4):   # wait for servo to reach position
            return

        # 4. Lower lift to disk
        print("[SEQ] lowering to disk")
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_DOWN_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 5. Close gripper to grip disk
        print("[SEQ] gripping disk")
        robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
        if not task.sleep(0.5):
            return

        # 6. Raise lift with disk
        print("[SEQ] raising with disk")
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_UP_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 7. Retract arm
        print("[SEQ] retracting arm")
        robot.step_move(
            ARM_STEPPER,
            steps=ARM_EXTEND_STEPS,
            move_type=StepMoveType.RELATIVE,
            blocking=True,
            timeout=10.0,
        )
        robot.step_disable(ARM_STEPPER)

    robot.step_disable(ARM_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)

    print("[SEQ] return lift to idle")
    return restore_idle_lift(robot)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            home_arm(robot)
            restore_idle_lift(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to run the pick sequence")
            print(
                f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}° close={GRIPPER_CLOSE_DEG:.0f}°"
            )
            print(
                f"[CFG] lift low={LIFT_DOWN_TICKS} high={LIFT_UP_TICKS} ticks "
                f"max_vel={LIFT_MAX_VEL_TICKS} ticks/s"
            )
            print(
                f"[CFG] arm extend={ARM_EXTEND_STEPS} steps "
                f"home_vel={ARM_HOME_VELOCITY} steps/s"
            )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SEQUENCE")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_pick_sequence(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE — sequence complete")
            else:
                print("[FSM] IDLE — sequence stopped due to actuator timeout")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
