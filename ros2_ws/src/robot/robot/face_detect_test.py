from __future__ import annotations
import time
from robot.robot import Robot, FirmwareState
from robot.hardware_map import Button
from robot.face_helpers import CustomerClassifier

def run(robot: Robot) -> None:
    # 1. Setup Robot State
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    # 2. Initialize the Face Classification Helper
    # This loads the model once and keeps it ready.
    classifier = CustomerClassifier(robot)

    print("\n" + "="*40)
    print(" FACE HELPER TEST SCRIPT ")
    print("="*40)
    print("1. Start 'ros2 run vision vision_node'")
    print("2. Stand in front of camera.")
    print("3. Press BTN_1 to identify gender.")
    print("Waiting for trigger...")

    while True:
        if robot.was_button_pressed(Button.BTN_1):
            print("\n[ACTION] Identifying customer...")
            
            # Use the new helper! 
            # wait_for_face=2.0 means it will keep looking for 2 seconds 
            # if a face isn't immediately visible.
            gender = classifier.get_gender(wait_for_face=2.0)

            if gender:
                print(f"[RESULT] Customer is a {gender}")
            else:
                print("[RESULT] No face detected. Please try again.")

        time.sleep(0.05)

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
