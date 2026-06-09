from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COMMON_PARAMETERS = {
    "camera_width": 640,
    "camera_height": 480,
    "camera_fps": 5.0,
    "process_rate_hz": 1.5,
    "model_imgsz": 416,
    "confidence_threshold": 0.25,
    "iou_threshold": 0.7,
    "max_detections": 20,
    "ncnn_threads": 2,
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/video10",
                description="The V4L2 device path for the camera.",
            ),
            Node(
                package="vision",
                executable="vision_node",
                name="vision_node",
                prefix=["nice -n 15 taskset -c 2,3"],
                output="screen",
                parameters=[
                    COMMON_PARAMETERS,
                    {
                        "camera_device": LaunchConfiguration("camera_device"),
                        "debug_save_enabled": False,
                    },
                ],
            )
        ]
    )
