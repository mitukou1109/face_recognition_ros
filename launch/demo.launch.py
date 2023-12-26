from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera_node",
        output="screen",
    )

    face_identifier_node = Node(
        package="face_recognition_ros",
        executable="face_identifier",
        name="face_identifier",
        output="screen",
    )

    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
    )

    return LaunchDescription(
        [v4l2_camera_node, face_identifier_node, rqt_image_view_node]
    )
