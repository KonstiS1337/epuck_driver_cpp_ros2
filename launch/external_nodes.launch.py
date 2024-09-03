from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

epuck_names = ['epuck1', 'epuck2', 'epuck3']
corresponding_ball_colors = ["red", "blue", "green"]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="blob_detection",
            executable="cv2_blob_detector_node",
            name = "blob_detector_node",
            parameters=[
                {'robot_names': epuck_names}, # add more parameters here if needed
                {'corresponding_colors': corresponding_ball_colors}]
        ),
        Node(
            package="formation_calibration_action_server",
            executable="formation_calibration",
            name = "formation_calibration_node",
            parameters=[
                {'robot_names': epuck_names} # add more parameters here if needed
                        ]
        ),
        #Node(
        #    package="image_centering",
        #    executable="image_centering_node",
        #    name = "image_centering_node_{}".format(epuck_name),
        #    parameters=[
        #        {'epuck_name': epuck_name} # add more parameters here if needed
        #                ]
        #),
    ])