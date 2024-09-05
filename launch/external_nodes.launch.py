from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

epuck_names = ['epuck1', 'epuck2', 'epuck3']
corresponding_ball_colors = ["blue" , "green", "red"]


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
        Node(
            package="epuck_sound_data_collector",
            executable="epuck_sound_collection_action_server",
            name = "sound_collection_action_server",
            parameters=[
                {'robot_names': epuck_names},
                {'num_microphones': 3}]
        ),  

        Node(
            package="sound_loc_controller",
            executable="sound_loc_controller_node",
            name = "TeamController",
            parameters=[
                {'robot_names': epuck_names},
                {'robot_distance': 0.1},
                {'recording_time': 3},
                {'step_distance': 0.1},
                {'average_mic_amplitudes_per_robot': True},
                ]
        )
    ])