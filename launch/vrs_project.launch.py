from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

epuck_name = 'epuck1' #adjust this per epuck

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="epuck_driver_cpp_ros2",
            executable="epuck_driver_cpp_ros2_node",
            name = "epuck_driver_cpp_ros2_node_{}".format(epuck_name),
            parameters=[
                {'epuck_name': epuck_name} # add more parameters here if needed
                        ]
        ),
        Node(
            package="epuck_movement_controller",
            executable="epuck_movement_controller_node",
            name = "epuck_movement_controller_node_{}".format(epuck_name),
            parameters=[
                {'epuck_name': epuck_name} # add more parameters here if needed
                        ]
        ),
        Node(
            package="image_centering",
            executable="image_centering_node",
            name = "image_centering_node_{}".format(epuck_name),
            parameters=[
                {'epuck_name': epuck_name} # add more parameters here if needed
                        ]
        ),
        Node(
            package="epuck_tof_ros2",
            executable="VL53L0X_node",
            name = "epuck_tof_ros2_node_{}".format(epuck_name),
            parameters=[
                {'epuck_name': epuck_name} # add more parameters here if needed
                        ]
        ),
        Node(
            package="epuck_ros2_camera",
            executable="camera",
            name = "epuck_camera_node_{}".format(epuck_name),
            namespace = epuck_name,
            parameters=[
                {'epuck_name': epuck_name} # add more parameters here if needed
                        ]
        )  
    ])