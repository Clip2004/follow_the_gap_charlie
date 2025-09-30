# library to move between files and folders in the O.S.
import os
from ament_index_python.packages import get_package_share_directory
# libraries to define the Launch file and Function
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='follow_the_gap_charlie' #<--- CHANGE ME

    # Define Launch configuration variables specific to simulation
    gap_finder_charlie = Node(package=package_name,
                             executable='gap_finder_charlie',
                             name='gap_finder_charlie_node',
    )
    gap_controller_charlie = Node(package=package_name,
                          executable='gap_controller_charlie',
                          name='gap_controller_charlie_node',
    )

# Launch them all!
    return LaunchDescription([
        gap_finder_charlie,
        gap_controller_charlie,
    ])