import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path



def generate_launch_description():

    

    launch_tflite_package = os.path.join(get_package_share_path('tflite_prop_detection'),"launch","tflite_prop_detection.launch.py")

    include_tflite_package = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_tflite_package))
    # Get namespace from environment variable
    uav_namespace = os.getenv('UAV_NAMESPACE')
    if uav_namespace is None:
        raise RuntimeError("Environment variable UAV_NAMESPACE must be set (e.g., uav_1, uav_2, etc.)")
    
    # Add the forward slash prefix
    uav_namespace = f"/{uav_namespace}"

    ld = LaunchDescription()
    

    offboard_control_node = Node(
        package="px4_ros_com", 
        executable="offboard_control_constant_velocity", 
        output="screen",
        namespace=uav_namespace
    )

    ld.add_action(TimerAction(
        period=2.0,
        actions=[offboard_control_node]
    ))

    ld.add_action(include_tflite_package)

    return ld
