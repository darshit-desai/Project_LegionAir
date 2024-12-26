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
    
    # Extract the id parameter
     # Get namespace from environment variable
    uav_namespace = os.getenv('UAV_NAMESPACE')
    
    if uav_namespace is None:
        raise RuntimeError("Environment variable UAV_NAMESPACE must be set (e.g., uav_1, uav_2, etc.)")
    
    # Add the forward slash prefix
    uav_namespace = f"/{uav_namespace}"
    ld = LaunchDescription()
    
    tf_static_node = Node(
        package="your_tf_package",
        executable="tf_publisher_cpp",
        namespace=uav_namespace,
        output="screen",
    )

    pc_transform_node = Node(
        package="your_pointcloud_package", 
        executable="pointcloud_transformer", 
        output="screen",
        namespace=uav_namespace,
    )

    obj_det_node = Node(
        package="tflite_prop_detection", 
        executable="tflite_prop_detection_cpp", 
        output="screen",
        namespace=uav_namespace,
    )
    
    
    # # Add nodes with delay
    ld.add_action(TimerAction(
        period=0.0,
        actions=[tf_static_node]
    ))
    ld.add_action(TimerAction(
        period=0.0,
        actions=[pc_transform_node]
    ))
    ld.add_action(TimerAction(
        period=0.0,
        actions=[obj_det_node]
    ))


    return ld
