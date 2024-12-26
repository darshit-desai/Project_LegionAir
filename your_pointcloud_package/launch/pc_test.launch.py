import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    tf_static_node = Node(
        package="your_tf_package",
        executable="tf_publisher_cpp",
        output="screen",
    )

    pc_transform_node = Node(
        package="your_pointcloud_package", 
        executable="pointcloud_transformer", 
        output="screen"
    )

    nodes_to_start = [
        tf_static_node,
        pc_transform_node
    ]

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )

