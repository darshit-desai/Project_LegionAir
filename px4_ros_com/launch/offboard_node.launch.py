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
    offboard_node_params = PathJoinSubstitution(
        [FindPackageShare('px4_ros_com'), "config", "ns_conf.yaml"]
    )
    pack_path = os.path.join(get_package_share_path('px4_ros_com'), "config", "ns_conf.yaml")
    print(pack_path)
    with open(pack_path) as file:
        config = yaml.safe_load(file)

    # Extract the id parameter
    id_value = config['offboard_control_node']['ros__parameters']['id']
    ld = LaunchDescription()
    
    # tf_static_node = Node(
    #     package="your_tf_package",
    #     executable="tf_publisher_cpp",
    #     namespace='uav_'+str(id_value),
    #     output="screen",
    # )

    offboard_control_node = Node(
        package="px4_ros_com", 
        executable="offboard_control_constant_velocity", 
        output="screen",
        namespace='/uav_'+str(id_value),
        parameters=[offboard_node_params]
    )

    # obj_det_node = Node(
    #     package="tflite_prop_detection", 
    #     executable="tflite_prop_detection_cpp", 
    #     output="screen",
    #     namespace='uav_'+str(id_value),
    #     parameters=[tflite_node_params]
    # )
    
    
    
    
    # tf_static_node_with_namespace = GroupAction( 
    #     actions = [PushRosNamespace('uav_' + str(id_value)), tf_static_node]
    # )
    # pc_transform_node_with_namespace = GroupAction( 
    #     actions = [PushRosNamespace('uav_' + str(id_value)), pc_transform_node]
    # )
    # obj_det_node_with_namespace = GroupAction( 
    #     actions = [PushRosNamespace('uav_' + str(id_value)), obj_det_node]
    # )
    
    # kalman_filter_node = Node(
    #     package='kalman_filter_node',
    #     executable='kalman_filter_node',
    #      namespace='uav_'+str(id_value),
    #     output='screen',
    # )

    # offboard_control_node = Node(
    #     package='px4_ros_com',
    #     executable='offboard_control',
    #      namespace='uav_'+str(id_value),
    #     output='screen',
    # )
    
    # # Add nodes with delay
    # ld.add_action(TimerAction(
    #     period=0.0,
    #     actions=[tf_static_node]
    # ))
    # ld.add_action(TimerAction(
    #     period=0.0,
    #     actions=[pc_transform_node]
    # ))
    # ld.add_action(TimerAction(
    #     period=0.0,
    #     actions=[obj_det_node]
    # ))
    # ld.add_action(TimerAction(
    #     period=2.0,
    #     actions=[kalman_filter_node]
    # ))
    ld.add_action(TimerAction(
        period=0.0,
        actions=[offboard_control_node]
    ))

    return ld
