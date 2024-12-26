# Import necessary libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# Define the launch description
def generate_launch_description():
    ld = LaunchDescription()
    # Declare a command-line argument "cmd_line_parameter"
    # cmd_line_parameter = DeclareLaunchArgument(
    #     "mode",
    #     default_value="away",
    #     description="A parameter from the command line.",
    # )
    # Path to the parameters file
    node_params_0 = PathJoinSubstitution(
        [FindPackageShare("drone_cpp"), "config", "params_0.yaml"]
    )
    node_params_1 = PathJoinSubstitution(
        [FindPackageShare("drone_cpp"), "config", "params_1.yaml"]
    )
    node_params_2 = PathJoinSubstitution(
        [FindPackageShare("drone_cpp"), "config", "params_2.yaml"]
    )
    node_params_3 = PathJoinSubstitution(
        [FindPackageShare("drone_cpp"), "config", "params_3.yaml"]
    )
    
    drone_node_cpp_0=Node(
            package='drone_cpp',
            name='drone_zero',
            executable='drone_node_cpp',
            parameters=[node_params_0],
            
    )
    print("Drone zero is created")
    """
    drone_node_cpp_1=Node(
            package='drone_cpp',
            name='drone_one',
            executable='drone_node_cpp',
            parameters=[node_params_1],
            
    )
    print("Drone one is created")
    drone_node_cpp_2=Node(
            package='drone_cpp',
            name='drone_two',
            executable='drone_node_cpp',
            parameters=[node_params_2],
            
    )
    print("Drone two is created")
    drone_node_cpp_3=Node(
            package='drone_cpp',
            name='drone_three',
            executable='drone_node_cpp',
            parameters=[node_params_3],
            
    )
    print("Drone three is created")
    
    # Add the actions to the launch description
    # ld.add_action(cmd_line_parameter)
    """
    ld.add_action(drone_node_cpp_0)
    #ld.add_action(drone_node_cpp_1)
    #ld.add_action(drone_node_cpp_2)
    #ld.add_action(drone_node_cpp_3)
  
    return ld
