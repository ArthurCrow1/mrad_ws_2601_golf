import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo_pkg_name = "gazebo_2602_golf"
    bringup_pkg_name = "bringup_2602_golf"
    description_pkg_name = "description_2602_golf"

    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick_real.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params, {'use_sim_time': False}],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': False}],
                    remappings=[('/cmd_vel_out','/cmd_vel_stamped')]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(get_package_share_directory(gazebo_pkg_name), "worlds", "exam_1.sdf"),
            description="Full path to world SDF file",
        ),
        
        joy_node,
        teleop_node,
        twist_mux_node,
        
    ])