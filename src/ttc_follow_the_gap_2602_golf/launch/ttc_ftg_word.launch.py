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



    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # --- Robot description (xacro -> URDF XML string) ---
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), "diffdrive_urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
    )

    # --- Launch Gazebo (via ros_gz_sim launch file) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    # --- Spawn entity into Gazebo from robot_description topic ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "diffbot",
            "-topic", "robot_description",
            "-x", "-15", "-y", "-3", "-z", "2", '-Y', '1.5708'
            #"-x", "-6", "-y", "2", "-z", "0.5", '-Y', '-1.5708' #track1 
        ],
    )

    bridge_params = os.path.join(get_package_share_directory(gazebo_pkg_name),'config','topic_bridge.yaml')

    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    output="screen",
    arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller"],     #EL MISMO EN CONTROLLERS YAML
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster_controller"],
    )


    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/diffdrive_controller/cmd_vel')]
    )

    
    ttc_min_arg = DeclareLaunchArgument('ttc_min', default_value='1.5', description='Tiempo minimo de colision permitido')
    v_max_arg = DeclareLaunchArgument('v_max', default_value='1.2', description='Velocidad max')
    kp_arg = DeclareLaunchArgument('kp_steering', default_value='1.5', description='Potencia del volante')

    # 
    gap_finder_node = Node(
        package='ttc_follow_the_gap_2602_golf',
        executable='ttc_gap_finder',
        name='ttc_gap_finder',
        output='screen',
        parameters=[{
            'ttc_min': LaunchConfiguration('ttc_min')
        }]
    )

    # 
    control_node = Node(
        package='ttc_follow_the_gap_2602_golf',
        executable='ttc_gap_control',
        name='ttc_gap_control',
        output='screen',
        parameters=[{
            'ttc_min': LaunchConfiguration('ttc_min'),
            'v_max': LaunchConfiguration('v_max'),
            'kp_steering': LaunchConfiguration('kp_steering')
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(get_package_share_directory(gazebo_pkg_name), "worlds", "exam_1_2.sdf"),#exam_1_2.sdf
            description="Full path to world SDF file",
        ),


        #ttc_min_arg,
        #v_max_arg,
        #kp_arg,

        rsp,
        gz_launch,
        spawn,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner,
        joy_node,
        teleop_node,
        twist_mux_node,
        #gap_finder_node,
        #control_node,

    ])