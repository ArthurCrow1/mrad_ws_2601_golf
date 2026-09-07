import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # parametros
    ttc_recta_arg = DeclareLaunchArgument('ttc_recta', default_value='1.2')#0.4
    ttc_curva_arg = DeclareLaunchArgument('ttc_curva', default_value='1.5')#1.5
    v_max_arg = DeclareLaunchArgument('v_max', default_value='2.0', description='Velocidad máxima en recta')
    kp_arg = DeclareLaunchArgument('kp_steering', default_value='2.5', description='Ganancia proporcional del volante')
    kd_arg = DeclareLaunchArgument('kd_steering', default_value='0.8', description='Ganancia derivativa del volante')
    #0.4
    #1.6
    #2.0
    #1.5
    #0.5
    #Encontrar
    gap_finder_node = Node(
        package='ttc_follow_the_gap_2602_golf',
        executable='ttc_gap_finder',
        name='ttc_gap_finder',
        output='screen',
        parameters=[{
            'ttc_min': LaunchConfiguration('ttc_curva')
        }]
    )

    # control
    control_node = Node(
        package='ttc_follow_the_gap_2602_golf',
        executable='ttc_gap_control',
        name='ttc_gap_control',
        output='screen',
        parameters=[{
            'ttc_recta': LaunchConfiguration('ttc_recta'),
            'ttc_curva': LaunchConfiguration('ttc_curva'),
            'v_max': LaunchConfiguration('v_max'),
            'kp_steering': LaunchConfiguration('kp_steering'),
            'kd_steering': LaunchConfiguration('kd_steering')
        }]
    )

    return LaunchDescription([
        ttc_recta_arg,
        ttc_curva_arg,
        v_max_arg,
        kp_arg,
        kd_arg,
        gap_finder_node,
        control_node
    ])