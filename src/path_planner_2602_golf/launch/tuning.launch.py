from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_tracker_2602_golf',
            executable='csv_path_player_golf',     
            name='csv_path_player_golf',
            output='screen'
        ),
        Node(
            package='path_tracker_2602_golf',
            executable='mpc_pt_2602_golf',     
            name='mpc_pt_2602_golf',
            output='screen',
            parameters=[{
                'N': 7,                 # Horizonte de prediccion
                'dt': 0.05,              # 20 Hz
                'v_ref': 0.5,            # Velocidad crucero
                'w_pos': 25.0,           # W1: Prioridad de ruta
                'w_smooth': 1.0,         # W2: Suavidad
                'w_straight': 1.5,       # W3: Evitar zig-zag
                'w_vel': 0.2,            # W4: Mantener velocidad
                'v_max': 2.0,            # Límite motor lineal
                'w_max': 4.0,            # Límite motor angular      
            }]
        ),
    ])