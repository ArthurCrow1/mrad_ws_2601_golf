from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='path_planner_2602_golf',
        #     executable='dijkstra_pp_2602_golf',
        #     name='dijkstra_pp_2602_golf',
        #     output='screen',
        #     parameters=[{
        #         'map_topic': '/map',
        #         'goal_topic': '/goal_pose',
        #         'path_topic': '/planned_path',
        #         'base_frame': 'base_link',
        #         'global_frame': 'map',
        #         'occupied_threshold': 65,
        #         'treat_unknown_as_obstacle': True,
        #         'use_8_connected': True,
        #         'prevent_corner_cutting': True,
        #         'inflate_radius': 0.25,          # robot radius + margin
        #         'traversal_cost_weight': 1.5,    # 0.0 = pure geometric
        #     }]
        # ),
        Node(
            package='path_planner_2602_golf',
            executable='prm_pp_2602_golf',      
            name='prm_pp_2602_golf',            
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'goal_topic': '/goal_pose',
                'path_topic': '/planned_path',
                'base_frame': 'base_link',
                'global_frame': 'map',
                'occupied_threshold': 65,
                'treat_unknown_as_obstacle': True,
                'inflate_radius': 0.45,         
                # --- Parámetros exclusivos del PRM ---
                'density': 12.0,                # Puntos por metro cuadrado libre
                'k_neighbors': 10,               # Conexiones por nodo (subido a 5 para más fluidez)
                'max_edge_dist': 6.0,           # Longitud máxima de las líneas
            }]
        ),

        # Node(
        #     package='path_tracker_2602_golf',
        #     executable='pure_pursuit_pt_2602_golf',
        #     name='pure_pursuit_pt_2602_golf',
        #     output='screen',
        #     parameters=[{
        #         'path_topic': '/planned_path',   # MUST match planner output
        #         'cmd_vel_topic': '/cmd_vel_nav',
        #         'base_frame': 'base_link',
        #         'control_rate_hz': 20.0,
        #         'v_nominal': 0.5,
        #         'max_speed': 1.0,
        #         'max_omega': 1.0,
        #         'goal_tolerance': 0.25,
        #         'lookahead_L0': 0.6,
        #         'lookahead_kv': 0.0,
        #         'lookahead_min': 0.4,
        #         'lookahead_max': 2.0,
        #     }]
        # ),

        Node(
            package='path_tracker_2602_golf',
            executable='mpc_pt_2602_golf',     
            name='mpc_pt_2602_golf',
            output='screen',
            parameters=[{
                'N': 10,                 # Horizonte de prediccion
                'dt': 0.05,              # 20 Hz
                'v_ref': 0.5,            # Velocidad crucero
                'w_pos': 15.0,           # W1: Prioridad de ruta
                'w_smooth': 0.1,         # W2: Suavidad
                'w_straight': 1.5,       # W3: Evitar zig-zag
                'w_vel': 0.2,            # W4: Mantener velocidad
                'v_max': 1.0,            # Límite motor lineal
                'w_max': 2.5,            # Límite motor angular
            }]
        ),

        Node(
            package='path_planner_2602_golf', # O 'path_tracker_2602_golf', según dónde lo guardaste
            executable='race_manager_golf',   # El nombre sin el .py si así lo declaraste en el setup
            name='race_manager_node',
            output='screen'
        ),
    ])