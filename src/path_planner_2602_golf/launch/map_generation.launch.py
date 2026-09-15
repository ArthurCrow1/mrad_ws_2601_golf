from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner_2602_golf',
            executable='prm_pp_2602_golf',      
            name='prm_pp_2602_golf',            
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'path_topic': '/planned_path',
                'base_frame': 'base_link',
                'global_frame': 'map',
                'occupied_threshold': 65,
                'treat_unknown_as_obstacle': True,
                'inflate_radius': 0.45,         
                'density': 18.0,                
                'k_neighbors': 10,               
                'max_edge_dist': 6.0,           
            }]
        ),
        Node(
            package='path_planner_2602_golf', 
            executable='race_manager_golf',   
            name='race_manager_node',
            output='screen'
        ),
    ])