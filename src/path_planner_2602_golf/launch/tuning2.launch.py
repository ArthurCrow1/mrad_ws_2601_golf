import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas a los paquetes
    # Tu configuracion esta en el tracker
    tracker_pkg_dir = get_package_share_directory('path_tracker_2602_golf')
    
    params_file = os.path.join(tracker_pkg_dir, 'config', 'mppi_golf_params.yaml')

    # 2. Nodo: Servidor de Control de Nav2
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # 3. Nodo: Administrador de Ciclo de Vida (Enciende el servidor de control automaticamente)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['controller_server']}
        ]
    )

    # 4. Nodo: Tu Cliente Action en Python (que envia la ruta CSV)
    csv_client_node = Node(
        package='path_tracker_2602_golf', # Paquete donde pusiste el script
        executable='csv_to_nav2_client',
        name='csv_to_nav2_client',
        output='screen'
    )

    # 5. Agregar todo a la descripcion del lanzamiento
    ld = LaunchDescription()
    
    ld.add_action(controller_server_node)
    ld.add_action(lifecycle_manager_node)
    
    # Se recomienda un pequeno retraso (TimerAction) para el cliente en sistemas complejos, 
    # pero como dependemos de ActionClient.wait_for_server() en el codigo, podemos lanzarlo directo.
    ld.add_action(csv_client_node)

    return ld