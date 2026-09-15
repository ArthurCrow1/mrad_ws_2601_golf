#!/usr/bin/env python3
"""
Race Manager Node
Orquesta la solicitud de rutas al PRM, ensambla las vueltas y se las envía al MPC.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import time
from rclpy.qos import QoSProfile, DurabilityPolicy

import csv
import os

class RaceManagerNode(Node):
    def __init__(self):
        super().__init__('race_manager_node')
        
        # 1. El ReentrantCallbackGroup permite hacer llamadas síncronas sin bloquear ROS 2
        self.cb_group = ReentrantCallbackGroup()
        
        # Cliente del Servicio PRM y Publicador para el MPC
        self.prm_client = self.create_client(GetPlan, 'get_prm_plan', callback_group=self.cb_group)
        #self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/planned_path', latch_qos)
        
        # Lector de TF para saber de dónde arranca el carro
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ==========================================================
        # TODO: Coordenadas puntos
        # ==========================================================
        #Lista SIMPLE
        # (10.7, -0.445),    # Inicio S
        # (10.4, -5.44),     #Fin primera curva
        # (3.27, -14.0),     # Fin S
        # (-0.0447, -24.6),  # Mitad curva grande
        # (9.63, -19.5),     # Fin curva grande
        # (16.2, -27.2),     # Inicio diagonal
        # (-0.315, -31.1),   # Fin diagonal
        # (-5.38, -20.9),    # Inicio recta
        # (-6.11, -12.0)     # Punto crítico recta  

        self.checkpoints = [
            (10.7, -0.445),    # Inicio S
            (12.2, -3.44),      #Mitad curva 1
            (10.4, -5.44),     #Fin primera curva
            (4.1, -3.81),      #Inicio segunda vuelta
            (2.0, -5.85),      #Mitad curva 2
            (4.0, -8.4),       #Fin curva 2
            (10.2, -9.2),      #Inicio curva 3 
            (11, -11.4),       #Mitad de curva 3
            (-0.9, -17.0),     # Fin S
            (-0.0447, -24.6),  # Mitad curva grande
            (9.63, -19.5),     # Fin curva grande
            (16.2, -27.2),     # Inicio diagonal
            (-0.315, -31.1),   # Fin diagonal
            (-5.38, -20.9),    # Inicio recta
            (-6.11, -12.0)     # Punto crítico recta
        ]
        self.num_laps = 2
        
        # Usamos un Timer de 1 segundo para darle tiempo a Gazebo de cargar todo
        self.timer = self.create_timer(1.0, self.orchestrate_race, callback_group=self.cb_group)

    def orchestrate_race(self):
        self.timer.cancel()  # Cancelamos el timer para que se ejecute una sola vez
        self.get_logger().info("Race Manager iniciado. Esperando al PRM...")
        
        self.prm_client.wait_for_service()
        self.get_logger().info("¡PRM detectado!")
        
        # 1. Obtener la posición inicial del carro físicamente
        start_pose = self.get_initial_pose()
        if not start_pose:
            self.get_logger().error("No se detectó el TF del robot. Abortando.")
            return
        
        # Añadimos la posición EXACTA de inicio como el último checkpoint para cerrar el ciclo
        self.checkpoints.append((start_pose.pose.position.x, start_pose.pose.position.y))

        self.get_logger().info("Calculando la vuelta base...")
        current_start = start_pose
        single_lap_poses = []
        
        # 2. Pedir los segmentos al PRM uno por uno
        for i, (cx, cy) in enumerate(self.checkpoints):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = float(cx)
            goal_pose.pose.position.y = float(cy)
            
            request = GetPlan.Request()
            request.start = current_start
            request.goal = goal_pose
            
            self.get_logger().info(f"Pidiendo tramo {i+1} hacia ({cx}, {cy})...")
            
            # Llamada al servicio y pausa hasta recibir la respuesta
            response = self.prm_client.call(request)
            
            while not response.plan.poses:
                self.get_logger().warn(f"El PRM aún no está listo (probablemente calculando telaraña). Reintentando en 2 segundos...")
                time.sleep(2.0)
                response = self.prm_client.call(request)
            
            # Ensamblar evitando duplicar el punto de conexión exacto
            if single_lap_poses:
                single_lap_poses.extend(response.plan.poses[1:])
            else:
                single_lap_poses.extend(response.plan.poses)
                
            # El destino de este tramo es el inicio del siguiente
            current_start = goal_pose
        
        # 3. Multiplicar las vueltas
        self.get_logger().info(f"Vuelta base lista. Multiplicando por {self.num_laps} vueltas...")
        final_race_poses = []
        for _ in range(self.num_laps):
            if final_race_poses:
                # Evitamos duplicar la línea de meta al unir la V1 con la V2
                final_race_poses.extend(single_lap_poses[1:]) 
            else:
                final_race_poses.extend(single_lap_poses)

        # 4. Construir el mensaje Final
        final_path_msg = Path()
        final_path_msg.header.frame_id = 'map'
        final_path_msg.header.stamp = self.get_clock().now().to_msg()
        final_path_msg.poses = final_race_poses

        # Guardar la ruta en un archivo CSV
        # Se guarda en la raiz del workspace para encontrarlo facil
        # /home/arthur/mrad_ws_2602_golf/src/path_planner_2602_golf/csv
        home_dir = os.path.expanduser('~')
        csv_file_path = os.path.join(home_dir, 'mrad_ws_2602_golf', 'src', 'path_planner_2602_golf', 'csv', 'track_2.csv')
        
        try:
            with open(csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y']) # Encabezados
                for p in final_race_poses:
                    writer.writerow([p.pose.position.x, p.pose.position.y])
            self.get_logger().info(f"Ruta guardada exitosamente en: {csv_file_path}")
        except Exception as e:
            self.get_logger().error(f"No se pudo guardar el CSV: {e}")
        
        # Publicamos unas cuantas veces para asegurar que el MPC (que acaba de arrancar) reciba el mensaje
        for _ in range(3):
            self.path_pub.publish(final_path_msg)
            time.sleep(0.5)
            
        self.get_logger().info(f"¡CARRERA ENVIADA AL MPC! Puntos totales: {len(final_race_poses)}. ¡Acelerando!")

    def get_initial_pose(self):
        # Intenta leer el TF durante 5 segundos
        for _ in range(5):
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                return pose
            except Exception:
                time.sleep(1.0)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = RaceManagerNode()
    
    # El MultiThreadedExecutor permite procesar el TF y los Servicios en paralelo
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()