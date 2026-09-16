#!/usr/bin/env python3
import os
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import tf2_ros

class CsvToNav2Client(Node):
    def __init__(self):
        super().__init__('csv_to_nav2_client', allow_undeclared_parameters=True)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        
        # NUEVO: Herramientas para leer la posicion actual del carro
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.goal_sent = False
        # Un temporizador que intenta arrancar la mision cada segundo
        self.timer = self.create_timer(1.0, self.attempt_start)

    def attempt_start(self):
        if self.goal_sent:
            return
            
        try:
            # Preguntar donde esta el carro exactamente en este instante
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            self.get_logger().info(f"Posicion inicial del carro detectada: X={rx:.2f}, Y={ry:.2f}")
            
            # Leer el CSV y cortarlo desde esta posicion
            path_msg = self.load_path_from_csv(rx, ry)
            
            if path_msg:
                self.send_path_goal(path_msg)
                self.goal_sent = True
                self.timer.cancel()
                
        except Exception as e:
            self.get_logger().info("Buscando el carro en el mapa (TF)... esperando sincronizacion.")

    def load_path_from_csv(self, robot_x, robot_y):
        home_dir = os.path.expanduser('~')
        # Asegurate de que la ruta coincida con la tuya
        csv_file_path = os.path.join(home_dir, 'mrad_ws_2602_golf', 'src', 'path_planner_2602_golf', 'csv', 'track_1.csv')
        
        if not os.path.exists(csv_file_path):
            self.get_logger().error("No se encontro el archivo CSV.")
            return None
            
        path = Path()
        path.header.frame_id = 'map'
        
        try:
            raw_points = []
            with open(csv_file_path, mode='r') as file:
                reader = csv.DictReader(file)
                last_x, last_y = None, None
                
                for row in reader:
                    curr_x = float(row['x'])
                    curr_y = float(row['y'])
                    
                    if last_x is not None and last_y is not None:
                        dist = math.hypot(curr_x - last_x, curr_y - last_y)
                        if dist < 0.02:
                            continue
                            
                    raw_points.append((curr_x, curr_y))
                    last_x = curr_x
                    last_y = curr_y

            # NUEVO: Encontrar el punto de la ruta mas cercano al parachoques del carro
            distances = [math.hypot(px - robot_x, py - robot_y) for px, py in raw_points]
            closest_idx = distances.index(min(distances))
            
            # Cortar la ruta: descartar todo lo que quedo atras del carro
            relevant_points = raw_points[closest_idx:]
            
            last_yaw = 0.0
            for i in range(len(relevant_points)):
                x, y = relevant_points[i]
                
                # Calcular orientacion basandose en el punto siguiente
                if i < len(relevant_points) - 1:
                    nx, ny = relevant_points[i+1]
                    yaw = math.atan2(ny - y, nx - x)
                    last_yaw = yaw
                else:
                    yaw = last_yaw
                    
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.orientation.z = math.sin(yaw / 2.0)
                p.pose.orientation.w = math.cos(yaw / 2.0)
                
                path.poses.append(p)
                
            path.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f"Ruta recortada inteligentemente: {len(path.poses)} puntos por delante del carro listos.")
            return path
            
        except Exception as e:
            self.get_logger().error(f"Error procesando CSV: {e}")
            return None

    def send_path_goal(self, path_msg):
        self.get_logger().info('Esperando al servidor de control (/follow_path)...')
        self.action_client.wait_for_server()
        
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Ruta rechazada.')
            return
            
        self.get_logger().info('¡Ruta aceptada! Pisando el acelerador.')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Mision finalizada con estado: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CsvToNav2Client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()