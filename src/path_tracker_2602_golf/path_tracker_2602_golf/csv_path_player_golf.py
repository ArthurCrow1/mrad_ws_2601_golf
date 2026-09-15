#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import os

class CsvPathPlayer(Node):
    def __init__(self):
        super().__init__('csv_path_player')
        
        # Publicador estandar, sin Latch para evitar conflictos con el MPC
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.path_msg = None
        self.first_publish = True
        
        # 1. Leemos el archivo CSV una sola vez al arrancar
        self.load_path_from_csv()
        
        # 2. Creamos un bucle que envia la ruta 1 vez por segundo constantemente
        self.timer = self.create_timer(1.0, self.publish_continuous_path)

    def load_path_from_csv(self):
        home_dir = os.path.expanduser('~')
        csv_file_path = os.path.join(home_dir, 'mrad_ws_2602_golf', 'src', 'path_planner_2602_golf', 'csv', 'track_2.csv')
        
        if not os.path.exists(csv_file_path):
            self.get_logger().error(f"No se encontro el archivo: {csv_file_path}")
            return
            
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        try:
            with open(csv_file_path, mode='r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    p = PoseStamped()
                    p.header.frame_id = 'map'
                    p.pose.position.x = float(row['x'])
                    p.pose.position.y = float(row['y'])
                    p.pose.orientation.w = 1.0 
                    self.path_msg.poses.append(p)
                    
            self.get_logger().info(f"Memoria CSV cargada exitosamente: {len(self.path_msg.poses)} puntos listos.")
            
        except Exception as e:
            self.get_logger().error(f"Error leyendo el CSV: {e}")

    def publish_continuous_path(self):
        if self.path_msg is not None and len(self.path_msg.poses) > 0:
            # Actualizamos la marca de tiempo por protocolo de ROS 2
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path_msg)
            
            if self.first_publish:
                self.get_logger().info("Ruta inyectada en el MPC. Transmitiendo en bucle (1 Hz)...")
                self.first_publish = False

def main(args=None):
    rclpy.init(args=args)
    node = CsvPathPlayer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()