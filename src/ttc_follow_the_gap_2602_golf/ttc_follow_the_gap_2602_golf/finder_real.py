#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TTCGapFinder(Node):
    def __init__(self):
        super().__init__('ttc_gap_finder')

        self.declare_parameter('ttc_min', 2.5)  #Tiempo minimo de choque
        self.current_vel = 0.0
        self.declare_parameter('car_width', 0.3)   #Ancho carro
        self.vel_sub = self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.vel_callback, 10)    #obtener velocidad del robot
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)        
        self.angle_pub = self.create_publisher(Float32, '/ttc_target_angle', 10)
        self.ttc_pub = self.create_publisher(Float32, '/ttc_min_val', 10)
        self.marker_pub = self.create_publisher(Marker, '/gap_visualizer', 10)
        self.declare_parameter('safe_dist', 0.5)  #distancia seguridad frontal

    def vel_callback(self, msg):
        self.current_vel = msg.twist.linear.x

    def scan_callback(self, msg):
        ttc_min_param = self.get_parameter('ttc_min').value
        car_width_param = self.get_parameter('car_width').value
        safe_dist_param = self.get_parameter('safe_dist').value
        
        ranges = np.array(msg.ranges)
        
        # Rotacion de 180 grados
        raw_angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = raw_angles + np.pi
        angles = np.where(angles > np.pi, angles - 2 * np.pi, angles)
        # Punto ciego lidar
        ranges = np.where(ranges < 0.15, 0.0, ranges)
        ranges = np.where(~np.isfinite(ranges), 10.0, ranges)
        # Inflar radio
        inflation_radius = (car_width_param / 2.0) + 0.05
        inflated_ranges = np.copy(ranges)
        
        for i in range(len(ranges)):
            dist = ranges[i]
            # inflacion dentro del rango real
            if 0.15 < dist < 10.0:
                # Calcula la apertura angular que ocupa el radio del vehiculo a esta distancia
                angle_span = math.atan2(inflation_radius, dist)
                num_indices = int(angle_span / abs(msg.angle_increment))
                # Define los topes del arreglo para evitar desbordamientos de indice
                start_idx = max(0, i - num_indices)
                end_idx = min(len(ranges), i + num_indices + 1)                
                # CORRECCION: En lugar de asignar 0.0 (colision fantasma), propaga la distancia real del obstaculo sobre la zona inflada
                inflated_ranges[start_idx:end_idx] = np.minimum(inflated_ranges[start_idx:end_idx], dist)
                
        #Nuevo arreglo filtrado
        ranges = inflated_ranges
        
        v = self.current_vel
        ttcs = np.full(len(ranges), np.inf)
        
        if v > 0.01: 
            cos_angles = np.cos(angles)
            valid_mask = cos_angles > 0.0
            ttcs[valid_mask] = ranges[valid_mask] / (v * cos_angles[valid_mask])

        front_mask = (angles > -math.pi/2) & (angles < math.pi/2)
        safe_rays = (ttcs >= ttc_min_param) & (ranges > safe_dist_param) & front_mask

        diff = np.diff(safe_rays.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0] + 1
        
        if safe_rays[0]:
            starts = np.insert(starts, 0, 0)
        if safe_rays[-1]:
            ends = np.append(ends, len(ranges))     
            
        if len(starts) == 0:
            self.get_logger().warn("Bloqueo total: Ningun camino frontal cumple la distancia o tiempo seguro.")
            self.publish_targets(0.0, 0.0, valid=False)
            return
            
        best_gap_idx = 0
        best_score = -1

        for i in range(len(starts)):
            s = starts[i]
            e = ends[i]
            gap_width_rays = e - s
            
            if gap_width_rays < 5:  #descartar gaps de solo 5 rayos
                continue
                
            r1 = ranges[s]
            r2 = ranges[e - 1]
            gap_angle = gap_width_rays * msg.angle_increment
            physical_width = math.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * math.cos(gap_angle))
            if physical_width < 0.05:    #filtro de holgura; no justo chasis, chasis + 0.05
                continue
            
            gap_depth = np.mean(ranges[s:e])
            score = physical_width * gap_depth

            if score > best_score:
                best_score = score
                best_gap_idx = i

        if best_score != -1:
            best_start = starts[best_gap_idx]
            best_end = ends[best_gap_idx]
            center_idx = int((best_start + best_end - 1) / 2)
            target_angle = angles[center_idx]
            is_valid = True
        else:
            target_angle = 0.0
            is_valid = False
            self.get_logger().warn("NINGUN GAP VALIDO DETECTADO")

        min_current_ttc = np.min(ttcs)
        self.publish_targets(target_angle, min_current_ttc, valid=is_valid)

    # Visualizacion y angulo, etc
    def publish_targets(self, angle, min_ttc, valid=True):
        msg_angle = Float32()
        msg_angle.data = float(angle)
        self.angle_pub.publish(msg_angle)
        
        msg_ttc = Float32()
        msg_ttc.data = float(min_ttc)
        self.ttc_pub.publish(msg_ttc)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "optimal_gap"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        p1 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.0
        
        distancia_visual = 1.5
        p2 = Point()
        p2.x = distancia_visual * math.cos(angle)
        p2.y = distancia_visual * math.sin(angle)
        p2.z = 0.0
        
        marker.points = [p1, p2]
        
        # Cambia el color dependiendo de si el gap es valido
        if valid:
            # Verde para camino libre
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            # Rojo para indicar que no hay camino
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
        marker.color.a = 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TTCGapFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()