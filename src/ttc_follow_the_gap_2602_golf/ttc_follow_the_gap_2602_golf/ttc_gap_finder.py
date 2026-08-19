#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

class TTCGapFinder(Node):
    def __init__(self):
        super().__init__('ttc_gap_finder')

        self.declare_parameter('ttc_min', 1.5)  #tiempo a chocar
        self.current_vel = 0.0

        self.odom_sub = self.create_subscription(Odometry, '/diffdrive_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)        
        self.angle_pub = self.create_publisher(Float32, '/ttc_target_angle', 10)
        self.ttc_pub = self.create_publisher(Float32, '/ttc_min_val', 10)
        
        #self.get_logger().info("FTG+TTC finder iniciado")

    def odom_callback(self, msg):
        # velocidad en x
        self.current_vel = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        ttc_min_param = self.get_parameter('ttc_min').value
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        # cambiar inf por 10
        ranges[~np.isfinite(ranges)] = 10
        v = self.current_vel
        ttcs = np.full(len(ranges), np.inf)
        
        # tiempo de colision
        if v > 0.01: 
            cos_angles = np.cos(angles)
            # ign rayos de atras
            valid_mask = cos_angles > 0.0
            # ttc para rangos validos; dist/ (v*/cos(theta))
            ttcs[valid_mask] = ranges[valid_mask] / (v * cos_angles[valid_mask])

        # rayos seguros dado el ttc_min
        safe_rays = ttcs >= ttc_min_param
        # minimmo de dist para evitar error por v baja
        safe_rays = safe_rays & (ranges > 0.5)
        # cortar vision, lo mismo, se puede quitar por el cambio de jorge
        front_mask = (angles > -math.pi/2) & (angles < math.pi/2)
        safe_rays = safe_rays & front_mask

        # encontrar el gap, si es seguro 1 o no -1
        diff = np.diff(safe_rays.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0] + 1
        
        if safe_rays[0]:
            starts = np.insert(starts, 0, 0)
        if safe_rays[-1]:
            ends = np.append(ends, len(ranges))     
        if len(starts) == 0:
            self.publish_targets(0.0, 0.0)
            return
        # anchura
        best_gap_idx = 0
        best_score = -1
        
        for i in range(len(starts)):
            s = starts[i]
            e = ends[i]
            gap_width_rays = e - s
            # minimo de 5 rayos 
            if gap_width_rays < 5:
                continue
            # lo mismo que en ftg, calcular ancho
            r1 = ranges[s]
            r2 = ranges[e - 1]
            gap_angle = gap_width_rays * msg.angle_increment
            physical_width = math.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * math.cos(gap_angle))
            # prom profundidad
            gap_depth = np.mean(ranges[s:e])
            score = physical_width * gap_depth

            if score > best_score:
                best_score = score
                best_gap_idx = i

        best_start = starts[best_gap_idx]
        best_end = ends[best_gap_idx]

        # el centro del best
        center_idx = int((best_start + best_end - 1) / 2)
        target_angle = msg.angle_min + (center_idx * msg.angle_increment)
        min_current_ttc = np.min(ttcs)
        
        self.publish_targets(target_angle, min_current_ttc)

    def publish_targets(self, angle, min_ttc):
        msg_angle = Float32()
        msg_angle.data = float(angle)
        self.angle_pub.publish(msg_angle)
        
        msg_ttc = Float32()
        msg_ttc.data = float(min_ttc)
        self.ttc_pub.publish(msg_ttc)

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