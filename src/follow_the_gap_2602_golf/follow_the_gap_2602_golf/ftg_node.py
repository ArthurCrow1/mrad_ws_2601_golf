#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np
import math

class FTGNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        
        self.declare_parameter('bubble_radius', 0.28)  # radio obstaculo #0.25
        self.declare_parameter('max_vel', 2.0)        # Velocida max, 2.0 
        self.declare_parameter('kp', 2.0)              
        self.declare_parameter('kd', 0.7)            
        
        self.prev_angle = 0.0 # memoria para D
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ftg', 10)
        
        #self.get_logger().info("FTG iniciado")

    def scan_callback(self, msg):

        bubble_radius = self.get_parameter('bubble_radius').value
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        max_vel = self.get_parameter('max_vel').value
        
        # datos lidar
        ranges = np.array(msg.ranges)
        ranges[~np.isfinite(ranges)] = 10.0     #reorganizacion para inf
        
        # reescribir angulos a 0 (180 frente)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        front_mask = (angles > -math.pi/2) & (angles < math.pi/2)
        ranges[~front_mask] = 0.0 
        # con el cambio por jorge esto se puede quitar
        
        # dado el range, encontrar el mas cercano
        valid_ranges = np.where(ranges > 0.0, ranges, np.inf)
        closest_idx = np.argmin(valid_ranges)
        closest_dist = valid_ranges[closest_idx]
        
        if not math.isinf(closest_dist):
            # angulo burbuja, evitar esas zonas
            theta_bubble = math.asin(min(bubble_radius / closest_dist, 1.0))
            num_indices_bubble = int(theta_bubble / msg.angle_increment) # cambio de grados a rayos

            min_idx = max(0, closest_idx - num_indices_bubble)
            max_idx = min(len(ranges) - 1, closest_idx + num_indices_bubble)
            
            ranges[min_idx:max_idx + 1] = 0.0
        
        # rayos validos y saber los gaps con astype T o F
        is_free = ranges > 0.0
        diff = np.diff(is_free.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0] + 1
        
        if is_free[0]:
            starts = np.insert(starts, 0, 0)
        if is_free[-1]:
            ends = np.append(ends, len(ranges))
            
        if len(starts) == 0:
            self.publish_cmd(0.0, 0.0)
            return
            
        # seleccionar el camino
        best_score = -1
        best_gap_idx = 0
        
        for i in range(len(starts)):
            s = starts[i]
            e = ends[i]
            gap_width_rays = e - s
            if gap_width_rays < 5:
                continue
            # ancho del gap, dado ley cose
            r1 = ranges[s]
            r2 = ranges[e - 1]
            gap_angle = gap_width_rays * msg.angle_increment
            physical_width = math.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * math.cos(gap_angle))            
            # promedio dist de los rayos
            gap_depth = np.mean(ranges[s:e])
            # Puntaje por Área Libre
            score = physical_width * gap_depth
            
            if score > best_score:
                best_score = score
                best_gap_idx = i
        # tomar la mejor ruta
        # cambio paradicma, el centro, no el mas lejano.
        best_start = starts[best_gap_idx]
        best_end = ends[best_gap_idx]
        center_idx = int((best_start + best_end - 1) / 2)
        target_angle = msg.angle_min + (center_idx * msg.angle_increment)
        
        # PD
        dt = 0.05  # 20 hz del lidar
        derivative = (target_angle - self.prev_angle) / dt
        steering = (kp * target_angle) + (kd * derivative)
        self.prev_angle = target_angle
        # restriccion para giros bruscos
        steering = max(-2.5, min(2.5, steering))        
        abs_angle = abs(target_angle)

        # cambio de v, basado en la alineacion
        if abs_angle > 1.0: 
            angle_penalty = 0.0     #mucho desfase, no lieal, solo direccion. 
        else:
            angle_penalty = 1.0 - (abs_angle / 0.8) #lineal o curvas peq
            
        linear_vel = max_vel * angle_penalty
        self.publish_cmd(linear_vel, steering)

    def publish_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FTGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()