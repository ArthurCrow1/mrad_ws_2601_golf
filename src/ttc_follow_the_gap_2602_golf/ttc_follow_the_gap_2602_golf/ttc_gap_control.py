#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class TTCGapControl(Node):
    def __init__(self):
        super().__init__('ttc_gap_control')
        
        self.declare_parameter('v_max', 1.2) 
        self.declare_parameter('kp_steering', 1.5) 
        self.declare_parameter('kd_steering', 0.5) 
        self.declare_parameter('ttc_recta', 0.5)
        self.declare_parameter('ttc_curva', 1.5)
        self.target_angle = 0.0
        self.current_ttc = float('inf')
        self.prev_angle = 0.0
        
        self.angle_sub = self.create_subscription(Float32, '/ttc_target_angle', self.angle_callback, 10)
        self.ttc_sub = self.create_subscription(Float32, '/ttc_min_val', self.ttc_callback, 10)  
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ftg', 10)       
        self.timer = self.create_timer(0.05, self.control_loop)
        
        #self.get_logger().info("FTG+TTC control iniciado")

    def angle_callback(self, msg):
        self.target_angle = msg.data

    def ttc_callback(self, msg):
        self.current_ttc = msg.data

    def control_loop(self):
        v_max = self.get_parameter('v_max').value
        kp = self.get_parameter('kp_steering').value
        kd = self.get_parameter('kd_steering').value
        # limites de ttc
        ttc_recta = self.get_parameter('ttc_recta').value
        ttc_curva = self.get_parameter('ttc_curva').value
        
        dt = 0.05
        derivative = (self.target_angle - self.prev_angle) / dt
        steering = (kp * self.target_angle) + (kd * derivative)
        self.prev_angle = self.target_angle
        # limitacion de giros
        steering = max(-2.5, min(2.5, steering))
        # dado el giro se considera si es curva o no; si angle_ratio > 0.5 es curva, si no recta
        abs_angle = abs(self.target_angle)
        angle_ratio = min(1.0, abs_angle / 0.5)
        dynamic_ttc_min = ttc_recta + angle_ratio * (ttc_curva - ttc_recta) #ttc dinamico cambio en recta o en curva, dado el angle ratio
        # margen de seguridad, evitar valores muy pequenos
        safe_ttc = dynamic_ttc_min * 1.5 
        
        if self.current_ttc == 0.0:
            base_vel = 0.0 
        elif self.current_ttc >= safe_ttc:
            base_vel = v_max
        else:
            urgency = (self.current_ttc - dynamic_ttc_min) / (safe_ttc - dynamic_ttc_min)
            base_vel = max(0.1, v_max * urgency)

        # para la angular
        if abs_angle > 0.5:
            angle_penalty = 0.0
        else:
            angle_penalty = 1.0 - (abs_angle / 0.5)
        
        linear_vel = base_vel * angle_penalty
            
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_vel)
        cmd.twist.angular.z = float(steering)
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TTCGapControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()