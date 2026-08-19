#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # 1. Declarar los parámetros que exige el profesor
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('max_vel', 0.8)
        
        # Límite físico
        self.max_steering_angle = 0.5
        self.min_vel = 0.3 # Velocidad mínima en curvas para no detenerse por completo
        
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        self.error_sub = self.create_subscription(Float32, '/error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)

    def error_callback(self, msg):
        # Leer parámetros actualizados
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        max_vel = self.get_parameter('max_vel').value

        current_error = msg.data
        current_time = self.get_clock().now()
        dt_duration = current_time - self.prev_time
        dt = dt_duration.nanoseconds * 1e-9
        
        if dt <= 0.0: return
            
        error_derivative = (current_error - self.prev_error) / dt
        
        # --- CÁLCULO DE DIRECCIÓN (STEERING) ---
        steering_correction = (kp * current_error) + (kd * error_derivative)
        
        # Saturación de dirección
        if steering_correction > self.max_steering_angle:
            steering_correction = self.max_steering_angle
        elif steering_correction < -self.max_steering_angle:
            steering_correction = -self.max_steering_angle
            
        # --- CÁLCULO DINÁMICO DE VELOCIDAD (Requisito del profesor) ---
        # Calculamos qué tan fuerte estamos girando (de 0.0 a 1.0)
        turn_intensity = abs(steering_correction) / self.max_steering_angle
        
        # Si turn_intensity es 0 (recta), la velocidad es max_vel.
        # Si turn_intensity es 1 (curva máxima), la velocidad baja a min_vel.
        dynamic_velocity = max_vel - (turn_intensity * (max_vel - self.min_vel))
            
        # --- PUBLICAR COMANDO ---
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = current_time.to_msg()
        cmd_msg.header.frame_id = 'base_link'
        
        cmd_msg.twist.linear.x = float(dynamic_velocity)
        cmd_msg.twist.angular.z = float(steering_correction)
        
        self.cmd_pub.publish(cmd_msg)
        
        self.prev_error = current_error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()