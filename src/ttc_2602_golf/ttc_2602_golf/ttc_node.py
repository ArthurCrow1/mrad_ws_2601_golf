import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float32
import numpy as np

class TTCNode(Node):
    def __init__(self):
        super().__init__('ttc_node')

        # tiempo de activacion 
        self.declare_parameter('ttc_threshold', 1.0)
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel_joy', self.cmd_vel_callback, 10)
        self.brake_pub = self.create_publisher(TwistStamped, '/cmd_vel_ttc', 10)
        
        self.prev_ranges = None
        self.prev_time = 0.0
        self.danger_front = False
        
        self.aeb_locked = False 
        
        self.get_logger().info(f"TTC  initialized (Threshold: {self.ttc_threshold}s)")

    def scan_callback(self, msg):
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        # obtener rango
        current_ranges = np.array(msg.ranges)
        current_ranges = np.where(np.isinf(current_ranges) | np.isnan(current_ranges), 0.0, current_ranges)
        
        if self.prev_ranges is None:
            self.prev_ranges = current_ranges
            self.prev_time = current_time
            return

        # derivada del rango    
        dt = current_time - self.prev_time
        if dt <= 0: return
            
        range_rate = (current_ranges - self.prev_ranges) / dt
        ttc = np.full_like(current_ranges, np.inf)

        # solo si la distancia se reducce a 0.05 m/s
        approaching_mask = range_rate < -0.05 
        valid_ranges_mask = current_ranges > 0.0
        compute_mask = approaching_mask & valid_ranges_mask

        # aplicar la formula
        ttc[compute_mask] = current_ranges[compute_mask] / -range_rate[compute_mask]

        # tiempo minimo, peligro mas inminente
        min_ttc_value = np.min(ttc)

        # comparar el min con el limite 
        if min_ttc_value < self.ttc_threshold:
            self.danger_front = True
        else:
            self.danger_front = False
            
        self.prev_ranges = current_ranges
        self.prev_time = current_time

    def cmd_vel_callback(self, msg):
        # accion del mando
        user_pushing_forward = msg.twist.linear.x > 0.0
        user_released_or_reverse = msg.twist.linear.x <= 0.0
        
        # quitar seguro
        if user_released_or_reverse:
            self.aeb_locked = False
            
        # Si hay peligro real detectado por el láser, activamos el seguro
        if self.danger_front:
            self.aeb_locked = True
            
        # Si el seguro está puesto y se insiste en ir hacia adelante, bloquear
        if self.aeb_locked and user_pushing_forward:
            brake_msg = TwistStamped()
            brake_msg.header = msg.header
            brake_msg.twist.linear.x = 0.0
            brake_msg.twist.angular.z = 0.0
            
            self.brake_pub.publish(brake_msg)
            self.get_logger().warn("Bloqueado, soltar mando", throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = TTCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()