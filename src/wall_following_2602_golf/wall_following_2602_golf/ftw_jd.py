import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum, auto

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class Estado(Enum):
    FORWARD = auto()
    TURN_RIGHT = auto()
    TURN_LEFT = auto()
    ERROR = auto()

e = 0
e1 = 0 
e2 = 0
ti = 0.0
td = 0.9
kp=6
u1 = 0

T=0.06

max_rot = 5.0
class FollowWall(Node):
    def __init__(self):
        super().__init__('follow_wall_node')
        self.get_logger().info("Follow Wall Node has been started.")
        self.msg_vel=TwistStamped()
        self.samples=720
        self.min_range=-1.57079632679
        self.max_range=1.57079632679

        self.rate=self.samples/(self.max_range-self.min_range)
        self.tetha = 4*((self.max_range-self.min_range)/self.samples)
        self.fr_range=np.abs(round((self.min_range+np.deg2rad(55))*self.rate))
        self.fl_range=np.abs(round((self.max_range+np.deg2rad(55))*self.rate))
        self.f_range=np.abs(round((self.min_range-np.deg2rad(-5))*self.rate))
        self.width = round((np.deg2rad(5)-np.deg2rad(-5))*self.rate)

        self.right=np.zeros(5)
        self.right_front=np.zeros(self.width)
        self.front=np.zeros(self.width)
        self.left_front=np.zeros(self.right_front.shape)
        self.left=np.zeros(5)

        self.linear_x=0.0
        self.angular_z=0.0
        self.estado = Estado.FORWARD

        self.scan_sub=self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_sub_callback,
            10
        )

        self.cmd_pub=self.create_publisher(
            TwistStamped,
            'diffdrive_controller/cmd_vel',
            10
        )

        self.timer_pub=self.create_timer(
            0.05,  # 20 Hz
            self.timer_pub_callback
        )

        self.timer_state=self.create_timer(
            0.05,  # 10 Hz
            self.timer_state_callback
        )

        self.timer_controller=self.create_timer(
            0.05,
            self.controller_callback
        )

    def scan_sub_callback(self,msg):
        global e
        self.right=msg.ranges[0:5]
        self.right_front=msg.ranges[self.fr_range:self.fr_range+self.width]
        self.front=msg.ranges[self.f_range:self.f_range+self.width]
        self.left_front=msg.ranges[self.fl_range-self.width:self.fl_range]
        self.left=msg.ranges[self.samples-6:self.samples-1]
        
        if np.isfinite(self.right[0]) and np.isfinite(self.right[-1]):
            alpha = np.arctan((self.right[-1]*np.cos(self.tetha)-self.right[0])/(self.right[-1]*np.sin(self.tetha)))
            y = self.right[0]*np.cos(alpha)
            ba = self.linear_x*np.sin(alpha)*T
            e = (1.2 - y - ba)


        match self.estado:
            case Estado.FORWARD:
                self.linear_x=2.7
                # self.get_logger().info("adelante")

            case Estado.TURN_RIGHT:
                self.linear_x=2.6
                self.angular_z=-2.58
                # self.get_logger().info("right")
            case Estado.TURN_LEFT:
                self.linear_x=2.9
                self.angular_z=3.08
                # self.get_logger().info("left")
                # self.get_logger().info(f"frente izquierda: {self.left_front}\izquierda: {self.left}")
            case Estado.ERROR:
                self.linear_x=0.0
                self.angular_z=0.0
                # self.get_logger().info("error")
                # self.get_logger().info(f"frente derecha: {self.right_front}\nderecha: {self.right}")
                # self.get_logger().info(f"frente izquierda: {self.left_front}\izquierda: {self.left}")

        # print(f"frente_izquierda: {self.left_front}\nizquierda: {self.left}")
        # print(f"frente_derecha: {self.right_front}\nderecha: {self.right}")

    def timer_pub_callback(self):
        self.msg_vel.header.stamp = self.get_clock().now().to_msg()
        self.msg_vel.twist.linear.x=self.linear_x
        self.msg_vel.twist.angular.z=self.angular_z

        self.cmd_pub.publish(self.msg_vel)

    def timer_state_callback(self):
        global e1,e2,u1

        if(np.max(self.right_front)>np.max(self.left_front)):
            if ((np.max(self.right_front)>2.7 or np.max(self.right)>3.1)):
                self.estado=Estado.TURN_RIGHT
            elif ((np.min(self.front)>0.6 and np.max(self.front)<12) or ((np.min(self.left)<1.6 or np.min(self.right)<1.3) and (np.min(self.left_front)<1.5 or np.min(self.right_front)<1.5))):
                self.estado=Estado.FORWARD
            else:
                self.estado=Estado.ERROR
        elif(np.max(self.right_front)<np.max(self.left_front)):
            if (np.max(self.left_front)>2.6 or np.max(self.left)>3.5):
                self.estado=Estado.TURN_LEFT
            elif ((np.min(self.front)>0.6 and np.max(self.front)<12) or ((np.min(self.left)<1.6 or np.min(self.right)<1.3) and (np.min(self.left_front)<1.5 or np.min(self.right_front)<1.5))):
                self.estado=Estado.FORWARD
            else:
                self.estado=Estado.ERROR
            

        # print(f"estado: {self.estado}")

    def controller_callback(self):
        global e, e1, e2, u1, ti, td, kp,T,max_rot

        if self.estado == Estado.FORWARD:
            if ti == 0:
                q0 = kp*(1 + td/(T))
                q1 = -kp*(1 + 2*td/(T))
                q2 = kp*td/(T)  
            else:
                q0 = kp*(1 + T/ti + td/(T))
                q1 = -kp*(1 + 2*td/(T)-T/ti)
                q2 = kp*td/(T)
            u = (q0*e + q1*e1 + q2*e2)*0.01 + u1
            if u>1.0:
                u = 1.0
            elif u<-1.0:
                u=-1.0
            self.angular_z =u *max_rot

            
            e2 = e1
            e1 = e
            u1 = u
        elif self.estado != Estado.FORWARD:
            e1 = 0.0
            e2 = 0.0
            u1 = 0.0

        # print(f"control: {self.angular_z}\nerror: {e}")

def main(args=None):
    rclpy.init(args=args)

    follow_wall = FollowWall()

    rclpy.spin(follow_wall)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    FollowWall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()