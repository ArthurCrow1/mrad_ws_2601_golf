import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class LKPNode(Node):
    def __init__(self):
        super().__init__('lkp_node')
        self.bridge = CvBridge()
        
        # parametro de kp
        self.kp = 0.005 

        # al topico de la camara
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # al joy
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel_joy', self.joy_callback, 10)
        
        # twist
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel_lka', 10)
        
        self.current_linear_x = 0.0
        self.user_turning = False
        
        self.get_logger().info("LKP Node initialized")

    def joy_callback(self, msg):
        # velocidad lineal de
        self.current_linear_x = msg.twist.linear.x
        # control manual, apagar lkp
        self.user_turning = abs(msg.twist.angular.z) > 0.1

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        height, width, _ = cv_image.shape
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # evitar el horizonte
        mask = np.zeros_like(edges)
        polygon = np.array([[(0, height), (width, height), (width, int(height*0.6)), (0, int(height*0.6))]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=20)

        left_lines = []
        right_lines = []

        debug_image = np.copy(cv_image)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                if x1 == x2: continue
                
                slope = (y2 - y1) / (x2 - x1)
                
                # ignorar lineas horizontales
                if abs(slope) < 0.5:
                    continue 

                if slope < 0:
                    left_lines.append(line[0])
                    cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 4) # azul izquierda
                else:
                    right_lines.append(line[0])
                    cv2.line(debug_image, (x1, y1), (x2, y2), (0, 0, 255), 4) # rojo derecha


        center_x = width // 2 # Centro del carril/pantalla
        lane_center = center_x 
        
        # dado el caso, si es izquierda o derecha
        if len(left_lines) > 0 and len(right_lines) > 0:
            left_avg_x = np.mean([ (l[0] + l[2])/2 for l in left_lines ])
            right_avg_x = np.mean([ (l[0] + l[2])/2 for l in right_lines ])
            lane_center = int((left_avg_x + right_avg_x) / 2)
            
        elif len(left_lines) > 0:
            left_avg_x = np.mean([ (l[0] + l[2])/2 for l in left_lines ])
            lane_center = int(left_avg_x + 300)
            
        elif len(right_lines) > 0:
            right_avg_x = np.mean([ (l[0] + l[2])/2 for l in right_lines ])
            lane_center = int(right_avg_x - 300)

        # centro 
        cv2.circle(debug_image, (center_x, height - 50), 5, (0, 255, 0), -1) 
        cv2.circle(debug_image, (lane_center, height - 50), 5, (0, 255, 255), -1)
        cv2.line(debug_image, (center_x, height - 50), (lane_center, height - 50), (255, 255, 255), 2)

        # desviacion en pixeles
        error = center_x - lane_center 
        
        # correcion si se esta acelerando y no girando
        if self.current_linear_x > 0.0 and not self.user_turning:
            lka_msg = TwistStamped()
            lka_msg.header = msg.header
            lka_msg.twist.linear.x = self.current_linear_x 
            
            # correccion en base a kp 
            lka_msg.twist.angular.z = float(self.kp * error) 
            
            self.cmd_vel_pub.publish(lka_msg)

        cv2.imshow("LKP - Control Pipeline", debug_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LKPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()