#!/usr/bin/env python3
import numpy as np
import math
import warnings
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
import tf2_ros

# ================= FASE 1: FISICA =================
class DifferentialDriveKinematics:
    def __init__(self, dt: float):
        self.dt = dt

    def predict_state(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        x, y, theta = state
        v, w = control

        next_x = x + v * math.cos(theta) * self.dt
        next_y = y + v * math.sin(theta) * self.dt
        next_theta = theta + w * self.dt
        next_theta = math.atan2(math.sin(next_theta), math.cos(next_theta))

        return np.array([next_x, next_y, next_theta])

    def simulate_trajectory(self, initial_state: np.ndarray, control_sequence: np.ndarray) -> np.ndarray:
        N = control_sequence.shape[0]
        trajectory = np.zeros((N + 1, 3))
        trajectory[0] = initial_state

        for k in range(N):
            trajectory[k + 1] = self.predict_state(trajectory[k], control_sequence[k])
            
        return trajectory

# ================= FASES 2 y 3: EL MPC =================
class MPCController:
    def __init__(self, dt: float, N: int, v_ref: float, weights: list, bounds: dict):
        self.kinematics = DifferentialDriveKinematics(dt)
        self.N = N            
        self.v_ref = v_ref    
        
        self.W1, self.W2, self.W3, self.W4 = weights
        self.v_min, self.v_max = bounds['v']
        self.w_min, self.w_max = bounds['w']
        self.bounds_seq = [(self.v_min, self.v_max), (self.w_min, self.w_max)] * self.N

    def mpc_cost_function(self, control_sequence_flat, initial_state, reference_trajectory, prev_control):
        N = len(control_sequence_flat) // 2
        controls = control_sequence_flat.reshape((N, 2))
        
        predicted_states = self.kinematics.simulate_trajectory(initial_state, controls)[1:] 
        
        pred_x, pred_y = predicted_states[:, 0], predicted_states[:, 1]
        ref_x, ref_y = reference_trajectory[:, 0], reference_trajectory[:, 1]
        v, w = controls[:, 0], controls[:, 1]
        
        J1 = np.sum((pred_x - ref_x)**2 + (pred_y - ref_y)**2)
        
        v_extended = np.insert(v, 0, prev_control[0])
        w_extended = np.insert(w, 0, prev_control[1])
        J2 = np.sum(np.diff(v_extended)**2 + np.diff(w_extended)**2)
        
        J3 = np.sum(w**2)
        J4 = np.sum((v - self.v_ref)**2)
        
        return (self.W1 * J1) + (self.W2 * J2) + (self.W3 * J3) + (self.W4 * J4)

    def solve_mpc(self, initial_state, reference_trajectory, prev_control):
        initial_guess = np.tile(prev_control, self.N) 
        options = {'maxiter': 50, 'ftol': 1e-3}
        
        try:
            result = minimize(
                fun=self.mpc_cost_function,
                x0=initial_guess,
                args=(initial_state, reference_trajectory, prev_control),
                method='SLSQP',
                bounds=self.bounds_seq,
                options=options
            )
            
            # Validacion de seguridad y logs visibles en consola
            if not result.success or np.isnan(result.x[0]) or np.isnan(result.x[1]):
                print(f"[ALERTA MPC] Solver fallo. Exito: {result.success} | Motivo: {result.message}")
                safe_u = prev_control * 0.8 
                return safe_u, np.tile(safe_u, (self.N, 1))
            
            optimal_controls = result.x.reshape((self.N, 2))
            v_est, w_est = optimal_controls[0]
            
            return np.array([v_est, w_est]), optimal_controls
            
        except Exception as e:
            print(f"[ERROR CRITICO] Fallo en la libreria SciPy: {e}")
            safe_u = np.array([0.0, 0.0])
            return safe_u, np.tile(safe_u, (self.N, 1))

# ================= FASE 4: NODO ROS 2 =================
class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_pt_2602_golf')
        
        self.declare_parameter('N', 10)               
        self.declare_parameter('dt', 0.05)            
        self.declare_parameter('v_ref', 0.5)          
        
        self.declare_parameter('w_pos', 10.0)         
        self.declare_parameter('w_smooth', 1.0)       
        self.declare_parameter('w_straight', 0.5)     
        self.declare_parameter('w_vel', 2.0)          
        
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('w_max', 1.5)
        
        N = self.get_parameter('N').value
        dt = self.get_parameter('dt').value
        v_ref = self.get_parameter('v_ref').value
        weights = [
            self.get_parameter('w_pos').value,
            self.get_parameter('w_smooth').value,
            self.get_parameter('w_straight').value,
            self.get_parameter('w_vel').value
        ]
        bounds = {
            'v': (0.0, self.get_parameter('v_max').value), 
            'w': (-self.get_parameter('w_max').value, self.get_parameter('w_max').value)
        }
        
        self.mpc = MPCController(dt, N, v_ref, weights, bounds)
        
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        
        self.local_path_pub = self.create_publisher(Path, '/mpc_predicted_path', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.global_path = []
        self.prev_control = np.array([0.0, 0.0]) 
        self.current_target_idx = 0
        
        self.timer = self.create_timer(dt, self.control_loop)
        self.get_logger().info("Controlador MPC inicializado (Version Base). Esperando ruta...")

    def path_cb(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if new_path == self.global_path:
            return
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_target_idx = 0  

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return np.array([x, y, yaw])
        except Exception:
            return None

    def extract_local_reference(self, robot_state):
        if not self.global_path:
            return None    
        rx, ry = robot_state[0], robot_state[1] 
        
        search_window = 30
        max_idx = min(self.current_target_idx + search_window, len(self.global_path))
        window_path = self.global_path[self.current_target_idx:max_idx]
        
        distances = [math.hypot(px - rx, py - ry) for px, py in window_path]
        local_closest_idx = np.argmin(distances)
        
        self.current_target_idx += local_closest_idx
        
        N = self.get_parameter('N').value
        local_ref = []
        
        for i in range(N):
            # Version original, sin path_step
            idx = min(self.current_target_idx + i, len(self.global_path) - 1)
            local_ref.append(self.global_path[idx])

        # print(f'target actual{self.current_target_idx}')

        return np.array(local_ref)

    def control_loop(self):
        if not self.global_path:
            return 
            
        robot_state = self.get_robot_pose()
        if robot_state is None:
            return
            
        ref_trajectory = self.extract_local_reference(robot_state)
        
        dist_to_final_goal = math.hypot(self.global_path[-1][0] - robot_state[0], self.global_path[-1][1] - robot_state[1])
        if dist_to_final_goal < 0.3 and self.current_target_idx >= len(self.global_path) - self.get_parameter('N').value - 10: 
            self.stop_robot()
            self.global_path = []
            self.get_logger().info("Carrera finalizada (MPC).")
            return

        optimal_u, optimal_controls = self.mpc.solve_mpc(robot_state, ref_trajectory, self.prev_control)
        
        v_opt, w_opt = optimal_u
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = float(v_opt)
        msg.twist.angular.z = float(w_opt)
        
        self.cmd_pub.publish(msg)
        
        predicted_states = self.mpc.kinematics.simulate_trajectory(robot_state, optimal_controls)
        
        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for state in predicted_states:
            p = PoseStamped()
            p.header = local_path_msg.header
            p.pose.position.x = float(state[0])
            p.pose.position.y = float(state[1])
            local_path_msg.poses.append(p)
            
        self.local_path_pub.publish(local_path_msg)
        
        self.prev_control = optimal_u

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()