import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np

class Bug2State:
    GO_TO_GOAL = 0
    WALL_FOLLOW = 1
    GOAL_REACHED = 2

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        self.get_logger().info('Conectando ao CoppeliaSim...')
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.get_logger().info('Conectado!')

        try:
            self.robot_handle = self.sim.getObject('/turtlebot3_base_link')
            self.left_motor_handle = self.sim.getObject('/wheel_left_joint')
            self.right_motor_handle = self.sim.getObject('/wheel_right_joint')
            self.front_sensor_handle = self.sim.getObject('/scan') 
            self.right_sensor_handle = self.sim.getObject('/scan_right')
        except Exception as e:
            self.get_logger().error(f'Erro ao obter handles dos objetos: {e}')
            rclpy.shutdown()
            return

        self.current_state = Bug2State.GO_TO_GOAL
        self.goal_position = np.array([0.925, -1.450])

        self.start_point = np.array(self.sim.getObjectPosition(self.robot_handle, -1)[:2])
        self.m_line_start = self.start_point
        self.m_line_end = self.goal_position

        self.hit_point = None
        self.dist_at_hit_point = float('inf')

        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        if self.current_state == Bug2State.GOAL_REACHED:
            return

        robot_pos_xyz = self.sim.getObjectPosition(self.robot_handle, -1)
        current_pos = np.array([robot_pos_xyz[0], robot_pos_xyz[1]])

        dist_to_goal = np.linalg.norm(self.goal_position - current_pos)
        goal_threshold = 0.1

        if dist_to_goal < goal_threshold:
            self.get_logger().info(f'Objetivo alcançado! Distância: {dist_to_goal:.2f}m. Parando o robô.')
            self.current_state = Bug2State.GOAL_REACHED
            self.set_robot_velocity(0.0, 0.0)
            self.timer.cancel()
            return

        matrix = self.sim.getObjectMatrix(self.robot_handle, -1)
        forward_vector = np.array([matrix[0], matrix[4]])
        current_yaw = math.atan2(forward_vector[1], forward_vector[0])

        res_front, dist_front, _, _, _ = self.sim.readProximitySensor(self.front_sensor_handle)
        res_right, dist_right, _, _, _ = self.sim.readProximitySensor(self.right_sensor_handle)
        is_obstacle_ahead = (res_front > 0 and dist_front < 0.4)

        if self.current_state == Bug2State.GO_TO_GOAL:
            self.go_to_goal_logic(current_pos, current_yaw, is_obstacle_ahead)
        elif self.current_state == Bug2State.WALL_FOLLOW:
            self.wall_follow_logic(current_pos, is_obstacle_ahead, res_right, dist_right)

    def go_to_goal_logic(self, current_pos, current_yaw, is_obstacle_ahead):
        if is_obstacle_ahead:
            self.get_logger().info('Obstáculo detectado! Mudando para WALL_FOLLOW.')
            self.current_state = Bug2State.WALL_FOLLOW
            self.hit_point = current_pos
            self.dist_at_hit_point = np.linalg.norm(self.goal_position - self.hit_point)
            return

        direction_to_goal = self.goal_position - current_pos
        angle_to_goal = math.atan2(direction_to_goal[1], direction_to_goal[0])
        angle_error = self.normalize_angle(angle_to_goal - current_yaw)

        Kp = 1.5 
        angular_vel = Kp * angle_error
        linear_vel = 0.2 * max(0, 1 - 2 * abs(angle_error))
        
        self.set_robot_velocity(linear_vel, angular_vel)

    def wall_follow_logic(self, current_pos, is_obstacle_ahead, res_right, dist_right):
        dist_from_m_line = self.distance_point_to_line(current_pos, self.m_line_start, self.m_line_end)
        on_m_line = dist_from_m_line < 0.1
        
        current_dist_to_goal = np.linalg.norm(self.goal_position - current_pos)
        is_closer_than_hit = current_dist_to_goal < self.dist_at_hit_point

        if on_m_line and is_closer_than_hit and self.hit_point is not None:
            self.get_logger().info('De volta à linha M e mais perto do objetivo! Mudando para GO_TO_GOAL.')
            self.current_state = Bug2State.GO_TO_GOAL
            self.hit_point = None
            return

        desired_dist = 0.35
        if is_obstacle_ahead:
            linear_vel = 0.05
            angular_vel = 0.8
        else:
            linear_vel = 0.15
            if res_right == 0:
                angular_vel = -0.6
            else:
                error_dist = desired_dist - dist_right
                angular_vel = error_dist * 2.0 
        
        self.set_robot_velocity(linear_vel, angular_vel)

    def set_robot_velocity(self, linear_vel, angular_vel):
        wheel_radius = 0.033
        wheel_base = 0.287
        v_left = (linear_vel - angular_vel * wheel_base / 2.0) / wheel_radius
        v_right = (linear_vel + angular_vel * wheel_base / 2.0) / wheel_radius
        self.sim.setJointTargetVelocity(self.left_motor_handle, v_left)
        self.sim.setJointTargetVelocity(self.right_motor_handle, v_right)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def distance_point_to_line(self, point, line_start, line_end):
        line_vec = line_end - line_start
        point_vec = point - line_start
        line_len_sq = np.dot(line_vec, line_vec)
        if line_len_sq == 0.0: return np.linalg.norm(point_vec)
        t = max(0, min(1, np.dot(point_vec, line_vec) / line_len_sq))
        projection = line_start + t * line_vec
        return np.linalg.norm(point - projection)

def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Controller()
    try:
        rclpy.spin(bug2_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and hasattr(bug2_controller, 'client') and bug2_controller.client.is_connected():
            bug2_controller.set_robot_velocity(0.0, 0.0)
            bug2_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()