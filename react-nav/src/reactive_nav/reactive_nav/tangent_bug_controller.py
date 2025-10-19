import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from std_msgs.msg import String

LIDAR_SCANNING_ANGLE_DEG = 180.0
LIDAR_RESOLUTION_POINTS = 180
GOAL_POSITION = np.array([4.0, 4.0])

class TangentBugState:
    GO_TO_GOAL = 0
    BOUNDARY_FOLLOW = 1
    GOAL_REACHED = 2

class TangentBugController(Node):
    def __init__(self):
        super().__init__('tangent_bug_controller')
        self.get_logger().info('Controlador Tangent Bug iniciado!')
        
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.get_logger().info('Conectado ao CoppeliaSim!')

        try:
            self.robot_handle = self.sim.getObject('/turtlebot3_base_link')
            self.left_motor_handle = self.sim.getObject('/wheel_left_joint')
            self.right_motor_handle = self.sim.getObject('/wheel_right_joint')
        except Exception as e:
            self.get_logger().error(f'Erro ao obter handles: {e}')
            rclpy.shutdown(); return

        self.pose_publisher = self.create_publisher(Pose, '/robot_pose', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/robot_state', 10)

        self.current_state = TangentBugState.GO_TO_GOAL
        self.goal_position = GOAL_POSITION
        self.lidar_angle_range = LIDAR_SCANNING_ANGLE_DEG
        self.lidar_resolution = LIDAR_RESOLUTION_POINTS
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def get_lidar_data(self):
        try:
            result, data = self.sim.callScriptFunction('getLidarData', self.robot_handle)
            return np.array(data) if result == self.sim.script_function_result_ok else None
        except Exception as e:
            self.get_logger().warn(f"Não foi possível ler o LIDAR: {e}")
            return None

    def control_loop(self):
        if self.current_state == TangentBugState.GOAL_REACHED:
            return

        robot_pos_xyz = self.sim.getObjectPosition(self.robot_handle, -1)
        robot_quat_xyzw = self.sim.getObjectQuaternion(self.robot_handle, -1)
        current_pos = np.array([robot_pos_xyz[0], robot_pos_xyz[1]])

        dist_to_goal = np.linalg.norm(self.goal_position - current_pos)
        goal_threshold = 0.1

        if dist_to_goal < goal_threshold:
            self.get_logger().info(f'Objetivo alcançado! Distância: {dist_to_goal:.2f}m. Parando o robô.')
            self.current_state = TangentBugState.GOAL_REACHED
            self.state_publisher.publish(String(data="GOAL_REACHED"))
            self.set_robot_velocity(0.0, 0.0)
            self.timer.cancel()
            return

        matrix = self.sim.getObjectMatrix(self.robot_handle, -1)
        forward_vector = np.array([matrix[0], matrix[4]])
        current_yaw = math.atan2(forward_vector[1], forward_vector[0])

        lidar_data = self.get_lidar_data()
        if lidar_data is None or len(lidar_data) == 0:
            self.get_logger().warn("Aguardando dados válidos do LIDAR...")
            self.set_robot_velocity(0.0, 0.0); return

        pose_msg = Pose()
        pose_msg.position = Point(x=robot_pos_xyz[0], y=robot_pos_xyz[1], z=robot_pos_xyz[2])
        pose_msg.orientation = Quaternion(x=robot_quat_xyzw[0], y=robot_quat_xyzw[1], z=robot_quat_xyzw[2], w=robot_quat_xyzw[3])
        self.pose_publisher.publish(pose_msg)
        
        if self.current_state == TangentBugState.GO_TO_GOAL:
            self.go_to_goal_logic(current_pos, current_yaw, lidar_data)
        elif self.current_state == TangentBugState.BOUNDARY_FOLLOW:
            self.boundary_follow_logic(current_pos, current_yaw, lidar_data)

    def go_to_goal_logic(self, current_pos, current_yaw, lidar_data):
        self.state_publisher.publish(String(data="GO_TO_GOAL"))
        direction_to_goal = self.goal_position - current_pos
        angle_to_goal = math.atan2(direction_to_goal[1], direction_to_goal[0])
        
        if not self.is_path_clear(lidar_data, angle_to_goal, current_yaw):
            self.get_logger().info('Caminho bloqueado. Mudando para BOUNDARY_FOLLOW.')
            self.current_state = TangentBugState.BOUNDARY_FOLLOW
            return
        
        angle_error = self.normalize_angle(angle_to_goal - current_yaw)
        Kp = 1.5
        angular_vel = Kp * angle_error
        linear_vel = 0.2 * max(0, 1 - 2 * abs(angle_error))
        self.set_robot_velocity(linear_vel, angular_vel)

    def boundary_follow_logic(self, current_pos, current_yaw, lidar_data):
        self.state_publisher.publish(String(data="BOUNDARY_FOLLOW"))
        direction_to_goal = self.goal_position - current_pos
        angle_to_goal = math.atan2(direction_to_goal[1], direction_to_goal[0])

        if self.is_path_clear(lidar_data, angle_to_goal, current_yaw):
            self.get_logger().info('Caminho livre! Retornando para GO_TO_GOAL.')
            self.current_state = TangentBugState.GO_TO_GOAL
            return

        min_dist_index = np.argmin(lidar_data)
        min_dist_angle = self.index_to_angle(min_dist_index)
        angle_error = self.normalize_angle(min_dist_angle - (-math.pi / 2))
        
        if abs(min_dist_angle) < math.radians(45):
             angular_vel = 0.6
        else:
             angular_vel = 0.5 * angle_error
        linear_vel = 0.1
        self.set_robot_velocity(linear_vel, angular_vel)

    def set_robot_velocity(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_vel)
        twist_msg.angular.z = float(angular_vel)
        self.twist_publisher.publish(twist_msg)

        wheel_radius, wheel_base = 0.033, 0.287
        v_left = (linear_vel - angular_vel * wheel_base / 2.0) / wheel_radius
        v_right = (linear_vel + angular_vel * wheel_base / 2.0) / wheel_radius
        self.sim.setJointTargetVelocity(self.left_motor_handle, v_left)
        self.sim.setJointTargetVelocity(self.right_motor_handle, v_right)

    def is_path_clear(self, lidar_data, target_angle_world, current_yaw):
        target_angle_robot = self.normalize_angle(target_angle_world - current_yaw)
        target_index = self.angle_to_index(target_angle_robot)
        for i in range(max(0, target_index - 5), min(len(lidar_data), target_index + 6)):
            if lidar_data[i] < 0.6: return False
        return True

    def angle_to_index(self, angle_robot):
        if self.lidar_angle_range == 0: return 0
        return int((angle_robot + math.radians(self.lidar_angle_range / 2)) / math.radians(self.lidar_angle_range) * (self.lidar_resolution - 1))

    def index_to_angle(self, index):
        if self.lidar_resolution <= 1: return 0
        return (index / (self.lidar_resolution - 1) * math.radians(self.lidar_angle_range)) - math.radians(self.lidar_angle_range / 2)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    controller = TangentBugController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok() and hasattr(controller, 'client'):
            controller.set_robot_velocity(0.0, 0.0)
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()