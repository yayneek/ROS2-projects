import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PointStamped
from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Trigger
import math
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import fsolve
from project2 import polynomial_trajectory

class robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        # Subscriptions:
        self.subscriber_lidar_pose = self.create_subscription(
            Pose2D, 
            '/position_from_lidar',
            self.subscription_callback,
            10
        )

        # Publishers:
        self.publisher = self.create_publisher(
            JointState, 
            '/joint_states',
            10
        )

        # Services:
        self.ready_srv = self.create_service(
            Trigger, 
            'set_robot_pose_ready',
            self.set_pose_ready_callback
        )
        self.place_srv = self.create_service(
            Trigger,
            'set_robot_pose_place',
            self.set_pose_place_callback
        )

        # Poses of the robot:
        # Ready:
        self.position_ready = np.array([0, 0.5, 0.5])
        self.orientation_ready = Rot.from_quat(np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])).as_matrix()
        
        # Place:
        self.position_place = np.array([0, -0.5, 0.2])
        self.orientation_place = Rot.from_quat(np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])).as_matrix()


        # JointState
        self.joint_states = JointState()
        self.joint_states.name = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                                'wrist_1_joint','wrist_2_joint','wrist_3_joint','rg2_finger_joint1','rg2_finger_joint2']
        self.joint_states.position = [0.0]*8  # lista
        self.joint_states.header.stamp = self.get_clock().now().to_msg()


        # Publikacja początkowa
        self.publisher.publish(self.joint_states)


    
        

    def ForwardKinematics(self, theta):
        theta = np.asarray(theta).ravel()
        if theta.size != 6:
            raise ValueError("theta must have length 6")

        def R_z(t):
            return np.array([[np.cos(t), -np.sin(t), 0],
                            [np.sin(t),  np.cos(t), 0],
                            [0,           0,        1]])
        def R_x(t):
            return np.array([[1, 0, 0],
                            [0, np.cos(t), -np.sin(t)],
                            [0, np.sin(t),  np.cos(t)]])
        def R_y(t):
            return np.array([[ np.cos(t), 0, np.sin(t)],
                            [0,          1, 0],
                            [-np.sin(t), 0, np.cos(t)]])

        # Arm links:
        shoulder_link = np.array([0.0, 0.0, 0.08915899693965912])
        upper_arm_link = np.array([0.0, 0.13585, 0.0])
        forearm_link = np.array([0.0, -0.1197, 0.425])
        wrist_1_link = np.array([0.0, 0.0, 0.39225])
        wrist_2_link = np.array([0.0, 0.093, 0.0])
        wrist_3_link = np.array([0.0, 0.0, 0.09465])
        rg2_hand_link = np.array([0.0, 0.0823, 0.0])
        rg2_finger_link = np.array([0.105, 0.017, 0])/2

        # Rotation matrices:
        R0 = R_z(theta[0])
        R1 = R_y(theta[1])
        R2 = R_y(theta[2])
        R3 = R_y(theta[3])
        R4 = R_z(theta[4])
        R5 = R_y(theta[5])

        # rg2 hand orientation
        orientation_mat = R0 @ R1 @ R2 @ R3 @ R4 @ R5 @ R_z(math.pi/2) @ R_x(math.pi/2)

        # Position of the following links:
        p = shoulder_link.copy()
        p = p + (R0 @ upper_arm_link)
        p = p + (R0 @ R1 @ forearm_link)
        p = p + (R0 @ R1 @ R2 @ wrist_1_link)
        p = p + (R0 @ R1 @ R2 @ R3 @ wrist_2_link)
        p = p + (R0 @ R1 @ R2 @ R3 @ R4 @ wrist_3_link)
        p = p + (R0 @ R1 @ R2 @ R3 @ R4 @ R5 @ rg2_hand_link)

        return p, orientation_mat

    def InerseKinematics(self, goal_position, goal_orientation, theta0):
        """
        goal_position: np.array(3,)
        goal_orientation: matrix(3x3)
        theta0: initial guess (6,)
        """
        def equations(theta):
            est_position, est_orientation = self.ForwardKinematics(theta)  
            def orientation_error_calc(R_des, R_cur):
                R_err = R_des @ R_cur.T   # relative rotation
                rotvec = Rot.from_matrix(R_err).as_rotvec()  # 3-vector
                return rotvec
            
            # position and orientation errors:
            position_error = goal_position - est_position
            orientation_error = orientation_error_calc(goal_orientation, est_orientation)
            

            return np.hstack([position_error, orientation_error])

        # fsolve czasami potrzebuje dobrego zgadu; spróbuj sensownego theta0 zamiast zeros
        solution = fsolve(equations, theta0, maxfev=2000)
        solution = solution % math.tau
        return solution

    def set_pose_ready_callback(self, request, response):
        initial_guess = np.zeros(6)
        joints_states = self.InerseKinematics(self.position_ready, self.orientation_ready, initial_guess)
        gripper_state = [0.0, 0.0]

        all_states = np.hstack([joints_states, gripper_state])

        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = all_states.tolist()

        self.publisher.publish(self.joint_states)

        response.success = True
        response.message = "Robot set to pose READY"
        return response


    def set_pose_place_callback(self, request, response):
        initial_guess = np.zeros(6)
        joints_states = self.InerseKinematics(self.position_place, self.orientation_place, initial_guess)
        gripper_state = [0.0, 0.0]

        all_states = np.hstack([joints_states, gripper_state])

        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = all_states.tolist()
        
        self.publisher.publish(self.joint_states)

        response.success = True
        response.message = "Robot set to pose READY"
        return response




    def subscription_callback(self,msg: Pose2D):
        self.target_lidar_pose = np.array([msg.x, msg.y, 0.1, 0, 0, msg.theta])

    def Open_gripper(self):
        return 0
    def Close_gripper(self):
        return 0

def main(args = None):
    rclpy.init(args=args)
    node = robot_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()