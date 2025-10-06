import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PointStamped
from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Trigger
import math
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import fsolve
from project2 import polynomial_trajectory as PolyTraj
import matplotlib.pyplot as plt

class robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        # GLOBAL CONSTANTS:
        self.dt = 0.01 # dt of the simulation
        self.t = 3     # desired reaching time
        self.todo = False
        self.i = 0


        # JointState
        self.joint_state = JointState()
        self.joint_state.name = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                                'wrist_1_joint','wrist_2_joint','wrist_3_joint','rg2_finger_joint1','rg2_finger_joint2']
        self.joint_state.position = [0.0]*8  # lista
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.q = self.joint_state.position[:6]
        # ----------------------------------------------------------------------------
        # Timer:
        self.timer = self.create_timer(
            self.dt, 
            self.timer_callback
        )
        # ----------------------------------------------------------------------------
        # Subscriptions:
        self.subscriber_lidar_pose = self.create_subscription(
            Pose2D, 
            '/position_from_lidar',
            self.lidar_subscription_callback,
            10
        )
        self.subsciber_joint_state = self.create_subscription(
            JointState, 
            '/joint_state',
            self.joint_state_subscription_callback,
            10
        )

        # Publishers:
        self.publisher = self.create_publisher(
            JointState, 
            '/joint_states',
            10
        )
        # ---------------------------------------------------------------------------
        # Services:
        # Gripper:
        self.open_gripper_srv = self.create_service(
            Trigger,
            'set_robot_pose_gripper_open',
            self.open_gripper
        )
        self.close_gripper_srv = self.create_service(
            Trigger,
            'set_robot_pose_gripper_close',
            self.close_gripper
        )
        #
        # Arm:
        self.set_robot_pose_ready = self.create_service(
            Trigger,
            'set_robot_pose_ready',
            self.set_robot_pose_ready

        )
        self.set_robot_pose_place = self.create_service(
            Trigger,
            'set_robot_pose_place',
            self.set_robot_pose_place
        )
        self.set_robot_pose_up = self.create_service(
            Trigger,
            'set_robot_pose_up',
            self.set_robot_pose_up
        )
        self.set_robot_pose_pick = self.create_service(
            Trigger, 
            'set_robot_pose_pick',
            self.set_robot_pose_pick
        )
        # ----------------------------------------------------------------------------
        # Poses of the robot:
        # Ready:
        self.position_ready = np.array([0, 0.5, 0.5])
        self.orientation_ready = Rot.from_quat(np.array([0, np.sqrt(2)/2, 0, np.sqrt(2)/2])).as_matrix()
        
        # Place:
        self.position_place = np.array([0, -0.5, 0.7])
        self.orientation_place = Rot.from_quat(np.array([0, np.sqrt(2)/2, 0, np.sqrt(2)/2])).as_matrix()

        # Up:
        self.position_up = np.array([0, 0.19145, 1.0011])
        self.orientation_up = Rot.from_quat(np.array([0.5, 0.5, 0.5, 0.5])).as_matrix()
        # ----------------------------------------------------------------------------
        # Initial publication:
        self.publisher.publish(self.joint_state)  

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        if self.todo == True:
            self.joint_state.position = np.hstack([
                self.q[self.i, :],
                [self.joint_state.position[6], self.joint_state.position[7]]
                ])
            self.i = self.i + 1

            if self.i >= int(self.t/self.dt):
                self.get_logger().info('Task complete!')
                self.i = 0
                self.todo = False

        
        self.publisher.publish(self.joint_state)

    def forward_kinematics(self, theta):
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

    def inverse_kinematics(self, position, orientation, theta0):
        """
        goal_position: np.array(3,)
        goal_orientation: matrix(3x3)
        theta0: initial guess (6,)
        """
        def equations(theta):
            est_position, est_orientation = self.forward_kinematics(theta)  
            def orientation_error_calc(R_des, R_cur):
                R_err = R_des @ R_cur.T   # relative rotation
                rotvec = Rot.from_matrix(R_err).as_rotvec()  # 3-vector
                return rotvec
            
            # position and orientation errors:
            position_error = position - est_position
            orientation_error = orientation_error_calc(orientation, est_orientation)
            

            return np.hstack([position_error, orientation_error])

        solution = fsolve(equations, theta0, maxfev=2000)
        solution = solution % math.tau
        return solution

    def joint_trajectory(self, start_joint_config, goal_joint_config, t, dt):
        """
        start_position:         np.array    (3,),
        start_orientation:      matrix      (3,3),
        goal_position:          np.array    (3,),
        goal_orientation:       matrix      (3,3),

        t:              float, 
        dt:             float,

        return: q(6, t/dt)
        """
        # --- Minimizing angles:
        goal_joint_config = self.minimize_angle_difference(start_joint_config, goal_joint_config)
        q, v, a, time = PolyTraj.poly_trajectory(start_joint_config, goal_joint_config, 0, self.t, self.dt) 

        # --------------------------------------
        # plt.figure(figsize=(10,6))
        # for i in range(q.shape[1]):  # dla każdej współrzędnej q[i]
        #     plt.plot(time, q[:, i], label=f'q{i+1}')
        # plt.title("Trajektoria przegubów robota")
        # plt.xlabel("Czas [s]")
        # plt.ylabel("Kąt przegubu [rad]")
        # plt.legend()
        # plt.grid(True)
        # plt.tight_layout()
        # plt.show()
        # --------------------------------------




        return q

    def set_robot_pose_ready(self, request, response):
        start_joint_configuration = self.joint_state.position[:6]
        goal_joint_configuration = self.inverse_kinematics(self.position_ready, self.orientation_ready, start_joint_configuration)

        self.q = self.joint_trajectory(start_joint_configuration, goal_joint_configuration, self.t, self.dt)

        self.todo = True
        response.success = True
        response.message = 'Robot is set to pose READY'
        return response
    
    def set_robot_pose_place(self, request, response):
        start_joint_configuration = self.joint_state.position[:6]
        goal_joint_configuration = self.inverse_kinematics(self.position_place, self.orientation_place, start_joint_configuration)

        self.q = self.joint_trajectory(start_joint_configuration, goal_joint_configuration, self.t, self.dt)

        self.todo = True
        response.success = True
        response.message = 'Robot is set to pose PLACE'
        return response
    
    def set_robot_pose_pick(self, request, response):
        start_joint_configuration = self.joint_state.position[:6]

        # --- Gripper is pointing downwards with little rotation along Z axis to match cube orientation ---
        downwards_orientation = self.orientation_ready
        goal_orientation = self.target_lidar_orientation @ downwards_orientation

        goal_joint_configuration = self.inverse_kinematics(self.target_lidar_position,goal_orientation, start_joint_configuration)

        self.q = self.joint_trajectory(start_joint_configuration, goal_joint_configuration, self.t, self.dt)
        self.open_gripper(request, response)
        self.todo = True
        response.success = True
        response.message = 'Robot is set to pose PICK'
        return response

    def set_robot_pose_up(self, request, response):
        start_joint_configuration = self.joint_state.position[:6]
        goal_joint_configuration = np.zeros(6)

        self.q = self.joint_trajectory(start_joint_configuration, goal_joint_configuration, self.t, self.dt)

        self.todo = True
        response.success = True
        response.message = 'Robot is set to pose UP'
        return response

    def lidar_subscription_callback(self,msg: Pose2D):
        
        self.target_lidar_position = np.array([msg.x, msg.y, 0.35])
        self.target_lidar_orientation = Rot.from_rotvec([0,0,msg.theta + math.pi/4]).as_matrix()

    def joint_state_subscription_callback(self, msg: JointState):
        self.current_joint_state = msg  

    def open_gripper(self, request, response):
        gripper_state = [1.180, 1.180]
        joint_state = self.joint_state.position[:6]

        all_states = np.hstack([joint_state, gripper_state])

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = all_states.tolist()

        self.publisher.publish(self.joint_state)

        response.success = True
        response.message = "Gripper is opened"
        return response
    
    def close_gripper(self, request, response):
        gripper_state = [0.0, 0.0]
        joint_state = self.joint_state.position[:6]

        all_states = np.hstack([joint_state, gripper_state])

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = all_states.tolist()

        self.publisher.publish(self.joint_state)

        response.success = True
        response.message = "Gripper is closed"
        return response
    
    def minimize_angle_difference(self, current, target):
        """
        Dopasowuje kąty tak, aby przejście z current -> target
        wymagało minimalnego obrotu (ciągłość ruchu).
        """
        adjusted = np.copy(target)
        for i in range(len(target)):
            delta = target[i] - current[i]
            # sprowadź różnicę do zakresu (-pi, pi)
            delta = (delta + np.pi) % (2 * np.pi) - np.pi
            adjusted[i] = current[i] + delta
        return adjusted


def main(args = None):
    rclpy.init(args=args)
    node = robot_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()