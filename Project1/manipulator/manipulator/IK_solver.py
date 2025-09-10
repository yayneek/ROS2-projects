import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation as Rot
from manipulator import poly_trajectory

class IK_solver(Node):
    
    def __init__(self):
        #Name of the node
        super().__init__('IK_solver') 

        #Inverse kinematics params:
        self.dt = 0.01
        self.i = 0
        self.tk = 3
        self.goal_position = np.zeros(6)
        self.q_traj = np.zeros(6)
        self.time = 0 
        self.clicked = False

        #Subscriber:
        self.subscription = self.create_subscription(PointStamped,'/clicked_point', self.IK_callback, 10)

        #Publisher:
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        #What I want to publish:
        self.joint_states = JointState()
        self.gripper_state = JointState()

        self.joint_states.name = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
        self.gripper_state.name = ['Left_gripper_joint']

        #Initial values:
        self.gripper_state.position = np.zeros(1)
        self.joint_states.position = np.zeros(6)


        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.gripper_state.header.stamp = self.get_clock().now().to_msg()
        if self.clicked == True:
            self.joint_states.position = self.q_traj[self.i, :]
            self.i = self.i + 1

            if self.i >= int(self.tk/self.dt):
                self.get_logger().info('Task complete!')
                self.clicked = False

        
        self.publisher.publish(self.joint_states)
        self.publisher.publish(self.gripper_state)
        








    def IK_callback(self, data):
        # Rotation matrices:
        def R_z(theta):
            return np.array([[np.cos(theta), -1* np.sin(theta),  0], 
                            [np.sin(theta),     np.cos(theta),  0], 
                            [0,                 0,              1]])
        def R_x(theta):
            return np.array([[1,                 0,              0], 
                            [0, np.cos(theta), -1 * np.sin(theta)], 
                            [0, np.sin(theta),      np.cos(theta)]])
        def R_y(theta):
            return np.array([[np.cos(theta),     0,  np.sin(theta)],
                            [0,                  1,              0],
                            [-np.sin(theta),     0,  np.cos(theta)]])
        def orientation_error(R_des, R_cur):
            R_err = R_des @ R_cur.T   # relative rotation
            rotvec = Rot.from_matrix(R_err).as_rotvec()  # 3-vector
            return rotvec
        def matrix_to_quaternions(R):
            return Rot.from_matrix(R).as_quat()
        
        # Desired coordinates of position and rotation of the end-effector:
        self.x = data.point.x
        self.y = data.point.y
        self.z = data.point.z
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        def FK_solution(theta):
            r_base = np.array([0, 0, 0.3])
            r1 = np.array([0.15,    0,      0.75])
            r2 = np.array([-0.15,   0.6,    0])
            r3 = np.array([0,       0,      0.15])
            r4 = np.array([-0.3,    0,      0.45])
            r5 = np.array([0,      -0.75,   0])
            gripper_claws = np.array([0,    -0.25,     0]) #Assuming end effector is the tip of the gripper claw

            #Link1:
            #link_1_position =  r_base
            #link_1_orientation = matrix_to_quaternions(R_z(theta[0]))


            
            #Link2:
            #link_2_position = r_base + np.dot(R_z(theta[0]), r1)
            #link_2_orientation = matrix_to_quaternions(np.dot(R_z(theta[0]), R_x(theta[1])))


            
            #Link3:
            #link_3_position = r_base + np.dot(R_z(theta[0]), r1 + np.dot(R_x(theta[1]), r2))
            #link_3_orientation = matrix_to_quaternions(np.dot(np.dot(R_z(theta[0]), R_x(theta[1])), R_x(theta[2])))



            #Link4:
            #link_4_position = r_base + np.dot(R_z(theta[0]), r1 + np.dot(R_x(theta[1]), r2 + np.dot(R_x(theta[2]), r3)))
            #link_4_orientation = matrix_to_quaternions(np.dot(np.dot(np.dot(R_z(theta[0]), R_x(theta[1])), R_x(theta[2])), R_z(theta[3])))


            #Link5:
            #link_5_position = r_base + np.dot(R_z(theta[0]), r1 + np.dot(R_x(theta[1]), r2 + np.dot(R_x(theta[2]), r3 + np.dot(R_z(theta[3]), r4))))
            #link_5_orientation = matrix_to_quaternions(np.dot(np.dot(np.dot(np.dot(R_z(theta[0]), R_x(theta[1])), R_x(theta[2])), R_z(theta[3])), R_x(theta[4])))



            #Link6:
            gripper_base_position = r_base + np.dot(R_z(theta[0]), r1 + np.dot(R_x(theta[1]), r2 + np.dot(R_x(theta[2]), r3 + np.dot(R_z(theta[3]), r4 + np.dot(R_x(theta[4]), r5)))))
            gripper_base_orientation = np.dot(np.dot(np.dot(np.dot(np.dot(R_z(theta[0]), R_x(theta[1])), R_x(theta[2])), R_z(theta[3])), R_x(theta[4])), R_y(theta[5]))



            #End_effector:
            end_effector_position = r_base + np.dot(R_z(theta[0]), r1 + np.dot(R_x(theta[1]), r2 + np.dot(R_x(theta[2]), r3 + np.dot(R_z(theta[3]), r4 + np.dot(R_x(theta[4]), r5 + np.dot(R_y(theta[5]), gripper_claws))))))

            clicked_point_position = np.array([self.x, self.y, self.z])
            desired_orientation = Rot.from_quat(np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])).as_matrix()

            pos_error= clicked_point_position - end_effector_position
            ori_error = orientation_error(desired_orientation, gripper_base_orientation)
            
            solution = np.hstack((pos_error, ori_error))
            return solution
        

        #Solution of inverse kinematics:
        initial_guess = np.zeros(6)
        self.goal_position = fsolve(FK_solution, initial_guess)
        self.q_traj, _, _, _ = poly_trajectory(self.joint_states.position, self.goal_position, self.time, self.tk, self.dt)
        
        
        self.clicked = True
        self.i = 0 # Reset of the iteration number in trajectro
        




def main(args = None):
    rclpy.init(args=args)
    clicked_point_subscriber = IK_solver()
    rclpy.spin(clicked_point_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()