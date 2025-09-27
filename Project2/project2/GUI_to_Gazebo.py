import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class Joint_State_Publisher_GUI_to_Gazebo(Node):
    def __init__(self):
        super().__init__('Joint_State_Publisher_GUI_to_Gazebo')
        
        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        

        self.publisher_arm_controller = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.publisher_gripper_controller = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

    def joint_state_callback(self, msg: JointState):
        arm_controller_data = JointTrajectory()
        gripper_controller_data = JointTrajectory()

        arm_controller_data.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        gripper_controller_data.joint_names = [
            'rg2_finger_joint1',
            'rg2_finger_joint2'
        ]

        # Arm point
        arm_point = JointTrajectoryPoint()
        arm_point.positions = list(msg.position[:6])
        arm_point.time_from_start.sec = 1

        # Gripper point
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = list(msg.position[6:])
        gripper_point.time_from_start.sec = 1

        arm_controller_data.points.append(arm_point)
        gripper_controller_data.points.append(gripper_point)

        self.publisher_arm_controller.publish(arm_controller_data)
        self.publisher_gripper_controller.publish(gripper_controller_data)


def main(args = None):
    rclpy.init(args=args)
    GUI_to_Gazebo = Joint_State_Publisher_GUI_to_Gazebo()
    rclpy.spin(GUI_to_Gazebo)

    GUI_to_Gazebo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()