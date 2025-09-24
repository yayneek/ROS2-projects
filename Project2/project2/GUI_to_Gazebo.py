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
        

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def joint_state_callback(self, msg: JointState):
        to_pass = JointTrajectory()
        to_pass.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = list(msg.position[:6])
        point.time_from_start.sec = 1

        to_pass.points.append(point)
        self.publisher.publish(to_pass)

def main(args = None):
    rclpy.init(args=args)
    GUI_to_Gazebo = Joint_State_Publisher_GUI_to_Gazebo()
    rclpy.spin(GUI_to_Gazebo)

    GUI_to_Gazebo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()