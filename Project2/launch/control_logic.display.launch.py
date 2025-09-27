import rclpy
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('project2')


    # Spawning controllers for arm and gripper:
    arm_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_arm_controller',
        arguments=['arm_controller']
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        arguments=['gripper_controller']
    )

    # lidar to pose node:
    lidar_to_pose_node = Node(
        package='project2',
        executable='lidar_to_pose',
        name='lidar_to_pose',
    )

    # gui to gazebo node:
    gui_to_gazebo_node = Node(
        package='project2',
        executable='gui_to_gazebo',
        name='gui_to_gazebo'
    )

    return LaunchDescription([
        arm_controller_spawn,
        gripper_controller_spawner,
        lidar_to_pose_node,
        gui_to_gazebo_node,
        
    ])