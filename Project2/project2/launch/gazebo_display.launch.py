from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # === Gazebo ===
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ), 
        launch_arguments={
            'gz_args': 'empty.sdf'
        }.items()
    )

    # Robot model spawn:
    robot_model_sdf = os.path.join('/home/jan/ros2_jazzy_ws/src/project2/models/ur5_rg2','model.sdf')
    robot_model_urdf = os.path.join('/home/jan/ros2_jazzy_ws/src/project2/models/ur5_rg2','model_minimal.urdf')
    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output = 'screen',
        arguments = [
            '-name', 'ur5_rg2',
            '-file', robot_model_sdf,
            '-world', 'empty'
        ]
    )

    # Cube model spawn:
    cube_model_sdf = os.path.join('/home/jan/ros2_jazzy_ws/src/project2/models/cube', 'model.sdf')
    cube_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cube',
        output ='screen',
        arguments=[
            '-name', 'cube',
            '-file', cube_model_sdf,
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.05',
            '-world','empty'
        ]
    )

    # joint_state_publisher:
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[robot_model_sdf]
    )

    # robot_state_publisher:
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[robot_model_urdf],
        output="screen"
    )

    # Rviz node:
    rviz_config_path = os.path.join('/home/jan/ros2_jazzy_ws/src/project2/rviz/urdf.rviz')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(rviz_config_path),
        description='Path to rviz config file'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )

    # Load controllers node:
    load_controllers_node = Node(
        
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_spawn_node,
        cube_spawn_node,
        robot_state_publisher_node,
        #joint_state_publisher_node,
        rviz_arg,
        rviz_node,
    ])