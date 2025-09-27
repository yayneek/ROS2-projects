from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import math
import random
import os


def generate_launch_description():
    package_share = get_package_share_directory('project2')


    # === Gazebo ===
    default_world = os.path.join(package_share, 'worlds', 'empty.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    world = LaunchConfiguration(
        'world'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ), 
        launch_arguments={
            'gz_args': ['-r ',world]
        }.items()
    )

    # Robot model spawn:
    robot_model_sdf = os.path.join(package_share,'models','ur5_rg2','model.sdf')
    robot_model_urdf = os.path.join(package_share,'models','ur5_rg2','model_minimal.urdf')
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
    # Spawning Lidars:
    # Y:
    lidar_y_urdf = os.path.join(package_share, 'sensors', 'lidar_y.sdf')
    lidar_y_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_camera',
        output = 'screen',
        arguments=[
            '-file', lidar_y_urdf,
            '-name','lidar_y',
            '-world','empty',
            '-x','0.75',
            '-y', '0.5',
            '-z','0.1'
        ]
    )
    static_transform_lidar_y_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.75", "0.5", "0.1", "3.14159265359", "0", "0",  "Base", "lidar_y/lidar_link/lidar_sensor"],
        name="static_transform_lidar_y",
        output ="screen"
    )
    # X:
    lidar_x_urdf = os.path.join(package_share, 'sensors', 'lidar_x.sdf')
    lidar_x_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_camera',
        output = 'screen',
        arguments=[
            '-file', lidar_x_urdf,
            '-name','lidar_x',
            '-world','empty',
            '-x','0.0',
            '-y','1.0',
            '-z','0.1',
        ]
    )
    static_transform_lidar_x_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "1", "0.1", "-1.57079632679", "0", "0",  "Base", "lidar_x/lidar_link/lidar_sensor"],
        name='static_transform_lidar_x',
        output ="screen"
    )

    # Cube model spawn:
    cube_model_sdf = os.path.join(package_share,'models','other','cube.sdf')
    randomize_cube_position_x = random.uniform(-0.2, 0.1)
    randomize_cube_position_y = random.uniform(0.25, 0.5)
    randomize_cube_angle = random.uniform(0, math.pi)
    cube_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cube',
        output ='screen',
        arguments=[
            '-name', 'cube',
            '-file', cube_model_sdf,
            '-x', str(randomize_cube_position_x),
            '-y', str(randomize_cube_position_y),
            '-z', '0.1',
            '-Y', str(randomize_cube_angle),
            '-world','empty'
        ]
    )

    # Goal_plane spawn:
    plane_model_sdf = os.path.join(package_share, 'models','other','goal_plane.sdf')
    plane_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_plane',
        output="screen",
        arguments=[
            '-name','goal_plane',
            '-file',plane_model_sdf,
            '-x', '0.0',
            '-y','-0.5',
            '-z','0.01',
            '-world','empty'            
        ]
    )

    # joint_state_publisher:
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # robot_state_publisher:
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': open(robot_model_urdf).read()}
        ],
        output="screen"
    )

    # Rviz node:
    rviz_config_path = os.path.join(package_share,'rviz','urdf.rviz')
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

    # gz to ros bridge:
    bridge_params = os.path.join(package_share, 'config','gz_bridge.yaml')
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
            ]
    )


    return LaunchDescription([
        joint_state_publisher_node,
        world_arg,
        rviz_arg,
        gz_sim_launch,        
        robot_spawn_node,
        cube_spawn_node,
        lidar_x_spawn_node,
        lidar_y_spawn_node,
        static_transform_lidar_x_node,
        static_transform_lidar_y_node,
        plane_spawn_node,
        rviz_node,
        robot_state_publisher_node,
        ros_gz_bridge_node,
    ])