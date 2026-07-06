from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    world_file = PathJoinSubstitution([
        FindPackageShare('drone_system'),
        'worlds',
        'follow_world.world',
    ])
    car_model_file = PathJoinSubstitution([
        FindPackageShare('drone_system'),
        'models',
        'car',
        'model.sdf',
    ])
    drone_model_file = PathJoinSubstitution([
        FindPackageShare('drone_system'),
        'models',
        'drone',
        'model.sdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('drone_system'),
                'config',
                'params.yaml',
            ]),
            description='Path to the ROS 2 parameters file.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])),
            launch_arguments={
                'world': world_file,
                'verbose': 'true',
            }.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'car', '-file', car_model_file, '-x', '12.0', '-y', '0.0', '-z', '0.05'],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'drone', '-file', drone_model_file, '-x', '6.0', '-y', '0.0', '-z', '0.25'],
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='drone_system',
                    executable='car_simulator',
                    name='car_simulator',
                    parameters=[params_file],
                    output='screen',
                ),
                Node(
                    package='drone_system',
                    executable='follower_node',
                    name='follower_node',
                    parameters=[params_file],
                    output='screen',
                ),
                Node(
                    package='drone_system',
                    executable='drone_manager',
                    name='drone_manager',
                    parameters=[params_file],
                    output='screen',
                ),
                Node(
                    package='drone_system',
                    executable='health_monitor',
                    name='health_monitor',
                    parameters=[params_file],
                    output='screen',
                ),
            ],
        ),
    ])
