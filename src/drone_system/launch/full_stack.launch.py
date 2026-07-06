from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    artifacts_root = LaunchConfiguration('artifacts_root')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    package_share = Path(get_package_share_directory('drone_system'))
    repo_root = Path(get_package_prefix('drone_system')).parents[1]
    venv_activate = repo_root / '.venv' / 'bin' / 'activate'
    px4_root = repo_root / 'external' / 'PX4-Autopilot'
    world_file = package_share / 'worlds' / 'follow_world.world'
    car_model_file = package_share / 'models' / 'car' / 'model.sdf'

    if headless:
        display_cmd = 'export HEADLESS=1 && unset GAZEBO_MASTER_URI && '
    else:
        display_cmd = 'unset HEADLESS && unset GAZEBO_MASTER_URI && '

    px4_command = (
        'pkill -x px4 || true && '
        'pkill -x mavsdk_server || true && '
        'pkill -x gzserver || true && '
        'pkill -x gzclient || true && '
        f'source "{venv_activate}" && '
        'source /opt/ros/humble/setup.bash && '
        'export ROS_VERSION=2 && '
        f'{display_cmd}'
        'export PX4_NO_FOLLOW_MODE=1 && '
        f'export PX4_SITL_WORLD="{world_file}" && '
        'make px4_sitl gazebo-classic_iris'
    )

    return [
        ExecuteProcess(
            cmd=['bash', '-lc', px4_command],
            cwd=str(px4_root),
            output='screen',
            on_exit=Shutdown(),
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'car', '-file', str(car_model_file), '-x', '12.0', '-y', '0.0', '-z', '0.05'],
                    output='screen',
                ),
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
                    executable='px4_manager',
                    name='px4_manager',
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
                Node(
                    package='drone_system',
                    executable='telemetry_recorder',
                    name='telemetry_recorder',
                    parameters=[params_file, {'output_root': artifacts_root}],
                    output='screen',
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('drone_system'))
    repo_root = Path(get_package_prefix('drone_system')).parents[1]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(package_share / 'config' / 'params.yaml'),
            description='Path to the ROS 2 parameters file.',
        ),
        DeclareLaunchArgument(
            'artifacts_root',
            default_value=str(repo_root / 'artifacts'),
            description='Directory where telemetry and derived artifacts are written.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without a GUI (set true for CI/Docker).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
