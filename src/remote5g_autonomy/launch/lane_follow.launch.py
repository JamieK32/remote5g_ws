"""Start the lane application against an already-running simulator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BOOL_OPTIONS = ('drive', 'show', 'filter_straight', 'continuous', 'reset_on_exit')
VALUE_OPTIONS = ('speed', 'lookahead', 'lane_width', 'max_steer', 'seconds', 'max_distance',
                 'save_dir', 'vehicle_config', 'vehicle_entity')


def start(context):
    arguments = ['--config', LaunchConfiguration('config').perform(context)]
    for option in BOOL_OPTIONS:
        value = LaunchConfiguration(option).perform(context).lower()
        if not value:
            continue
        if value not in ('true', 'false'):
            raise ValueError(f'{option} must be true or false')
        arguments.append(('--' if value == 'true' else '--no-')+option.replace('_', '-'))
    for option in VALUE_OPTIONS:
        value = LaunchConfiguration(option).perform(context)
        if value:
            arguments.extend(['--'+option.replace('_', '-'), value])
    # The application owns Ctrl+C shutdown so it can stop and optionally restore
    # the vehicle before destruction. Allow service timeouts before escalation.
    return [Node(package='remote5g_autonomy', executable='lane_follow', output='screen',
                 arguments=arguments, sigterm_timeout='35', sigkill_timeout='5')]


def generate_launch_description():
    config = Path(get_package_share_directory('remote5g_autonomy'))/'config/lane_follow.yaml'
    declarations = [DeclareLaunchArgument('config', default_value=str(config))]
    declarations += [DeclareLaunchArgument(name, default_value='') for name in BOOL_OPTIONS+VALUE_OPTIONS]
    return LaunchDescription(declarations+[OpaqueFunction(function=start)])
