import os
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('remote5g_sim'))
    workspace = Path(LaunchConfiguration('workspace').perform(context))
    scene = LaunchConfiguration('scene').perform(context)
    if scene not in ('flat', 'red', 'blue'):
        raise ValueError('scene must be flat, red or blue')
    world = share / 'worlds/flat.world' if scene == 'flat' else workspace / f'simulation/worlds/athletics_400m_{scene}.world'
    config = LaunchConfiguration('config').perform(context)
    p = yaml.safe_load(Path(config).read_text())
    mappings = {'config': config}
    for key in ('front_width', 'front_height', 'gimbal_width', 'gimbal_height'):
        value = LaunchConfiguration(key).perform(context)
        mappings[key] = value or str(p[key])
    robot = xacro.process_file(str(share / 'urdf/car.xacro'), mappings=mappings).toxml()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    # Distinct simulation instance; existing track server uses 11365.
    env = [SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:' + LaunchConfiguration('port').perform(context)),
           SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'), SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
           SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(workspace / 'simulation/models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
           SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', get_package_prefix('remote5g_sim') + '/lib:/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))]
    if scene == 'flat':
        defaults = ('0', '0', '0')
    else:
        # Bottom straight of lane 1, away from the raised inner kerb.
        defaults = ('0', '-37.2', '0')
    xyz = []
    for key, default in zip(('x', 'y', 'yaw'), defaults):
        xyz.append(LaunchConfiguration(key).perform(context) or default)
    nodes = [ExecuteProcess(cmd=['gzserver', '--verbose', str(world), '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
             Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot, 'use_sim_time': True, 'publish_frequency': 50.0}], output='screen'),
             Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'xtnetrc', '-topic', 'robot_description', '-x', xyz[0], '-y', xyz[1], '-z', '0.006', '-Y', xyz[2], '-timeout', '60'], output='screen')]
    if gui:
        client = ['gzclient']
        capture = LaunchConfiguration('capture_dir').perform(context)
        if capture:
            Path(capture).mkdir(parents=True, exist_ok=True)
            env.append(SetEnvironmentVariable('XTNETRC_GUI_CAPTURE', str(Path(capture).resolve())))
            client += ['--gui-client-plugin', get_package_prefix('remote5g_sim') + '/lib/libremote5g_gui_capture.so']
        nodes.append(ExecuteProcess(cmd=client, output='screen'))
    return env + nodes


def generate_launch_description():
    share = Path(get_package_share_directory('remote5g_sim'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in {
        'scene':'flat','gui':'true','capture_dir':'','port':'11375','workspace':'/home/jamiek/remote5g_ws',
        'config':str(share/'config/vehicle.yaml'),'front_width':'','front_height':'',
        'gimbal_width':'','gimbal_height':'','x':'','y':'','yaw':''}.items()] + [OpaqueFunction(function=setup)])
