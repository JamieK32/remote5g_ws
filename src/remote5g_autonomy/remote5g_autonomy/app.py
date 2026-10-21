"""Application lifecycle: CLI, optional preview, startup snapshot and cleanup."""
from pathlib import Path # 构造配置文件路径
import sys # 获取终端运行参数
import time # 获取单调时钟时间

from ament_index_python.packages import get_package_share_directory
import rclpy # ROS2 Python客户端库
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args #将ROS专用参数与应用参数分开

from .errors import SimulationResetError
from .exit_signals import ExitSignals #捕获推出请求
from .options import parse_args
from .ros.commands import stop_vehicle
from .ros.lane_follower import LaneFollower #我的巡线ROS2节点
from .visualization.display import DebugDisplay #显示保存调试图像


def create_returner(node, entity):
    # Perception/control and ordinary preview do not import Gazebo services.
    from .simulation.gazebo_start_pose import GazeboStartPose
    return GazeboStartPose(node, entity)


def run(options, ros_args=None):
    options.validate()
    with ExitSignals() as signals:
        return _run(options, ros_args, signals)


def _run(options, ros_args, signals):
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    display = None
    returner = None
    exit_code = 0
    try:
        node = LaneFollower(options)
        display = DebugDisplay(options.show, options.save_dir)
        if options.reset_on_exit:
            node.timer.cancel()
            returner = create_returner(node, options.vehicle_entity)
            returner.capture()
            node.controller.arm_after_snapshot(time.monotonic())
            node.timer.reset()
        mode = 'DRIVE' if options.drive else 'PREVIEW ONLY (NO VEHICLE COMMANDS)'
        node.get_logger().info(f'{mode}: waiting for /camera/front/image_raw')
        if options.drive:
            node.get_logger().info(f'Target speed: {options.speed:.2f} m/s')
            if options.continuous:
                node.get_logger().info('Continuous mode: no time/distance limit; stop and wait on '
                                       'lane/sensor loss, resume on recovery; Ctrl+C or q to exit')
        if options.filter_straight:
            node.get_logger().info('Ground lane tracker enabled: GREEN=selected lane borders, '
                                   'CYAN=center, RED=straight candidates; waiting for camera calibration')
        while rclpy.ok() and not node.controller.done and not signals.requested:
            rclpy.spin_once(node, timeout_sec=0.02)
            if display.update(node.debug):
                break
        if signals.requested:
            node.get_logger().info('Exit requested; stopping and cleaning up')
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Interrupted by user')
    except SimulationResetError as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        try:
            if node is not None:
                node.controller.done = True
                node.timer.cancel()
                if options.drive and rclpy.ok():
                    stop_vehicle(node, 0.6 if returner is not None else 0.2)
                    if returner is not None and returner.initial_pose is not None:
                        try:
                            returner.restore()
                        except SimulationResetError as exc:
                            node.get_logger().error(f'Return to start failed: {exc}')
                            exit_code = 1
                        stop_vehicle(node, 0.3)
        finally:
            try:
                if display is not None:
                    display.close()
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    return exit_code


def main(args=None):
    argv = list(sys.argv if args is None else ['lane_follow', *args])
    default = Path(get_package_share_directory('remote5g_autonomy'))/'config/lane_follow.yaml'
    options = parse_args(remove_ros_args(args=argv)[1:], default_config=str(default))
    raise SystemExit(run(options, ros_args=argv))
