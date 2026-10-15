
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


TARGET_DISTANCE = 0.50  # 目标距离：米
DRIVE_SPEED = 0.12     # 目标速度：米/秒


# 发布阿克曼控制指令
def send_command(node, publisher, speed):
    msg = AckermannDriveStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.drive.speed = speed
    msg.drive.steering_angle = 90.0
    publisher.publish(msg)


def main():
    rclpy.init()

    node = rclpy.create_node("a5_distance_driver")

    publisher = node.create_publisher(
        AckermannDriveStamped, "/ackermann_cmd", 1
    )

    # 保存最新位置和消息接收时间
    state = {"position": None, "last_odom": 0.0}

    # 收到 /odom 消息时自动调用
    def on_odom(msg):
        p = msg.pose.pose.position
        state["position"] = (p.x, p.y)
        state["last_odom"] = time.monotonic()

    subscriber = node.create_subscription(
        Odometry, "/odom", on_odom, 10
    )

    start_position = None

    try:
        print("等待 /odom 和车辆控制器连接...")

        wait_end = time.monotonic() + 10.0

        while rclpy.ok() and time.monotonic() < wait_end:
            rclpy.spin_once(node, timeout_sec=0.05)

            if (state["position"] is not None
                    and publisher.get_subscription_count() > 0):
                break

        if (state["position"] is None
                or publisher.get_subscription_count() == 0):
            print("连接失败，请检查Gazebo和ROS话题")
            return

        # 记录起点
        start_position = state["position"]
        x0, y0 = start_position
        print(f"起点：x={x0:.3f}, y={y0:.3f}")

        drive_end = time.monotonic() + 20.0
        last_print = 0.0

        # 不断读取位置并控制小车
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # 里程计断流保护
            if time.monotonic() - state["last_odom"] > 1.0:
                print("超过1秒未收到里程计，停车")
                break

            x, y = state["position"]

            # 计算从起点到当前位置的距离
            distance = math.hypot(x - x0, y - y0)

            if distance >= TARGET_DISTANCE:
                print("到达目标距离，开始停车")
                break

            if time.monotonic() >= drive_end:
                print("运行超时，安全停车")
                break

            # 尚未到达目标，继续前进
            send_command(node, publisher, DRIVE_SPEED)

            if time.monotonic() - last_print > 0.5:
                print(f"已移动：{distance:.3f} 米")
                last_print = time.monotonic()

    except KeyboardInterrupt:
        print("人工中断，停车")

    finally:
        # 连续发送停车命令
        if rclpy.ok():
            for _ in range(8):
                send_command(node, publisher, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)

        if start_position is not None and state["position"] is not None:
            x, y = state["position"]
            x0, y0 = start_position
            final_distance = math.hypot(x - x0, y - y0)
            print(f"停车阶段结束时位移：{final_distance:.3f} 米")

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
