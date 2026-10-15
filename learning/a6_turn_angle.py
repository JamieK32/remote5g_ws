
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


TARGET_TURN_DEG = 30.0  # 目标：车头向左转30度
DRIVE_SPEED = 0.12     # 行驶速度，m/s
STEERING_ANGLE = 0.25  # 前轮目标转角，rad


# 将四元数转换成Yaw角（弧度）
def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# 计算两个朝向之间的角度差
def angle_difference(current, start):
    delta = current - start
    return math.atan2(math.sin(delta), math.cos(delta))


# 发布车辆控制指令
def send_command(node, publisher, speed, steering):
    msg = AckermannDriveStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering
    publisher.publish(msg)


def main():
    rclpy.init()

    node = rclpy.create_node("a6_heading_driver")

    publisher = node.create_publisher(
        AckermannDriveStamped, "/ackermann_cmd", 1
    )

    # 保存当前朝向和最近一次收到里程计的时间
    state = {
        "yaw": None,
        "last_odom": 0.0
    }

    # 里程计回调函数
    def on_odom(msg):
        q = msg.pose.pose.orientation
        state["yaw"] = yaw_from_quaternion(q)
        state["last_odom"] = time.monotonic()

    subscriber = node.create_subscription(
        Odometry, "/odom", on_odom, 10
    )

    start_yaw = None

    try:
        print("等待 /odom 和车辆控制器连接...")

        wait_end = time.monotonic() + 10.0

        while rclpy.ok() and time.monotonic() < wait_end:
            rclpy.spin_once(node, timeout_sec=0.05)

            if (state["yaw"] is not None
                    and publisher.get_subscription_count() > 0):
                break

        if (state["yaw"] is None
                or publisher.get_subscription_count() == 0):
            print("连接失败，请检查Gazebo和ROS话题")
            return

        # 记录小车当前朝向
        start_yaw = state["yaw"]

        print(
            f"初始朝向：{math.degrees(start_yaw):.2f}度"
        )
        print(f"目标：向左转{TARGET_TURN_DEG}度")

        drive_end = time.monotonic() + 12.0
        last_print = 0.0

        # 根据朝向反馈持续控制车辆
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            now = time.monotonic()

            # 检查里程计是否断流
            if now - state["last_odom"] > 1.0:
                print("里程计超时，停车")
                break

            # 计算已经转过多少度
            turned_rad = angle_difference(
                state["yaw"], start_yaw
            )
            turned_deg = math.degrees(turned_rad)

            # 到达目标角度
            if turned_deg >= TARGET_TURN_DEG:
                print("达到目标朝向，开始停车")
                break

            # 安全超时
            if now >= drive_end:
                print("转弯超时，停车")
                break

            # 继续向左转弯
            send_command(
                node,
                publisher,
                DRIVE_SPEED,
                STEERING_ANGLE
            )

            if now - last_print > 0.5:
                print(f"已经向左转：{turned_deg:.2f}度")
                last_print = now

    except KeyboardInterrupt:
        print("用户中断，停车")

    finally:
        # 连续发送停车命令
        if rclpy.ok():
            for _ in range(10):
                send_command(node, publisher, 0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)

        # 计算停车阶段结束时的实际转角
        if start_yaw is not None and state["yaw"] is not None:
            final_turn = math.degrees(
                angle_difference(state["yaw"], start_yaw)
            )
            print(
                f"停车阶段结束时航向变化：{final_turn:.2f}度"
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
