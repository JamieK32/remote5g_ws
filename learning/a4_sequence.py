
import time
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


# 向车辆发送一条控制指令
def send_command(node, publisher, speed, angle):
    msg = AckermannDriveStamped()

    msg.header.stamp = node.get_clock().now().to_msg()
    msg.drive.speed = speed
    msg.drive.steering_angle = angle

    publisher.publish(msg)


# 持续控制车辆一段时间
def drive_for(node, publisher, speed, angle, seconds):
    end_time = time.monotonic() + seconds

    while rclpy.ok() and time.monotonic() < end_time:
        send_command(node, publisher, speed, angle)
        time.sleep(0.05)


def main():
    # 1. 初始化ROS2
    rclpy.init()

    # 2. 创建ROS2节点
    node = rclpy.create_node("a4_drive_sequence")

    # 3. 创建控制指令发布者
    publisher = node.create_publisher(
        AckermannDriveStamped,
        "/ackermann_cmd",
        1
    )

    try:
        # 等待Gazebo车辆控制器连接
        deadline = time.monotonic() + 10.0
        while (rclpy.ok()
               and publisher.get_subscription_count() == 0
               and time.monotonic() < deadline):
            time.sleep(0.05)

        if publisher.get_subscription_count() == 0:
            print("没有找到车辆控制器，请检查Gazebo")
            return

        # 4. 第一阶段：直线前进2秒
        print("第一阶段：直线前进")
        drive_for(node, publisher, 0.10, 0.0, 2.0)

        # 5. 第二阶段：向左转弯3秒
        print("第二阶段：向左转弯")
        drive_for(node, publisher, 0.10, -0.25, 3.0)

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        # 6. 发送停车指令
        if rclpy.ok():
            print("第三阶段：停车")
            for i in range(5):
                send_command(node, publisher, 0.0, 0.0)
                time.sleep(0.05)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
