# 单模块学习目录

此目录用于小型练习和独立算法实验，不承载完整自动驾驶运行入口。

当前练习：

- `a4_sequence.py`：有限动作序列。
- `a5_distance.py`：基于里程计的距离控制。
- `a6_turn_angle.py`：转角/航向练习。
- `lane_curve_filter.py`：单帧直线过滤实验；它不是当前实战巡线的车道选择器。
- `test_lane_curve_filter.py`：该单模块实验的离线检查。

运动练习会发送车辆命令，运行前查看参数并使用自己的仿真实例。

## 实战算法入口

原 `b1_lane_follow.py`、地面车道跟踪、Gazebo 起点复位、驾驶验收工具与对应测试，
已迁移到 **`src/remote5g_autonomy/`**，并拆成感知、控制、ROS 适配、显示和仿真工具。
详细结构见 [实战包说明](../src/remote5g_autonomy/README.md)。

从工作区根目录编译并加载环境后运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to remote5g_autonomy --symlink-install
source install/setup.bash
ros2 run remote5g_autonomy lane_follow --filter-straight --show
```

以目标 0.7m/s 连续运行、退出后 Gazebo 复位：

```bash
ros2 run remote5g_autonomy lane_follow --drive --speed 0.7 --continuous --filter-straight --show --reset-on-exit
```

终端的 ROS_DOMAIN_ID 应与仿真一致。旧的 `python3 learning/b1_lane_follow.py` 入口已移除，
实战包不反向导入学习目录。后续在本目录练习单个函数，再把验证过的能力集成到实战包。
