# remote5g_autonomy：自动驾驶实战包

本包接收已运行仿真的相机、CameraInfo、里程计，执行视觉巡线。
由原 `learning/b1_lane_follow.py` 及相关模块迁移并拆分而来，运行时不依赖 `learning/` 或源码工作区的相对路径。
比赛状态机、完整避障、变道和停车尚未在此实现。

## 三个包的职责

| 包 | 职责 |
|---|---|
| `remote5g_sim` | 车辆、物理插件、传感器、车辆参数与基础仿真 |
| `remote5g_competition` | 比赛道具、赛道布局、场景生成和验收夹具 |
| **`remote5g_autonomy`** | **视觉识别、路径跟踪、驾驶应用；后续比赛任务算法在此扩展** |

## 编译和运行

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to remote5g_autonomy --symlink-install
source install/setup.bash
```

先启动原有仿真，再在 ROS_DOMAIN_ID 与仿真一致的终端启动巡线。
只预览，不发车辆指令：

```bash
ros2 run remote5g_autonomy lane_follow --filter-straight --show
```

以目标 0.7m/s 连续巡线，Ctrl+C / Q 停车并在 Gazebo 中复位：

```bash
ros2 run remote5g_autonomy lane_follow --drive --speed 0.7 --continuous --filter-straight --show --reset-on-exit
```

Launch 入口只启动算法，不重复启动 Gazebo：

```bash
ros2 launch remote5g_autonomy lane_follow.launch.py drive:=true speed:=0.7 continuous:=true filter_straight:=true show:=true reset_on_exit:=true
```

原有参数均保留，额外支持 `--config 配置文件`、`--vehicle-config 车辆YAML`，以及 `--no-drive` 等布尔参数关闭形式。
默认配置来自安装目录的 `config/lane_follow.yaml`；CLI 参数覆盖 YAML。
车辆配置默认通过 ROS 包索引读取 `remote5g_sim/config/vehicle.yaml`，可从任意当前目录启动。
配置中的相对 `vehicle_config` 按该配置文件所在目录解析。
ROS 原生重映射可以继续使用，例如 `--ros-args -r /camera/front/image_raw:=/other/image_raw`。

保留迁移前用户已设置的速度参数范围 `0 < speed <= 2.1`，默认仍为 0.10m/s。
当前车辆 YAML 的 `max_speed` 为 1.5m/s，物理插件还会按曲率限速；请求 2.1m/s 不代表实际能达到。
迁移不提高车辆速度上限，也不表示高速度已通过循线验收。
不加 `--continuous` 仍默认限制 15 秒和距起点 2 米；连续模式中丢线/数据过期停车等待，恢复后继续。
`--reset-on-exit` 是 Gazebo 调试复位，不是自主驶回，也不适用于正式比赛结束动作。

## 源码结构与数据流

```text
remote5g_autonomy/
  options.py                    CLI、YAML、参数验证
  control.py                    Pure Pursuit、像素转向、停车/恢复/限时策略（无ROS/OpenCV）
  perception/
    white_lines.py              白线分割、原像素检测器
    ground_lane_tracker.py      投影、边界拟合、车道配对与时序跟踪（不算转角、不画图）
    pipeline.py                 图像 → LaneObservation（不发布车辆指令）
  ros/
    images.py                   Image解码，处理RGB/BGR与行填充
    resources.py                通过包索引加载车辆几何参数
    lane_follower.py            订阅、消息转换、50Hz控制适配与唯一指令发布
    commands.py                 退出时持续发送零速
  visualization/
    render.py                   原图/俯视图绘制
    display.py                  可选窗口、Q退出和定期保存
  simulation/gazebo_start_pose.py  可选世界位姿记录/复位
  tools/check_curve_follow.py   有界行驶验收，结束后恢复测试前位姿
  app.py                        装配模块、Ctrl+C、退出清理
  exit_signals.py               合并终端与Launch重复退出信号，保护清理流程
```

`LaneObservation` 提供误差、目标点、车道模型和可视化所需观测；控制模块独立计算转向。
`FollowController.step(now)` 输出 `Decision`，不直接接触 ROS 发布器，可离线测试。
地面车道拟合算法、前视距离、时序融合和转向公式保持迁移前行为；像素模式也保留。
地图真值只供验收工具测偏差，不用于选择车道。
下一步接比赛状态机时，可复用感知观测和路径控制，避免再把任务判断塞进图像回调。

## 测试

```bash
colcon test --packages-select remote5g_autonomy --event-handlers console_direct+
colcon test-result --verbose
```

源码下运行同样的单元测试（先加载 ROS 环境）：

```bash
python3 -m pytest src/remote5g_autonomy/test -q
```

左起点有界驾驶验收工具会实际控制车辆并在结束时复位，请针对自己启动的测试实例运行：

```bash
ros2 run remote5g_autonomy check_curve_follow --distance 25
```

默认布局和输出在 `~/.ros/remote5g/reports/` 下，可用 `--layout`、`--output` 指定。
该工具仅针对当前左起点验收几何，不是比赛控制器。
