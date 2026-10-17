# B1 视觉循线

在已启动 `remote5g_ws` 仿真、且终端 ROS_DOMAIN_ID 与仿真一致时运行。

查看原始白线识别：

```bash
cd ~/remote5g_ws
python3 learning/b1_lane_follow.py --show
```

在直弯重叠区使用地面车道跟踪：

```bash
python3 learning/b1_lane_follow.py --filter-straight --show
```

`--filter-straight` 现在启用完整的地面车道跟踪，不再使用旧的图像长直线擦除和三行配对逻辑。
它读取 `/camera/front/camera_info` 和车辆配置中的相机安装参数，将白线投影到地面，
按照车道宽度和弯曲形状配对两侧边界。交叉处优先选择可靠弧线，并通过里程计的相对运动
连续跟踪同一车道；用 Pure Pursuit 前视目标点计算 Ackermann 转角。
控制器不读取赛道地图、全局中心线或 Gazebo 模型真值来选择路径。

绿色为选中的车道边界，青色为目标中心线，黄色点为转向目标；
红色为检测到的直线候选，右侧是有米制尺度的地面俯视图，白色为原始白线观测。
正常直道没有竞争弧线时仍可识别直线车道，缺少两条可靠边界时停车。
相机内参缺失时也会等待，不使用未经校准的投影起步。

默认 `--lane-width 1.22` 米、`--lookahead 2.5` 米，适配当前平坦赛道和固定前置相机。
相机安装位置、高度和俯仰角来自 `src/remote5g_sim/config/vehicle.yaml`；
车辆滚转、俯仰和高度变化由 `/odom` 补偿。
不加 `--filter-straight` 时保留原像素循线模式，`--kp` 仅作用于该像素模式。

上述命令仅预览，不发送车辆控制命令；`--drive` 才启用控制。
Q 或 Ctrl+C 退出。`--save-dir /绝对路径` 可保存周期性预览图。

以目标速度 0.7 m/s 开启直线过滤、显示窗口，并持续运行到手动退出：

```bash
python3 learning/b1_lane_follow.py --drive --speed 0.7 --continuous --filter-straight --show
```

`--continuous` 关闭时间和距离限制，也不会因起步等待或丢线而自动退出。
识别不到车道、相机或里程计数据过期时仍发送停车指令，并等待数据和车道检测恢复后继续行驶。
Ctrl+C 或窗口中按 Q 会发送停车指令后退出。速度参数范围为 `0 < speed <= 0.7`。
不加 `--continuous` 时仍使用默认的 15 秒、距起点 2 米限制及丢线退出逻辑。

在上述行驶命令上加 `--reset-on-exit`，可在启动时记录车辆位姿，退出后回到该位置和朝向：

```bash
python3 learning/b1_lane_follow.py --drive --speed 0.7 --continuous --filter-straight --show --reset-on-exit
```

这是 Gazebo 内直接复位车辆模型：行驶前读取 `world` 坐标下的完整位置、姿态；
Ctrl+C、Q 或其他正常结束路径会先发送停车指令，再复位车辆并将模型速度设为零。
不会重置地图、道具或整个仿真，也不会沿路线自动驾驶返回。
默认车辆实体名为 `xtnetrc`，可用 `--vehicle-entity 名称` 指定。
若启动时无法读取位姿或找不到复位服务，会停止并报错，不开始行驶。
Gazebo 关闭、强制终止进程等情况无法保证复位；复位不会把独立的里程计原点清零。

地面车道几何回归检查（不需要 ROS 或启动仿真）：

```bash
python3 -m unittest discover -s learning -p test_ground_lane_tracker.py -v
```

全部单元检查（需先 source ROS 和工作空间，无须启动仿真）：

```bash
python3 -m unittest discover -s learning -p 'test_*.py' -v
```

左起点 Gazebo 实际行驶验收，会以 0.7 m/s 行驶 25 米并复位到测试前位姿：

```bash
python3 learning/check_curve_follow.py --distance 25
```

此工具读取当前 layout 仅用于测量与目标车道中心的偏差，不把地图传给控制器。
2026-10-16 当前左起点测试通过：25.01 米、航向变化约 35.1 度、最大中心偏差 0.138 米。
记录保存在 `reports/curve_follow_v2/drive2/`。该结果覆盖当前重叠弯道段，尚未验证整圈。
