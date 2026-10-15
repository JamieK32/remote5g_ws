# 06 基础ROS工具：按当前工程学习

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

事实来源：已安装工具与本地 `--help`、teleop源码、当前接口、snapshot/evidence。工具安装、程序启动、显示内容正确是不同层级。当前应使用ROS2命令，不使用roscore、rostopic、rosbag（ROS1）。

## 键盘控制

[V] 系统包 `teleop_twist_keyboard` 2.4.1存在，实际PTY启动/前进/停车已验证。工作区没有专用键盘脚本，但不需要新写节点才能手动开车。

终端A启动平坦场，B看odom，C加载同环境/同域并运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.05
```

| 键 | 当前工具发出的Twist | 此阿克曼车的意义 |
|---|---|---|
| i | vx=+speed，wz=0 | 前进 |
| u/o | vx=+speed，wz=±turn | 前进时左/右曲线；delta=atan(L*wz/vx) |
| , | vx=−speed，wz=0 | 倒车 |
| m/. | vx=−speed，wz=±turn | 倒车转弯，转角符号需按v反号计算，不套前进手感 |
| k/空格 | 速度和角速度归零 | 减速停车 |
| q/z | speed和turn同时×1.1/×0.9 | 修改命令幅值，插件仍有限幅 |
| w/x、e/c | 分别修改线速度或角速度幅值 | 后续再学，第一课不必使用 |
| j/l | vx=0，wz非零 | 不能原地转，插件拒绝并停车 |
| 大写侧移、t/b | 非平面分量 | 本车不支持，插件拒绝 |

安装源码 `/opt/ros/humble/lib/python3.10/site-packages/teleop_twist_keyboard.py`只声明stamped、frame_id、speed、turn四个只读参数；**没有教程常见的repeat_rate/key_timeout**。默认stamped=false，发布Twist；不要改为TwistStamped，因为控制插件订阅的是Twist。每次按键发布一次，持续按键依赖键盘自动重复；松键保护来自车辆0.3秒租约。

验证分三步：`ros2 topic info /cmd_vel --verbose`看真实publisher/subscriber；`ros2 topic echo /cmd_vel`看消息；`ros2 topic echo /odom --once`与diagnostics看实际响应。后两条用于教学的形式[S]，本次以直接订阅及实车位移验证了等价数据。指令存在不等于轮胎真的运动。

## RViz2

[V 启动/OpenGL通过；以下显示项人工配置待做]：

```bash
rviz2 -f odom --ros-args -p use_sim_time:=true
```

所有查看终端与仿真同ROS域。Fixed Frame用 `odom`，不存在map/world TF。建议逐项Add，不一次配置全部：

| 显示 | 数据与设置 | 能说明什么 |
|---|---|---|
| TF | /tf、/tf_static | 帧连通，区分固定/运动安装 |
| RobotModel | Description Source=Topic，/robot_description；如QoS可选用Transient Local | 当前简化车辆与关节，不能替代Gazebo碰撞/物理验证 |
| Image | /camera/front/image_raw，再单独加gimbal | 原始像素，无需先做地面投影 |
| Camera（后学） | 匹配的Image/CameraInfo与optical TF | 图像与三维相机模型关系，不能混用两个K |
| Odometry | /odom，保留适量历史箭头 | 位姿/方向序列；本工程没有nav_msgs/Path发布器 |
| LaserScan/PointCloud2 | 不添加 | 本模型无这类传感器，空显示不是故障 |

File→Save Config As保存到你选定的实验目录，例如 `/home/jamiek/remote5g_ws/data/experiments/rviz_learning.rviz`；这是后续用户练习路径，本次没有创建RViz项目配置。之后 `rviz2 -d ... --ros-args -p use_sim_time:=true`加载。保存与全显示项验收目前[U]。RViz用于可视化，不录制实验，不能替代bag。

## rqt工具

[插件枚举V；rqt_image_view启动V；其他GUI实际操作U] 所有以下插件均已在本机清单中找到。命令形式来自实际包/插件，但本次没有逐个打开并人工操作：

```bash
rqt --standalone rqt_graph.ros_graph.RosGraph
rqt --standalone rqt_plot.plot.Plot
rqt --standalone rqt_console.console.Console
ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
ros2 run rqt_reconfigure rqt_reconfigure
```

- Graph：先观察 `/joint_states`→RSP、`/odom`→查看工具，运行键盘时再看 `/cmd_vel`→车辆。检查隐藏未连接/调试节点的选项，和本次导出图比较。
- Plot：可选 `/odom/twist/twist/linear/x`、`/odom/pose/pose/position/x`、`/cmd_vel/linear/x`或 `/ackermann_cmd/drive/speed`。绘图语法按插件字段选择；JointState数组先在Topic工具确认joint名称顺序，不把数组索引当固定轮名。
- 图中cmd_vel与odom speed是目标和实际，可以对照响应延迟；odom位置x/y随时间两条线不是自动的二维轨迹图。
- ImageView：选择raw；不存在的单一路径 `/camera/image_raw`来自另一工程，不能使用。
- Console：查看 `/rosout`，关联命令超时、非法Twist及插件加载日志；WARN不一定意味着崩溃。
- Reconfigure：ROS2参数工具，先看RSP的publish_frequency/use_sim_time。车辆max_speed等并非该节点ROS动态参数，不能在这里调整后宣称生效。暂不修改任何参数。

## ros2 bag：本次实际录放

[V] 已记录约5.5秒、501条消息、303.1KiB：/odom111、/joint_states111、/tf222、/tf_static1、/clock56。文件本地在 `docs/chatgpt_handoff/snapshot/runtime/competition/sample_bag`；metadata与bag info可分享，db3留本地作练习。

本次使用的命令（环境为检查域95，路径是文档证据目录）：

```bash
ros2 bag record -o /home/jamiek/remote5g_ws/docs/chatgpt_handoff/snapshot/runtime/competition/sample_bag --use-sim-time /odom /joint_states /tf /tf_static /clock
```

**上面是已执行记录，不要重复用同名目录重新录制。**后续新实验取唯一目录，例如 `data/experiments/YYYYMMDD_A2_keyboard_01/bag`，先建父目录，不预建bag最终目录。只录必要话题，初学先5秒，Ctrl+C结束，让metadata正常收尾。将来视觉实验再加入对应Image与CameraInfo及控制topic，并记录分辨率/带宽；不默认全话题录几个小时。

[V] 查看已有录制：

```bash
ros2 bag info /home/jamiek/remote5g_ws/docs/chatgpt_handoff/snapshot/runtime/competition/sample_bag
```

这份bag使用仿真时间，info起点显示1970附近是仿真秒数映射，不是系统日期错误。`--use-sim-time`录制器等待第一个clock后才按仿真时间记录。tf_static是一次静态消息，频率为空属于正常。

## 安全回放与时钟

[V] 本次在无Gazebo控制器的域96实际回放，并用use_sim_time=true观察节点收到111条odom、109条播放器clock，见 [playback_observation.json](snapshot/playback_observation.json)。命令：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=96
ros2 bag play /home/jamiek/remote5g_ws/docs/chatgpt_handoff/snapshot/runtime/competition/sample_bag --topics /odom /joint_states /tf /tf_static --clock 20
```

这个选择列表**不包含录下来的/clock**，由播放器单独以20Hz提供clock，避免同时播放旧clock和生成clock。不要让Gazebo和播放器在同一ROS域各发一个clock。本次raw日志未连接终端时提示键盘控制关闭，播放仍正常完成，不是失败。

在同一域96另开RViz（use_sim_time=true、Fixed Frame=odom）可练习看Odometry/TF；本次没有人工验收回放RViz效果。RobotModel还需要URDF描述：这份bag没有robot_description，单放TF不会自动出现车模型。初学样例先只看Odometry/TF；以后录制时一并录 `/robot_description`，确认其TRANSIENT_LOCAL持久性，再练习离线RobotModel。不要为显示模型重启整套Gazebo，也不要在回放TF的同时另起RSP重复发布同一批TF。

用rqt_plot观察回放的 `/odom/twist/twist/linear/x`，这份样例是静止录制，速度应接近0，不会凭播放出现行驶过程。运动曲线要另录键盘/drive实验。带图像的bag才能播放相机画面，此样例未录图像。

## 数据回放与控制指令回放的区别

传感器/状态回放只是发布旧数据，不重新计算当时的物理。控制回放会驱动任何订阅者，效果还取决于初始位姿/世界/物理/超时。初学回放显式白名单/odom等，排除 `/ackermann_cmd`、`/cmd_vel`、云台命令及任务控制，不在在线车辆域播放控制。未来确需控制复现，另立实验：唯一指令源、独立域、固定初始场景、记录seed/版本及看门狗策略；本次未做控制bag回放。

建议记录格式：日期、实验编号、目的、source版本/文件hash、vehicle和场景配置/seed、ROS域/端口、命令、bag路径、预期/实测、失败与结论。保持配置副本和日志，不用仅凭截图判断闭环是否成功。
