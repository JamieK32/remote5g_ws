# 从这里开始：我的 Gazebo 智能车工程

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

采集日期：2026-10-15（Asia/Shanghai）。任务范围经用户确认：**只以 `/home/jamiek/remote5g_ws` 为主**。本目录是可交给 ChatGPT 的调查资料；不是新建的仿真工程，也没有实现新的比赛算法。

## 项目与事实边界

目标是先在 Gazebo 中学习 ROS 工具、视觉与运动控制，逐项完成全国大学生智能汽车竞赛室外5G远程驾驶任务，充分验证后再考虑实车。学习方式：先理解原理、亲自做小实验、记录结果，再逐渐组合功能。

全文统一状态：**[V] 本次运行验证**；**[S] 当前文件/源码存在、未在本次完整运行验收**；**[U] 待验证或无法确认**；**[N] 本工作区未找到实现/模型**。“未找到”不等于整台电脑从未安装。历史测试单独写 [H]，不能冒充本次重测。

| 核心事实 | 当前结果 |
|---|---|
| 主工程 | `/home/jamiek/remote5g_ws`，用户已确认；另一个 `autocar_ws` 不纳入本工程 |
| 系统 | [V] Ubuntu 22.04.5 LTS，WSL2、x86_64，Bash |
| ROS | [V] ROS **2 Humble**，`/opt/ros/humble`；构建为 colcon + ament_cmake |
| Gazebo | [V] **Gazebo Classic 11.10.2**；不是新 Gazebo，不需要 ROS–Gazebo Bridge |
| 源码包 | [V/S] `remote5g_sim`（车辆/插件/操作/测试）、`remote5g_competition`（道具/布局/验证） |
| 车辆 | [V/S] 模型名 `xtnetrc`，自建 XT-NetRC 参考几何，4轮、前轮阿克曼转向、后轮共同扭矩驱动 |
| 主控制 | [V] `/ackermann_cmd` → `ackermann_msgs/msg/AckermannDriveStamped`，speed=m/s，steering_angle=中心转角rad |
| 兼容控制 | [V] `/cmd_vel` → `geometry_msgs/msg/Twist`，linear.x=m/s，angular.z=rad/s；插件转换成转角，不能原地旋转 |
| 传感器 | [V] 前置固定RGB与云台RGB，两路 Image + CameraInfo；云台2自由度 |
| 其他数据 | [V] `/odom` 仿真真值、`/joint_states`、`/diagnostics`、`/tf`、`/tf_static`、`/clock` |
| 未找到设备/模块 | [N] LiDAR、深度相机、IMU、GPS、SLAM、Nav2、ros2_control、比赛状态机、视觉巡线/避障/自主泊车 |
| 场景 | [V/S] 平坦测试场；红蓝400m跑道；比赛道具含挡板、横道、随机弯道单锥桶、四排引导、左右标志、A/B停车与测试二维码 |
| 工具 | [V] RViz启动、rqt_image_view启动、工具插件枚举、键盘遥控实际低速运动、短bag录制/检查/隔离回放；完整GUI显示配置仍待人操作 |

## 车为什么会运动

`drive.py` 或键盘工具发 ROS 消息 → gzserver 内的 `libremote5g_vehicle.so` → 限速、限转角和命令超时检查 → 两个前轮转向关节PD、两后轮相同扭矩 → Gazebo碰撞/摩擦产生真实运动 → 插件发布实际位姿与关节状态。不是通过直接改位姿来冒充车辆运动。

软件速度上限0.6m/s，中心转角上限25°，命令租约0.3秒墙钟。第一批实验只用0.10m/s。`/odom`是Gazebo真值，不是实体编码器。完整参数及假设见03。

## 实际常驻节点

比赛入口实测6个：`/gazebo`、`/gazebo/competition_state`、`/xtnetrc_control`、`/robot_state_publisher`、`/camera/front/front_camera_ros`、`/camera/gimbal/gimbal_camera_ros`。`spawn_entity`成功插车后退出；键盘和检查工具按需出现。车辆插件节点位于gzserver内部，没有独立的 `vehicle_controller` 进程。

## 最小启动与关闭

新开 WSL Ubuntu 终端，先用适合初学者的平坦场地：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=75
ros2 launch remote5g_sim sim.launch.py scene:=flat gui:=true
```

[本次验证] 同入口以 `gui:=false port:=11405`、ROS域95启动，并实际接受低速指令；比赛入口另外验证了gzclient截图。上面平坦场GUI组合本次未重复验收，已有历史记录。

比赛场景另一次启动使用：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch remote5g_competition competition.launch.py gui:=true
```

默认发车挡板存在，车不会自动发车。GUI/道具/车辆加载在本次隔离端口11405、域95实际验证。普通启动的端口为11375（车辆）或11385（比赛），**ROS域由终端设置，launch不自动替你选75/85**。不要混入 `/home/jamiek/autocar_ws/install`。

在启动终端按Ctrl+C，等待所有子进程结束；关闭Gazebo窗口不保证服务端已退出。每次只运行一个控制指令源。初始调查创建的仿真、GUI与录放进程已停止；本次经用户授权修改频率配置与发布脚本，未改几何模型或比赛算法。

## 摄像头与分辨率

`/camera/front/image_raw`、`/camera/front/camera_info`；`/camera/gimbal/image_raw`、`/camera/gimbal/camera_info`。车辆包默认640×480，比赛launch默认1280×720，两者配置不同，不是文档矛盾。两个CameraInfo与optical frame都独立，不可混用实车标定。

本次键盘工具确实可用：在平坦场的相同ROS域，运行 `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.05`。按i低速前进，k/空格停车。该安装版本每次按键发布一次，松键后由车辆0.3s租约保护减速；详见06，先完成启动观察实验再驾驶。

## 现有能力与缺口

已有：动力学车辆、低层转向/驱动、双RGB云台、跑道/比赛道具、控制接口、几何和物理历史测试、实际操作脚本。尚缺：视觉道路误差、障碍位置估计、路径规划/跟踪、横道任务、语音、随机标志识别、自动变道和停车、统一任务状态机。能手动开动不等于能参加比赛。

本地规则PDF是第二十届、2025年资料，尚未确认目标赛季是否沿用。停车等待在PDF有3s与10s冲突，不能由导师自行确定。用户已明确：区域2二维码暂没有明确车载识别任务，现有码只是可读性测试、不是支付接口。

## 文档索引与上传顺序

1. [01 环境](01_ENVIRONMENT.md)：版本、安装、依赖、环境变量。
2. [02 结构](02_PROJECT_STRUCTURE.md)：目录、包、文件与引用。
3. [03 模型与场景](03_VEHICLE_AND_GAZEBO.md)：机械、动力学、传感器。
4. [04 ROS接口与五张图](04_ROS_GRAPH_AND_INTERFACES.md)：节点、消息、TF、控制链路。
5. [05 操作手册](05_RUNBOOK.md)：实际命令、验证状态和停止方式。
6. [06 工具](06_ROS_TOOLING.md)：键盘、RViz、rqt、bag与仿真时钟。
7. [07 能力缺口](07_CAPABILITY_GAP.md)：比赛需求逐项对照。
8. [08 学习路线](08_LEARNING_ROADMAP.md)：A到G的小实验与验收。
9. [09 工程规范](09_ENGINEERING_PRACTICES.md)：现状、最小改进。
10. [10 未知与问题](10_OPEN_QUESTIONS.md)：冲突、失败、待用户补充。
11. [11 给ChatGPT的指导](11_HANDOFF_TO_CHATGPT.md)：直接复制给导师。

最省事的上传文件是本目录生成的 `CHATGPT_FULL_CONTEXT.md`（合并以上12份文档与关键事实）。也可先上传00与11，再补齐01–10。需要源代码定位时补充 `snapshot/source/` 中的小型文本副本；不是上传整个工作区。`chatgpt_handoff_text.zip` 是轻量文本资料包；`snapshot/README.md`说明原始证据与本地专用文件。

**第一项动手实验：只启动平坦场，列出节点，收到一条 `/odom`，理解“发布者/订阅者/反馈”。先不写代码、不做巡线，确认结果后再学键盘驾驶。**

证据入口：[本次运行摘要](snapshot/verification_summary.json)、[比赛图快照](snapshot/runtime/competition/runtime.json)、[平坦运动快照](snapshot/runtime/flat/runtime.json)、[包列表](snapshot/evidence/packages.txt)。根README已在GitHub发布整理时更新；当前Git/远端状态见snapshot/repository_setup.json。
