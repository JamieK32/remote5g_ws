#!/usr/bin/env python3
"""Generate handoff documentation from the inspected local project; no project edits."""
from pathlib import Path
import json
D=Path(__file__).resolve().parent.parent
S=D/'snapshot';W=D.parents[1]
def save(name,text):(D/name).write_text(text.strip()+'\n')
save('00_START_HERE.md',r'''
# 从这里开始：我的 Gazebo 智能车工程

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
''')
save('01_ENVIRONMENT.md',r'''
# 01 实际开发环境

日期2026-10-15，范围 `/home/jamiek/remote5g_ws`。状态定义见00；安装、可启动和功能验证分别记录。

## 系统、工具与隔离

| 项目 | 结果及状态 | 证据 |
|---|---|---|
| OS/架构 | [V] Ubuntu22.04.5 LTS，x86_64，WSL2 | snapshot/evidence/system.txt |
| 内核 | [V] 6.18.54.1-microsoft-standard-WSL2 | 同上 |
| Shell | [V] Bash，`/bin/bash` | 环境白名单与system.txt |
| ROS | [V] ROS2 Humble，`/opt/ros/humble` | environment_after_source.json、packages.txt |
| Gazebo | [V] Classic11.10.2，`/usr/bin/gazebo`，gzserver/gzclient | versions.txt、两次launch日志 |
| RViz | [V] rviz2 11.2.29，启动并初始化OpenGL4.2；[U] 全部显示项人工验收 | installed_tools.txt、rviz_start.txt |
| rqt | [V] 插件清单成功，rqt_image_view实际启动；其余GUI功能只确认安装/可发现 | rqt_plugins.txt、rqt_image_start.txt |
| rosbag | [V] rosbag2 0.15.17，sqlite3，已录制/检查/隔离回放 | bag_info.txt、playback_observation.json |
| 构建 | [V/S] colcon命令可运行，两个ament_cmake包；本次频率修订已构建 | snapshot/frequency_update/build.log |
| Python | [V] `/usr/bin/python3` 3.10.12 | python_modules.txt、versions.txt |
| C++/CMake | [V/S] GCC/G++11.4.0、CMake3.22.1，项目C++17；工具版本已核对，本次两包构建成功 | versions.txt、包CMakeLists |
| 图像依赖 | [V] OpenCV4.5.4，NumPy1.21.5，PyYAML5.4.1，均系统Python路径 | python_modules.txt |
| 当前隔离 | [V] 未激活Conda/venv；WSL2；未发现当前Docker容器标志，本工程无Dockerfile/compose | 环境白名单、system.txt、目录清单 |
| catkin | [N] 当前PATH未找到catkin_make，两个包均非catkin | command-v及package.xml |

`ros2`、rviz2、rqt在未source的初始shell里不在PATH，source后可用。不能据此误判ROS没安装。Docker工具当前PATH未检出，不证明这台Windows从未装过Docker。未读取整份用户环境、shell配置、SSH或凭据文件。

## 路径与空间

| 绝对路径 | 工程相对位置 | 用途 |
|---|---|---|
| `/home/jamiek/remote5g_ws` | `.` | 当前选定工作区 |
| `/home/jamiek/remote5g_ws/src` | `src/` | 两个ROS源码包 |
| `/home/jamiek/remote5g_ws/build` | `build/` | 已有CMake/CTest产物，不能当源代码 |
| `/home/jamiek/remote5g_ws/install` | `install/` | 已有安装覆盖层，setup.bash可用 |
| `/home/jamiek/remote5g_ws/log` | `log/` | 历史colcon日志 |
| `/home/jamiek/remote5g_ws/reports` | `reports/` | 车辆/场景历史验收，不是全部当前功能 |
| `/home/jamiek/remote5g_ws/data` | `data/` | 现有预留数据目录，未发现正式采集数据集 |
| `/home/jamiek/remote5g_ws/docs/chatgpt_handoff` | `docs/chatgpt_handoff/` | 本次唯一交接资料输出根 |
| `/home/jamiek/.cache/remote5g_scene_libs` | 工程外 | 现有qrcode8.2、zxing-cpp2.3.0及PDF查阅PyMuPDF缓存 |

Gazebo ROS与各消息包在 `/opt/ros/humble`；本工程插件在 `install/remote5g_sim/lib/libremote5g_vehicle.so` 与两个截图插件库。`ros2 pkg prefix`确实指向本工作区，没有混入autocar。

## 环境变量

取值快照分别在 [source前](snapshot/environment_before_source.json) 与 [source后](snapshot/environment_after_source.json)。只导出白名单。

- `ROS_VERSION=2`、`ROS_DISTRO=humble`。
- `AMENT_PREFIX_PATH`依次为competition、sim、`/opt/ros/humble`。
- `COLCON_PREFIX_PATH=/home/jamiek/remote5g_ws/install`。
- `RMW_IMPLEMENTATION`未显式设置；实际标识见 `snapshot/evidence/rmw.txt`，不能把空变量解释成没有中间件。
- `DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`，本次Gazebo/GUI依赖WSLg。无gzclient运行不等于已验证完全无显示环境。
- 普通用户选择 `ROS_DOMAIN_ID=75`（车辆练习）或85（比赛）；所有参与同场实验的终端必须一致。域不是Gazebo端口，也不是自动启动参数。
- 本次检查用域95、`ROS_LOCALHOST_ONLY=1`、Gazebo11405；离线bag播放用域96。所有本次进程已清理。
- launch为子进程设置 `GAZEBO_MODEL_PATH`、`GAZEBO_PLUGIN_PATH`、`GAZEBO_MASTER_URI`、禁用在线模型库；这些不一定自动回写另一个终端。

## 依赖的真实状态

[当前可运行] C++插件及系统ROS依赖已满足，两个场景均成功启动。基础环境不需要重装。

[重要] 系统Python直接import qrcode/zxingcpp/fitz未找到；现有场景脚本会把默认用户缓存路径加入sys.path，因此本次生成与QR验证可运行。包声明python3-qrcode不等于系统已装该模块。`scripts/bootstrap_deps.sh`固定qrcode8.2、zxing-cpp2.3.0，但依赖下载pip.pyz；本次**没有执行安装脚本**。新电脑的依赖复现仍应单独计划，不把缓存当完整环境锁。

[未确认] `ros2 doctor --report`在35秒内未完成，已超时停止，原因未定位；这不是“环境全部健康”的证明，也不推翻实际启动结果。没有为此修改网络或安装软件。

来源：`snapshot/evidence/{system,versions,installed_tools,python_modules,ros_tools}.txt`、两个package.xml、两份CMakeLists、环境JSON。版本均来自本机，不是网络推荐版本。
''')
save('02_PROJECT_STRUCTURE.md',r'''
# 02 工程结构、包与资源

主根目录 `/home/jamiek/remote5g_ws`，不是home目录下任意其他仓库。源码包清单经 `colcon list`复核只有两个。未在本工程及父级发现适用AGENTS.md。

## 精简目录树

完整受控树见 [project_tree.txt](snapshot/project_tree.txt)。省略构建缓存和重复资源内容：

```text
remote5g_ws/
├── src/
│   ├── remote5g_sim/
│   │   ├── package.xml, CMakeLists.txt, README.md
│   │   ├── urdf/{car,common,wheels,cameras}.xacro
│   │   ├── config/{vehicle,parameter_sources,hardware_reference_NOT_LOADED}.yaml
│   │   ├── src/{vehicle_plugin,gui_capture}.cpp
│   │   ├── include/remote5g_sim/control_math.hpp
│   │   ├── launch/sim.launch.py
│   │   ├── worlds/flat.world
│   │   ├── scripts/{drive,inspect_cameras}.py
│   │   ├── test/{control_math_test.cpp,model_check,regression,smoke}.py
│   │   └── docs/REFERENCE_REVIEW.md, reference_manifest.json
│   └── remote5g_competition/
│       ├── package.xml, CMakeLists.txt, README.md
│       ├── config/competition.yaml, layout_left.yaml, layout_right.yaml, parameter_sources.yaml
│       ├── launch/competition.launch.py
│       ├── scripts/{scene_core,generate_layout,validate_layout,manage_props,parking_check}.py
│       ├── models/comp_*/（19种独立SDF及网格/材质）
│       ├── materials/source_signs/（PDF嵌入标志位图）
│       ├── worlds/{left,right}_{red,blue}/（派生world/清单/资源）
│       ├── test/（几何、实际Gazebo、CLI、矩阵与报告工具）
│       └── docs/（尺寸、场景、规则素材溯源和验收）
├── config/athletics_track.json
├── simulation/{models,worlds,maps}/
├── scripts/{generate_athletics_tracks.py,launch_track.sh,test_xtnetrc.sh,lib/}
├── docs/{official,notes,GAZEBO_TEST_REPORT.md,chatgpt_handoff/}
├── reports/{xtnetrc,competition}/
├── artifacts/, deliverables/（历史图片及分发归档）
├── data/（预留）
└── build/, install/, log/（生成物）
```

树中 `.py`后缀组合只作简写，`control_math_test.cpp`确为C++，不存在同名Python文件。当前没有setup.py/setup.cfg的Python包，两包都是ament_cmake。

## 两个包及关系

| 包 | 功能与主要依赖 | 入口/产物 |
|---|---|---|
| remote5g_sim 0.1.0 | Gazebo ModelPlugin、机器人结构、双相机、关节状态/里程计、操作与回归。依赖gazebo_ros/gazebo_plugins、rclcpp/rclpy、ackermann_msgs、geometry/nav/sensor/std/diagnostic_msgs、tf2_ros、xacro、RSP、Qt5 | sim.launch.py；libremote5g_vehicle.so；drive.py、inspect_cameras.py与测试脚本 |
| remote5g_competition 0.1.0 | 独立比赛道具、跑道局部坐标、随机种子、几何/停车检查。运行依赖remote5g_sim与gazebo_ros、ROS消息、numpy/opencv/yaml/PIL/qrcode及已有zxing缓存 | competition.launch.py；生成overlay world；manage_props.py、parking_check.py、validate_layout.py、run_matrix.py |

精确依赖分类（buildtool/build/exec）见 [packages.json](snapshot/packages.json)。competition复用sim模型，sim不依赖competition；两个包都读取现有轨道资源，没有把轨道重建成另一套坐标。

## 重要文件定位

- `remote5g_sim/config/vehicle.yaml`：机械/质量/摩擦/控制/相机参数；默认配置事实源。
- `urdf/car.xacro`：模型组装与插件参数；其余三个xacro分别是通用盒惯性/固定连接、轮胎转向、相机云台宏。
- `src/vehicle_plugin.cpp`：真实控制回调、限幅、租约、PD/P与实际反馈；不是视觉算法。
- `include/.../control_math.hpp`：阿克曼角、Twist转换和速度变化率；可独立单测。
- `sim.launch.py`：flat/red/blue三种车辆场景；config参数指车辆YAML。
- `competition.launch.py`：生成道具world、插入同一车；config参数指**比赛YAML**，含camera_width/height，不能混用两入口的config含义。
- `competition/scripts/scene_core.py`：读取轨道几何、Route.pose(s,d)、资源生成、四排引导、随机单障碍、四轮停车几何、QR验证。
- `config/athletics_track.json` + `scripts/generate_athletics_tracks.py`：既有轨道参数与生成源码。调查中只读，没有重新生成。
- `simulation/maps/lane_*_centerline.csv`：名义车道参考线；存在不代表已有路径跟踪器使用。
- `scripts/launch_track.sh`：只开原跑道，**不会自动插车或建立完整ROS车辆接口**。
- `test/*`：测试可临时发布控制或移动夹具；不是常驻比赛算法。
- `hardware_reference_NOT_LOADED.yaml`：实车PWM说明数据，不被launch/plugin读取。本次副本不包含该无关硬件参数文件。

## 资源引用链

```mermaid
flowchart TD
  CL["competition.launch.py"] --> GEN["scene_core.generate"]
  GEN --> TRACK["原 simulation/worlds/athletics_400m_*.world"]
  GEN --> CFG["competition.yaml + athletics_track.json"]
  GEN --> OVER["输出目录/competition.world"]
  OVER --> INC["model://wa_400m_track_red 或 blue"]
  INC --> SDF["simulation/models/wa_400m_track_*/model.sdf"]
  SDF --> DAE["meshes: track / markings / infield / kerb / surround"]
  OVER --> PROP["输出目录/models/comp_* 网格与材质"]
  CL --> XAC["remote5g_sim/urdf/car.xacro"]
  XAC --> YAML["vehicle.yaml + 四份xacro"]
  XAC --> PLUGIN["libremote5g_vehicle.so + libgazebo_ros_camera.so"]
```

小型world/SDF引用已放入snapshot/source；[resources.json](snapshot/resources.json)列出803条资源路径/字节数/URI/插件，数量含派生场景副本，不代表803个不同模型。DAE/PNG等大资源只列清单，不复制进轻量交接包。现有生成SDF可能含绝对file URI，跨机器需重新由launch生成，不直接移动旧world后假设能运行。预生成worlds中的旧vehicle_config及初始审计输出是生成当时的快照，当前运行参数以vehicle.yaml与新launch展开结果为准。

## 文档和测试证据的优先级

源码/配置与本次命令证据优先；历史README和验收只补充背景。根README初始曾有“尚未添加车辆代码或ROS包”的过时描述，本次GitHub发布整理已更新为当前两包、启动及频率说明。`src/remote5g_sim/docs/REFERENCE_REVIEW.md`确实引用stupid_car固定commit及工程外只读缓存，因此仅将其列为几何/默认值参考，**当前项目不是stupid_car，也没有实车控制后端**。

交接源文件副本曾被colcon递归发现为重复包，已在本目录加入 `COLCON_IGNORE` 隔离，并复查只剩原src两包；不改任何原构建文件。保留该标记，不要删除后再从工作区根构建。
''')
save('03_VEHICLE_AND_GAZEBO.md',r'''
# 03 车辆模型、Gazebo与传感器

核心文件：[car.xacro副本](snapshot/source/src/remote5g_sim/urdf/car.xacro)、[车辆参数](snapshot/source/src/remote5g_sim/config/vehicle.yaml)、[插件源码](snapshot/source/src/remote5g_sim/src/vehicle_plugin.cpp)。默认展开URDF见 [vehicle_default.urdf](snapshot/vehicle_default.urdf)，比赛1280配置另有vehicle_competition.urdf。展开不代表重新创建车型。

## 结构、坐标与动力学

[V/S] URDF名 `xtnetrc`，19个Link、18个Joint，其中8个动态关节（2转向、4滚动、2云台）、10个固定关节。四个车轮都是真实圆柱碰撞与惯性。前轮滚动为被动轮，前轮转向用绕Z的revolute关节；后桥按单虚拟电机/开放差速器假设给左右轮相同扭矩，允许物理滚动速度不同。不是差速转向，没有悬挂与软胎。

车体X前、Y左、Z上，单位m/rad/kg。`base_footprint`在后轴下方地面参考点，`base_link`在后轴中心、上移一个轮半径。`odom`数值采用Gazebo世界位姿，再作轮半径参考点变换。当前没有map→odom或world→odom TF；不要把world字符串擅自设为RViz Fixed Frame。

| 配置 | 当前值 | 性质 |
|---|---:|---|
| 轴距 | 0.23861m | 本地参数；来源标记为上游实测记录，本次未重新测量实体车 |
| 前/后轮距 | 0.24/0.24m | 仿真假设 |
| 轮半径/轮宽 | 0.055315/0.04952m | 上游卡尺记录，非本次实测，负载半径待确认 |
| 底盘参考盒 | 0.410×0.190×0.105m | 参考几何，整车带轮/云台包络更大 |
| 底盘盒中心 | base_link坐标(0.119305,0,0.115)m | 不是整个机器人质心 |
| 底盘/单轮/单转向节质量 | 3.0/0.18/0.06kg | 仿真假设 |
| URDF可见惯性质量合计 | 4.201kg | 静态求和，含支架/云台等；不代表实体车称重结果 |
| 摩擦/contact | mu=1，kp=1e6，kd=10；轨道mu=0.9 | 仿真接触参数 |
| 轮关节 | axis Y；effort2、velocity50、damping0.0005 | URDF限制；驱动实际扭矩另外限为0.6N·m |
| 转向关节 | ±0.6rad，effort1N·m | 前内轮限位容纳软件中心25° |
| 世界物理 | ODE、重力(0,0,-9.81)，平坦world步长0.001s | 场景文件事实；真实时间倍率依机器负载 |

通用盒惯性由 `common.xacro`按质量/尺寸计算，惯性原点与盒中心一致；轮胎使用圆柱惯性公式。底盘盒、横梁、立柱、相机/云台有简化碰撞；某些固定Link在Gazebo转换时会合并，不能因此断言URDF里的TF消失。所有Link/Joint/原点/限位和碰撞几何已导出 [inventory](snapshot/vehicle_default_inventory.json)。

## 车辆到底如何控制

`libremote5g_vehicle.so`是Gazebo Classic ModelPlugin，ROS节点名 `/xtnetrc_control`，在gzserver内部运行。没有controller_manager、joint_trajectory_controller或ros2_control硬件接口。

- `/ackermann_cmd`：AckermannDriveStamped。`drive.speed` m/s，`drive.steering_angle`中心转角rad，正角左转、负角右转。`acceleration`和`steering_angle_velocity`非负：0用默认，正值只允许降低对应变化率。jerk只做有限数检查，没有jerk控制。
- `/cmd_vel`：Twist，仅linear.x和angular.z允许非零。转换 `delta=atan(wheelbase*omega/v)`；v接近0却要求非零omega不可实现，会清空目标并安全减速。倒车保留v符号。
- 两种入口共用最新有效命令，后收到的有效指令覆盖前者；它们不是两个独立控制器，不可同时跑键盘和算法。
- 最快速度0.6m/s；中心转角±0.436332rad（25°）；转向变化率2.094395rad/s；加速0.8、减速1.2m/s²。转弯按曲率进一步限速。
- 目标每0.02仿真秒更新；每物理步施加关节力。转向PD：kp8、kd0.15、限矩1；后轮速度P：kp1.5、单轮扭矩±0.6。
- 0.3秒**墙钟**收不到有效车辆命令时，目标速度归零、转角回中并按变化率减速，车辆不会瞬间固定在原地。暂停仿真期间租约也会过期，恢复后不复用旧行驶命令。
- `drive.py`通过Timer按50Hz墙钟发布，有时限，finally发零。它的header时间不被插件用于控制租约；租约依据消息到达墙钟。后续新控制节点应显式统一仿真时间，但不要误用消息时间推算现有租约。

初始调查的历史运动证据：平坦场Ackermann 0.10m/s、0.15rad、1秒，位移0.0941m；现有键盘Twist低速持续按键位移0.1178m，并停车。见snapshot/runtime/flat/runtime.json。没有通过SetWorldPose推动本次运动验证。

## 传感器与反馈

| 设备 | 配置/安装 | 话题/帧 | 验证状态 |
|---|---|---|---|
| 前置RGB，参考XWF名称 | 相对base_link(0.30,0,0.18)m；向下pitch0.45rad；距footprint名义高0.235315m；HFOV80°，clip0.02–100m | `/camera/front/image_raw`、`/camera/front/camera_info`；front_camera_optical_frame | [V] 图像、K、TF与PNG已采集 |
| 云台RGB，参考H65名称 | base_link→gimbal_base(0.08,0,0.195)，pan轴Z，tilt轴−Y，末端相机偏移(0.012,0,0.02)m；零位高度约0.310315m；HFOV70°，clip同前 | `/camera/gimbal/image_raw`、`/camera/gimbal/camera_info`；gimbal_camera_optical_frame | [V] 图像、K、TF；回中服务成功 |
| 里程计 | Gazebo base_link实际位姿/速度转换为后轴地面参考，50Hz仿真目标 | `/odom`，header=odom、child=base_footprint | [V] 当前收到；不是实车编码器 |
| 关节 | 实际8动态关节位置、速度、施力 | `/joint_states` | [V] 轮/转向/云台数据存在 |
| 深度、LiDAR、IMU、GPS | [N] 当前Xacro及运行图未找到 | 不应编造 `/scan`、`/imu`、`/gps/fix` 或点云 | 本工程不具备 |

两相机均由 `libgazebo_ros_camera.so`直连ROS2，不使用bridge。模型配置更新率为前置30Hz、云台15Hz（仿真时间），`R8G8B8`输出ROS `rgb8`。默认sim入口640×480；competition默认1280×720。两个camera_info均为针孔、plumb_bob、D全0；不是加载真实镜头标定。

比赛实测1280×720：front fx=fy=762.724934，gimbal fx=fy=914.015188；两者cx=640.5、cy=360.5。不要把这些K直接用到640模式。相机optical固定RPY=(-π/2,0,-π/2)，光学X右/Y下/Z前；地面投影需要当前K和TF，不可套实车单应矩阵。

初始调查实测publisher可靠性为RELIABLE（见image_qos.txt），BEST_EFFORT订阅也兼容；`/clock`是BEST_EFFORT。旧配置初始多话题/GUI采样窗口收到front约4.00Hz、gimbal约3.36Hz（墙钟、接收端）；CameraInfo约12Hz，后续/odom工具约19.8Hz。不能把配置15Hz当稳定实时接收保证，也不能据此断言相机插件只发布4Hz；有渲染负载、RTF和传输/订阅丢帧等未分离因素。当前30fps前置采集的实测另见FREQUENCY_UPDATE_20261015.md。

image_transport额外广告了带重复子命名空间的compressed/compressedDepth/theora话题；这里只验证raw，**compressedDepth名字不代表存在深度传感器**。

Pan限位±π/2，Tilt[-0.6,0.8]rad，目标速率1rad/s、PD与限矩0.15。正Pan左看、正Tilt抬头；服务 `/gimbal/center`为Trigger。机械角是仿真假设，不从PWM数值推导。

## 场景与比赛道具

| 场景 | 文件 | 内容 |
|---|---|---|
| 平坦 | src/remote5g_sim/worlds/flat.world | 地面、红蓝非对称立体标记、状态插件；适合最初控制实验 |
| 原红/蓝跑道 | simulation/worlds/athletics_400m_{red,blue}.world | 8道、白线、内沿；include独立wa_400m_track_*模型，不含比赛全套道具 |
| 比赛overlay | competition生成目录/competition.world | 原跑道+独立道具，状态插件与可选车辆，不覆盖原world |

轨道内半径36.5m、直道84.39m、名义道宽1.22m、白线0.05m、地面Z=0，世界X沿直道。默认比赛第3道的**白线内缘净宽1.17m**，Route中心半径39.525m；不要将名义1.22m直接替代当前净宽。

已有道具19类：发车区、区域2、停车区、蓝挡板、横道、红蓝黄三锥桶、红蓝底左右/A/B共8标志、主二维码架、A/B两地面码。场景53个道具实例：四排引导7+7+13+13=40个、弯道障碍1个及其余12件。障碍由obstacle_seed可重复随机生成，标志/停车由sign_seed控制。

引导按用户四排图组合图11两方向：原道两排蓝色（蓝底用红），两侧黄排收回原道；入口距标志约1.3995m，绝对纵向站位和曲线为仿真假设，不是官方精确坐标。已有静态回归曲率/空间检查，不等于实际自主变道成功。

主码与停车码都是非支付测试字符串。二维码支架碰撞存在；扫码测试不代表比赛要求已定义车载扫码，也不具备微信支付。区域2边界完整尺寸及45°空间解释在原场景文档标为假设。

原出处：config/parameter_sources.yaml及competition同名文件、MODEL_DIMENSIONS.md；规则PDF页3–9。stupid_car仅为本地明确引用的参数参考，未把其工程当当前仿真。
''')
save('04_ROS_GRAPH_AND_INTERFACES.md',r'''
# 04 ROS节点、通信接口与架构图

依据：[比赛运行图](snapshot/runtime/competition/runtime.json)、[平坦运行图](snapshot/runtime/flat/runtime.json)、[节点列表](snapshot/evidence/node_list.txt)、[服务列表](snapshot/evidence/service_list.txt)、源码副本。相对源码路径均相对 `/home/jamiek/remote5g_ws`。观察节点、ros2 CLI daemon和GUI工具不是车辆生产架构。

## 常驻节点统一清单

| 节点名称 | 所属包/源码或插件 | 功能 | 输入 | 输出 | 参数来源 | 启动入口 | 状态 |
|---|---|---|---|---|---|---|---|
| /xtnetrc_control | remote5g_sim；src/remote5g_sim/src/vehicle_plugin.cpp；libremote5g_vehicle.so | 限幅、看门狗、真实关节执行、反馈 | /ackermann_cmd、/cmd_vel、/gimbal/pan_cmd、/gimbal/tilt_cmd、/clock | /odom、/joint_states、/diagnostics、/tf | vehicle.yaml→Xacro→SDF；ROS use_sim_time=true | 两个launch随模型加载 | [V] |
| /robot_state_publisher | 系统robot_state_publisher；两个launch定义 | 将实际JointState与URDF转换为TF | /joint_states、/clock；robot_description参数 | /tf、/tf_static、/robot_description | use_sim_time=true、publish_frequency=50、URDF | 两个launch中的Node | [V] |
| /camera/front/front_camera_ros | 系统gazebo_plugins；cameras.xacro；libgazebo_ros_camera.so | 固定RGB渲染与针孔信息 | Gazebo camera sensor、/clock | /camera/front/image_raw、/camera/front/camera_info及传输插件广告 | front尺寸、HFOV、rate、光学帧，SDF | 随模型加载 | [V] |
| /camera/gimbal/gimbal_camera_ros | 同上，云台camera sensor | 云台RGB与独立内参 | 云台实际姿态、/clock | /camera/gimbal/image_raw、/camera/gimbal/camera_info及传输广告 | gimbal相机配置 | 随模型加载 | [V] |
| /gazebo | gazebo_ros；libgazebo_ros_init.so + factory | 仿真时间、插删模型、物理服务 | Gazebo更新、模型服务调用 | /clock、/performance_metrics、日志 | 系统插件参数与world | gzserver -s ... | [V] |
| /gazebo/competition_state | gazebo_ros；libgazebo_ros_state.so | 模型/Link真实状态与查询服务 | Gazebo状态 | /gazebo/model_states、/gazebo/link_states；get/set_entity_state服务 | update_rate=20，生成world插件 | competition world | [V] |

平坦场把最后一项替换成 `/gazebo/gazebo_state`，功能相近。两个相机插件、车辆控制、状态插件都是gzserver进程中的ROS节点；不要按“节点=独立进程”去寻找不存在的exe。

## 按需节点与离线工具

| 名称/工具 | 功能与入口 | 本次状态 |
|---|---|---|
| /spawn_entity | gazebo_ros spawn_entity.py；订阅robot_description后调用/spawn_entity | [V] 插入xtnetrc成功后退出，正常现象 |
| /xtnetrc_command_source | remote5g_sim/scripts/drive.py；只发布/ackermann_cmd | [V] 低速有限时长调用 |
| /teleop_twist_keyboard | 已装系统包teleop_twist_keyboard；只发布/cmd_vel | [V] PTY真实按键与运动验证，无本工程专用启动脚本 |
| /xtnetrc_camera_check | remote5g_sim/scripts/inspect_cameras.py；订阅Image/Info并保存PNG | [S/H] 脚本与历史成功记录；本次用独立观察程序采图 |
| /competition_prop_manager | manage_props.py；get/delete/spawn实体 | [S/H] 本次不增删道具 |
| /competition_parking_check | parking_check.py；读取真实四轮Link状态后做几何判定 | [S/H] 不代表自动泊车 |
| /xtnetrc_regression、/competition_integration、/competition_cli_verify | test/中的测试夹具，可能主动发控制/改变仿真实体 | [S/H] 本次没有运行整套回归 |
| generate_layout.py / validate_layout.py / scene_core.py | 生成器、离线校验与库；不是常驻视觉/规划节点 | [S] 比赛launch实际调用其中生成与校验函数 |
| GUI截图插件 | 两个包src/gui_capture.cpp；gzclient截图，不是控制节点 | [V] 本次比赛场景8张截图 |

## 关键Topic字典

| Topic | 消息类型 | 发布者→使用者 | 单位/时间/帧 | 状态 |
|---|---|---|---|---|
| /ackermann_cmd | ackermann_msgs/msg/AckermannDriveStamped | drive或未来单一控制源→xtnetrc_control | speed m/s，中心转角rad；接收墙钟租约 | [V] 控制运动 |
| /cmd_vel | geometry_msgs/msg/Twist | 键盘→xtnetrc_control | vx m/s、wz rad/s；非差速车 | [V] 控制运动 |
| /odom | nav_msgs/msg/Odometry | xtnetrc_control→显示/录制/未来算法 | pose在odom，twist在base_footprint；仿真stamp | [V] 真值、目标约50Hz仿真时间 |
| /joint_states | sensor_msgs/msg/JointState | xtnetrc_control→robot_state_publisher | 8关节rad、rad/s、N·m | [V] |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | xtnetrc_control→日志/检查工具 | 状态active/command_timeout等，目标、后轮扭矩、lease_age_s | [V] |
| /camera/front/image_raw | sensor_msgs/msg/Image | 前相机→显示/记录/未来视觉节点 | rgb8，front_camera_optical_frame，默认640或比赛1280模式 | [V] |
| /camera/front/camera_info | sensor_msgs/msg/CameraInfo | 前相机→未来投影/显示 | 与同相机图像尺寸/光学帧匹配 | [V] |
| /camera/gimbal/image_raw、/camera/gimbal/camera_info | Image / CameraInfo | 云台相机→显示/未来目标识别 | gimbal_camera_optical_frame；独立K/实际TF | [V] |
| /gimbal/pan_cmd、/gimbal/tilt_cmd | std_msgs/msg/Float64 | 手动或未来任务节点→车辆插件 | 弧度绝对目标，正Pan左看/正Tilt抬头 | [S/H] 本次回中服务成功 |
| /tf | tf2_msgs/msg/TFMessage | 插件+RSP→RViz/未来算法 | odom→footprint及活动关节 | [V] |
| /tf_static | tf2_msgs/msg/TFMessage | RSP→TF消费者 | 固定安装变换，TRANSIENT_LOCAL | [V] |
| /robot_description | std_msgs/msg/String | RSP→spawn/RViz | 展开URDF，TRANSIENT_LOCAL | [V] |
| /clock | rosgraph_msgs/msg/Clock | /gazebo→use_sim_time节点 | 仿真时间；BEST_EFFORT | [V] |
| /gazebo/model_states、/gazebo/link_states | gazebo_msgs/msg/ModelStates / LinkStates | 状态插件→检查工具 | 仿真模型/Link世界状态 | [V] 实体查询成功，广告列表存在 |

不存在活跃命令源时，`/ackermann_cmd`和`/cmd_vel`仍可因订阅者而列在topic list里；看见话题名不等于有指令。查看 `ros2 topic info ... --verbose` 区分发布者和订阅者。完整传输话题/参数事件/日志也在runtime.json，避免把compressedDepth广告误认成深度数据。

相机raw实测RELIABLE/VOLATILE，clock BEST_EFFORT/VOLATILE，tf_static与robot_description为TRANSIENT_LOCAL。采集里的depth=0可能是RMW端点报告值，不能据此声称发送缓存为0。可靠性/持久性优先以本次verbose及源码为准。

## Service、Action与参数

- [V] `/gimbal/center`，std_srvs/srv/Trigger；调用成功，设置云台目标0，不瞬间改位姿。
- [V] `/gazebo/get_entity_state`、`/gazebo/set_entity_state`，gazebo_msgs/srv/GetEntityState、SetEntityState；本次只调用get查询xtnetrc。
- [V/S] `/spawn_entity`、`/delete_entity`、`/get_model_list`；插车流程调用spawn，调查中不删除模型。
- [V/S] `/pause_physics`、`/unpause_physics`、`/reset_world`、`/reset_simulation`为Empty服务；列表存在，本次未调用reset。
- [V/S] `/camera/front/set_camera_info`和gimbal对应服务存在；未改标定。
- [V] `ros2 action list -t`为空，本场景未发现Action server；未发现controller_manager。
- [V] `/xtnetrc_control`的ROS参数dump只有use_sim_time、QoS覆盖等；机械/速度/转向P值来自SDF `p_` map，**没有ROS参数更新回调**。rqt_reconfigure不能当作实时底盘调参器。
- [V] RSP的use_sim_time=true、publish_frequency=50、frame_prefix空、robot_description存在。ROS参数与launch参数、YAML/SDF配置是三类不同对象。

## 图1：当前ROS通信

```mermaid
flowchart LR
  DRIVE["drive.py（按需）"] -->|"/ackermann_cmd"| CTRL["/xtnetrc_control"]
  KEY["键盘（按需）"] -->|"/cmd_vel"| CTRL
  CTRL -->|"/odom /diagnostics"| VIEW["RViz / rqt / bag（按需）"]
  CTRL -->|"/joint_states"| RSP["/robot_state_publisher"]
  RSP -->|"/tf /tf_static /robot_description"| VIEW
  CTRL -->|"/tf: odom到base_footprint"| VIEW
  CAM["两路camera_ros节点"] -->|"Image + CameraInfo"| VIEW
  GZ["/gazebo"] -->|"/clock"| CTRL
  GZ -->|"/clock"| RSP
  GZ -->|"/clock"| CAM
```

## 图2：传感器数据与未来视觉链

```mermaid
flowchart LR
  RENDER["Gazebo RGB sensor + 实际云台姿态"] --> PLUG["libgazebo_ros_camera.so"]
  PLUG --> IMG["两路 image_raw"]
  PLUG --> K["各自 camera_info"]
  RSP["TF安装外参/云台关节"] --> TF["/tf + /tf_static"]
  IMG --> DISPLAY["当前：看图与录制"]
  IMG -.-> VIS["未来：白线/锥桶/标志感知（未实现）"]
  K -.-> VIS
  TF -.-> VIS
  VIS -.-> PLAN["未来：轨迹/任务/路径跟踪（未实现）"]
  PLAN -.-> CMD["未来单一指令源：/ackermann_cmd"]
```

## 图3：从控制消息到物理执行

```mermaid
flowchart TD
  A["Ackermann speed/center steering"] --> CHECK["限幅/曲率限速/0.3秒墙钟租约"]
  T["Twist vx,wz"] --> CONVERT["delta=atan(L*wz/v)；拒绝原地转向"]
  CONVERT --> CHECK
  CHECK --> RATE["0.02秒仿真目标更新；速度/转角变化率限制"]
  RATE --> ACK["左右前轮阿克曼角"]
  ACK --> PD["转向PD与关节力矩"]
  RATE --> REAR["后桥平均轮速P；两轮相同扭矩"]
  PD --> CONTACT["Gazebo ODE + 轮胎碰撞/摩擦"]
  REAR --> CONTACT
  CONTACT --> ACTUAL["真实Link位姿/JointState"]
  ACTUAL --> FEED["/odom /joint_states /tf /diagnostics"]
```

## 图4：TF树

下图由当前19条TF边生成，含odom边；没有虚构map/world父帧。

TF_GRAPH_PLACEHOLDER

## 图5：Gazebo与ROS进程边界

```mermaid
flowchart TB
  LAUNCH["ros2 launch"] --> SERVER
  LAUNCH --> RSP["robot_state_publisher进程"]
  LAUNCH --> SPAWN["spawn_entity临时进程"]
  LAUNCH --> GUI["gzclient可选进程"]
  subgraph SERVER["gzserver进程"]
    WORLD["既有world + ODE + URDF转SDF模型"]
    VEH["ModelPlugin + xtnetrc_control ROS节点"]
    SENSOR["camera插件 + 两个ROS节点"]
    STATE["state/init/factory插件 + Gazebo ROS节点"]
    WORLD --> VEH
    WORLD --> SENSOR
    WORLD --> STATE
  end
  SPAWN -->|"/spawn_entity"| STATE
  VEH <-->|"ROS2 DDS"| RSP
  SERVER <-->|"Gazebo Transport"| GUI
  VEH <-->|"ROS2 DDS"| TOOLS["控制/显示/录制工具"]
  SENSOR -->|"ROS2 Image"| TOOLS
```

没有新Gazebo的ros_gz_bridge；Gazebo Transport用于server/client，ROS2 DDS用于Topic/Service。启动顺序是launch展开URDF→启动server和RSP→spawn等待服务/描述→插入模型并加载插件→spawn退出。launch中进程可并发，靠等待服务完成依赖，不能假设固定秒数后一切必定就绪。
''')
runtime=json.loads((S/'runtime/competition/runtime.json').read_text())
graph='```mermaid\nflowchart TD\n'+'\n'.join(f'  {v["header"]["frame_id"]}["{v["header"]["frame_id"]}"] --> {child}["{child}"]' for child,v in runtime['tf'].items())+'\n```'
p=D/'04_ROS_GRAPH_AND_INTERFACES.md';p.write_text(p.read_text().replace('TF_GRAPH_PLACEHOLDER',graph))
save('05_RUNBOOK.md',r'''
# 05 启动、观察、控制与停止操作手册

工作区固定 `/home/jamiek/remote5g_ws`。以下[V]命令/入口本次实际成功；[S/H]表示源码推导或历史验证，不冒充本次完整验收。每个终端都要source并选同一ROS域。

## 0 新终端公共环境

[V] 在WSL Ubuntu22.04 Bash中，而不是直接在Windows PowerShell粘贴ROS命令：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=75
```

若 `ros2`找不到，先检查source；不直接重装。不要同终端source autocar覆盖层。`ros2 pkg prefix remote5g_sim`应得到 `/home/jamiek/remote5g_ws/install/remote5g_sim`。

## 1 是否需要构建

已有install已通过本次实际启动，初学第一天**不必重新编译**。将来修改源码才在工作区运行以下[S/H]命令（初始调查未构建，本次用户授权频率修订已成功构建）：

```bash
colcon build --packages-select remote5g_sim remote5g_competition --symlink-install
source install/setup.bash
```

交接目录包含COLCON_IGNORE，避免文档中的package.xml副本被误识别为第三/第四个包。构建失败先读log而不是清空工作区。

## 2 终端A：先启动平坦练习场

```bash
ros2 launch remote5g_sim sim.launch.py scene:=flat gui:=true
```

该入口默认端口11375，默认车(0,0,0.006)、yaw0。本次以gui=false/11405/域95成功启动且实际运动；gui=true平坦组合有历史记录，本次GUI验证使用比赛入口。GUI不负责发车，启动后应静止。

## 3 终端B：确认车辆存在与数据流

在B重复第0步，然后依次运行[V]：

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 topic echo /odom --once
ros2 topic echo /clock --qos-reliability best_effort --once
ros2 topic hz /odom
```

应看到xtnetrc_control、RSP和两个相机节点；odom是nav_msgs/Odometry、frame_id=odom、child_frame_id=base_footprint。初始调查旧配置的hz窗口约19.8Hz；当前目标50Hz，新测量见频率修订报告。hz是持续命令，Ctrl+C结束查看。

看终端A的 `Successfully spawned entity [xtnetrc]`、插件loaded日志；spawn_entity节点随后消失是正常退出。`/cmd_vel`有订阅但没发布者也正常。

## 4 看图、TF、诊断

[V: rqt图像窗口启动；像素接收另由观察程序验证]：

```bash
ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
```

另一个终端可用下列[S/H]已有脚本保存两路相机图（输出目录由你指定；本次独立观察程序执行了相同订阅采集）：

```bash
ros2 run remote5g_sim inspect_cameras.py --output /home/jamiek/remote5g_ws/reports/xtnetrc/learning_cameras
```

[V] TF查询：

```bash
ros2 run tf2_ros tf2_echo odom front_camera_optical_frame
```

刚开始可能等待frame发现，随后应输出变换；本次已收到连续变换。持续查看结束按Ctrl+C。`ros2 topic echo /diagnostics --once`是根据当前接口推导的查看命令[S]；诊断数据在本次runtime.json已实际采到。

没有IMU/激光话题，不运行通用教程里的 `/imu` 或 `/scan` 检查来误判故障。

## 5 终端C：低速有限控制

重复第0步，先确保没有键盘或其他控制发布器。

[V] 阿克曼指令、1秒到时发零：

```bash
ros2 run remote5g_sim drive.py 0.10 0.15 --seconds 1
```

0.10是m/s，0.15是rad中心转角，约8.6°；初始调查平坦场位移0.0941m；当前版本另有频率与停车复测。不要把第二个参数当角速度。零指令[S/H]：

```bash
ros2 run remote5g_sim drive.py 0 0 --seconds 1
```

[V] 另一种方式是系统键盘工具，先停止drive脚本，再运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.05
```

让**此键盘终端**有焦点、英文输入。i前进，u/o前进左右转，逗号倒车，k/空格停车，Ctrl+C停车退出。不能用j/l原地旋转，不能用大写侧移键。按住键的重复事件维持指令，松手0.3s后控制器开始减速；不要把松键描述为立即停车。

初始调查通过PTY发送12次i（10Hz）、k、Ctrl+C，实际位移0.1178m，停车后vx约−0.0000185m/s。转向按键的详细关系及发布验证见06；本次键盘路径只验证前进和停车，没有逐键实测倒车/左右转。

## 6 云台与参数查看

[V] 回中接口：

```bash
ros2 service call /gimbal/center std_srvs/srv/Trigger '{}'
ros2 param dump /xtnetrc_control
ros2 param dump /robot_state_publisher
```

平移/俯仰单角命令[S/H]：

```bash
ros2 topic pub --once /gimbal/pan_cmd std_msgs/msg/Float64 '{data: 0.3}'
ros2 topic pub --once /gimbal/tilt_cmd std_msgs/msg/Float64 '{data: 0.2}'
```

gimbal绝对目标可一次发布；**车辆行驶指令不适合只发一次后期待持续走**，0.3秒租约会过期。参数dump没有max_speed，不代表限速不存在，它在YAML/SDF中，不支持当前ROS动态修改。

## 7 另一次启动比赛场

先Ctrl+C关闭平坦场，另开终端加载环境，选择域85：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch remote5g_competition competition.launch.py track_color:=red start_side:=left lane_change:=random parking_target:=random gui:=true
```

[V 同入口，审计使用自定义输出/端口] 包含挡板、单个弯道障碍、横道、左右标志、架子、四排引导、停车区，车最初在挡板前，不会自动启动。输出默认 `reports/competition/current`，会重新生成派生文件，原轨道world不改。本次只写到handoff/snapshot/runtime/competition/scene。

[S/H 本次未逐一重测] `track_color:=blue`、`start_side:=right`、`lane_change:=left/right`、`parking_target:=A/B`、`obstacle_seed:=42`；切换后同一seed可复现。几何随机不等于每次重启自动换seed。另一实例并行必须换ROS域和Gazebo端口，也要换output避免互相覆盖。

去掉挡板的已有管理入口[S/H]，仅在明确要测试起步时操作，不是第一课必做：

```bash
ros2 run remote5g_competition manage_props.py remove --layout /home/jamiek/remote5g_ws/reports/competition/current/layout.json --name comp_start_barrier
ros2 run remote5g_competition manage_props.py restore --layout /home/jamiek/remote5g_ws/reports/competition/current/layout.json --name comp_start_barrier
```

## 8 正常结束、重启与排障

先k或零指令停止并退出驾驶终端，再在launch终端Ctrl+C，等待gzserver、gzclient、RSP退出；不用killall/pkill，不关闭别人的实例。暂停物理不是关机。下次重新执行launch恢复新仿真；动态锥桶位置由生成seed恢复，别默认reset_world会重新随机化。

安全重启先确认原launch退出、对应端口没有占用。只读检查[S]：`ps -eo pid,args`、`ss -ltn`；不要凭进程名批量杀掉所有Gazebo。用户手动从GUI另存world会改变工程，不在本次调查操作中。

| 症状 | 先检查什么 |
|---|---|
| 包/ros2命令找不到 | source路径、ros2 pkg prefix、是否误用了autocar覆盖层 |
| 节点/图像全不见 | 两终端ROS_DOMAIN_ID是否一致；已有daemon缓存；查看--no-daemon列表 |
| 看到跑道但没车 | 是否只开launch_track.sh；spawn日志与/spawn_entity服务 |
| 指令发了不动 | Gazebo是否暂停、订阅者是否存在、是否单次发布已超时、diagnostics、是否要求原地旋转、是否碰到挡板 |
| 键盘动一下就停 | 本机teleop按键事件发布一次，0.3s租约；持续重复按键或使用有限时长drive.py |
| 图像慢/收不到 | 真实topic含front/gimbal层级、QoS、WSLg/渲染、分辨率和RTF；不要套另一工程/camera/image_raw |
| RViz TF错误 | Fixed Frame=odom、use_sim_time=true、tf_static持久性、是否等待发现 |
| 道具资源缺失 | 新路径重新生成，GAZEBO_MODEL_PATH/PLUGIN_PATH、用户缓存依赖；不要覆盖旧轨道 |
| 端口被占用 | 查原launch归属；换port和ROS域，而不是杀所有实例 |

初始调查未调整配置；本次已按用户授权完成30fps/50Hz配置和发布脚本更新、构建与复测，未重装依赖或重建工作区。所有本次拥有的检查进程已结束，用户原有运行实例保持不动。工具证据保留在snapshot/evidence；持续命令日志退出124通常是本次有界检查主动结束，见验证摘要的分类。
''')
save('06_ROS_TOOLING.md',r'''
# 06 基础ROS工具：按当前工程学习

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

在同一域96另开RViz（use_sim_time=true、Fixed Frame=odom）即可练习看Odometry/TF；本次没有人工验收回放RViz效果。RobotModel还需要URDF描述：离线bag没有robot_description，单放TF并不会自动出现车模型。可在导师指导下另启动**只读robot_state_publisher**使用snapshot/vehicle_competition.urdf，并避免重复发布已回放的关节TF；更简单的初学回放先只看Odometry/TF，不为显示模型额外开整套仿真。

用rqt_plot观察回放的 `/odom/twist/twist/linear/x`，这份样例是静止录制，速度应接近0，不会凭播放出现行驶过程。运动曲线要另录键盘/drive实验。带图像的bag才能播放相机画面，此样例未录图像。

## 数据回放与控制指令回放的区别

传感器/状态回放只是发布旧数据，不重新计算当时的物理。控制回放会驱动任何订阅者，效果还取决于初始位姿/世界/物理/超时。初学回放显式白名单/odom等，排除 `/ackermann_cmd`、`/cmd_vel`、云台命令及任务控制，不在在线车辆域播放控制。未来确需控制复现，另立实验：唯一指令源、独立域、固定初始场景、记录seed/版本及看门狗策略；本次未做控制bag回放。

建议记录格式：日期、实验编号、目的、source版本/文件hash、vehicle和场景配置/seed、ROS域/端口、命令、bag路径、预期/实测、失败与结论。保持配置副本和日志，不用仅凭截图判断闭环是否成功。
''')
save('07_CAPABILITY_GAP.md',r'''
# 07 当前能力与比赛需求缺口

本地规则：`/home/jamiek/remote5g_ws/docs/official/赛项二-室外5G远程驾驶无人车赛比赛规则.pdf`，第二十届、2025年7月、14页。元数据/hash在snapshot/rules_metadata.json。依据本地PDF页2–11任务/评分、页13–14设备表及已查看图1–12；未把网络记忆当新赛季规则。

## 规则与当前实现对照

| 事项 | PDF依据/真实要求 | 当前能力 | 缺口/验证状态 |
|---|---|---|---|
| 自主发车 | p2、p7，移除蓝挡板后自主起步，前轮越起点线计时 | 实体挡板、可移除服务、物理控制 | [N] 蓝挡板识别/移除事件检测、自主发车与计时节点 |
| 白线巡线 | p2–3，用田径白色边界运行约200m | 红蓝跑道白线、前置RGB、CameraInfo/TF | [N] 车道识别与视觉控制；CSV参考线存在不等于已实现跟踪 |
| 弯道避障 | p7，一个随机锥桶，左右绕均可，不能压线/出界 | 单锥桶、随机seed、物理碰撞、双相机 | [N] 检测、位置估计、安全路径与恢复巡线 |
| 横道停车/语音 | p2/p7停10s并播报学校/队名；p5写3s | 准确纸片横道、车能停车 | [N] 检测、计时、语音与恢复；[U] 等待时长冲突须确认 |
| 随机左右标志 | p8，横道后相邻，指示引导区前变道 | PDF素材左右标志、可seed选方向 | [N] 视觉辨识与置信度/时序判定 |
| 引导/归道 | p8–9，按锥桶引导回到原赛道 | 四排引导与原道、回归段几何检查 | [U] 入口空间与实际驾驶未动态验收；[N] 路径与跟踪算法 |
| A/B识别/停车 | p9、p11，A是行进左侧、B右侧，四轮全部入指定区 | 标志/停车框、实际四轮几何检查 | [N] 视觉选位与自动停车，验收接口不是泊车控制器 |
| 完整任务流程 | p2–9顺序1→2→3 | 独立道具与单项测试 | [N] 状态机、切换/去抖、统一指令仲裁与失败恢复 |
| 5G远程画面 | p7，视频回传用于裁判监控，不能据此把比赛当远程人工驾驶 | 本地ROS图像 | [N] 5G网络、远程推流/监控链路与时延验证 |
| 付款 | p3/p9/p11，停车后真人远程扫码付费评分 | 非支付测试QR，仅可读性验证 | [N] 真实支付；用户明确区域2码无已定义车载识别任务，不新增自动扫码任务 |
| 手册与源码 | p9–11，技术手册/原创性/提交要求 | 现有模型与测试文档，本次学习交接库 | 尚非比赛技术手册，不等于参赛提交完成 |

p10–11还明确锥桶接触、压线/出界、引导区未归道、长停、人工干预、停车不足四轮等处罚。未来评分器要依据当届确认规则建测试；不可把本次测试夹具瞬移算自主成功。PDF同时要求变道与“任意位置压线罚时”，变道阶段跨线的裁判口径需确认，不由导师自行解释。

## 每项算法怎么从简单实验进入

以下都是**未来开发建议**，没有新增节点/话题。输入只列本工程真实接口；未来内部感知/规划消息类型和话题名待设计，不把拟议接口写成已存在。

### 1 白线与巡线

- 先学：Image/rgb8与OpenCV的BGR差异、ROI、HSV/灰度阈值、形态学、线段/多项式拟合、左右边界和误差符号；随后学像素透视、K/TF地面投影、简单P再PID/路径跟踪。
- 复用：固定相机Image/CameraInfo、前相机TF、现有车辆限幅与/odom。增加：未来图像订阅/可视化节点，再加唯一控制节点。
- 输入：`/camera/front/image_raw`、camera_info、TF；输出初期离线误差图/数值，闭环才用 `/ackermann_cmd`。
- 单测：先静止数据，画出检测线与ROI；再平坦短直道/原跑道低速闭环。录图像+Info+TF+odom+实际控制，先5–10秒。
- 验证：人工标注与检测误差、误检/漏检、感知延迟；闭环横向误差、碰线次数与稳定时长，不能以“车开起来”代替。真实轮胎接地范围是否压线仍需单独评分器。
- 前置：ROS工具/录放、坐标与控制方向。实车差异：光照、白线磨损、阴影、镜头畸变、安装高度/姿态和延迟。

### 2 锥桶检测、定位与避障

- 先学HSV分色/连通域、面积/形状筛选；用真实数据统计不同距离下像素大小。红蓝场景颜色规则不同，黄锥桶不自动等同危险障碍。
- 复用：两RGB、独立K/TF、实际odom、随机锥桶seed。增加：未来目标检测/相对地面定位与局部避障模块；没有LiDAR可用。
- 输入：相机Image/Info/TF与odom；中间先输出离线bbox/地面点，控制仍只输出 `/ackermann_cmd`。
- 单测：固定seed、不同距离/横向位置，先离线检测，再绕一个障碍，最后恢复车道。录图/TF/Info/odom/控制，Gazebo障碍真值只作评估，不冒充视觉感知。
- 验证：检测召回/精度、米制定位误差、最小车体/轮胎净距、接触次数、压线与恢复误差。单目bbox中心本身不是距离。
- 前置：稳定低速巡线、地面投影与车辆包络/转弯半径。实车差异：小目标、遮挡、反光、地面非平面和滑移。

### 3 横道、等待与语音

- 先学重复白条纹几何、横向排列与路界的区别、触发去抖和有限状态机、ROS仿真时钟与墙钟区别。
- 复用：前相机、已有横道尺寸、odom/clock和零速度入口。增加：未来横道事件模块、停车/等待/恢复逻辑、语音输出适配；当前无语音节点。
- 单测：静态不同接近距离，测试单次触发；停车后计时与重新行驶独立验证；等待时长先向规则负责人确认。录图、clock、odom、控制和未来事件日志。
- 验证：停车前沿距横道、车速归零、实际等待时长、只播报一次/文案正确、不会反复停。仿真暂停不应混成完成等待。
- 前置：稳定停车与计时工具；实车新增音频设备、音量与时钟/通信差异。

### 4 左右标志识别与变道归道

- 先学ROI透视矫正、模板匹配/轮廓几何、置信度与多帧投票，不从深度网络起步。
- 复用：PDF原始标志纹理、随机sign_seed、四排锥桶、前相机和云台。增加：未来标志分类、车道切换规划和轨迹跟踪。
- 单测：独立左右分类；将识别输出先作为日志，再用已知目标方向单测变道，最后连起来。录两路图/Info/TF、odom、控制与选择记录。
- 验证：左右混淆矩阵、触发位置、起始变道距、全车包络与锥桶/边界净距、归道误差。当前回归几何曲率通过不意味着入口变道已可开过。
- 前置：感知稳定、单锥桶避让、曲率/转向变化率、跟踪控制。实车差异：打印白边、光照、舵机迟滞、地面与标志位置。

### 5 A/B与精准停车

- 先学字母模板/投影、车辆局部左右、停车框几何、四轮接地包络，最后学低速路径与停车状态切换。
- 复用：标志/停车区、CameraInfo/TF、odom、`parking_check.py`实际四轮验收。增加：未来停车目标估计与控制，不把base_link在框内当成功。
- 单测：先A/B分类，再已知目标框低速入位，然后感知接入。测试A、B、跨分隔线、车中心在框内但轮胎越界等反例。记录图/TF/odom/控制与四轮检查JSON。
- 验证：四轮角点全部在指定格、无停错格、停止速度与稳定时间；当前0.02m有限接触长度为仿真假设。
- 前置：坐标变换、可靠低速控制与标志识别。实车差异：前方近地遮挡、定位漂移、制动距离和真实轮胎变形。

### 6 统一状态机、远程显示与迁移

- 状态机前先确保每项可独立复现；任务顺序/超时/退出条件均写需求表。唯一控制源或明确仲裁，禁止每个感知模块直接抢cmd_vel/ackermann。
- 可复用现有clock/diagnostics/安全租约，增加未来任务协调/日志/失败恢复模块。视觉与任务状态内部Topic、Message需设计后再发布接口文档。
- 录制完整但有界的实验，按固定场景/seed建立回归集。指标：完成步骤、碰撞/压线、耗时、连续成功次数、CPU/帧率/延迟；随机seed留独立验证集。
- 远程视频作为单独系统试验，先本地观察→录制→局域网→真实5G，逐步测端到端延迟/丢帧，不等于允许远程人工驾驶比赛。
- 最后才考虑实车硬件适配：新可选后端、实体参数标定、人工急停/命令保护、台架到低速小场，不复用Gazebo启动输出GPIO。

## 必须保留的仿真—实车差异

PDF p13设备表写“四轮差速驱动”及摄像头I OV5647/130°等；**当前仿真是后轮共同扭矩假设、前轮被动滚动，前/云台HFOV80°/70°，相机参考名称也来自另一份本地上游记录**。这些是明确差异，不能把“XT-NetRC参考仿真”宣传成官方实车精确复刻。

真实轮距/负载半径、质量/质心、驱动拓扑、舵机/PWM—角度、电调/PWM—速度、镜头内外参、相机固定安装、轮胎摩擦、硬件ROS接口和5G链路均待实测/确认。现有硬件PWM说明不被仿真读取，不能推断已有实车控制功能。

当前仍未编写上述比赛算法；此次仅更新前置采集、现有低层目标/反馈与指令发布频率。
''')
save('08_LEARNING_ROADMAP.md',r'''
# 08 ChatGPT接手后的循序渐进路线

原则：每回合一个可观察的小实验，先目的/原理，再命令，最后让用户报告结果。先学工具与反馈，不把大段代码当教学。用户Linux/Python/C++基础尚未评估，节奏按反馈调整，不承诺固定几天完成。

## 依赖顺序

```mermaid
flowchart TD
  A["A 启动与键盘"] --> B["B Topic / TF / 可视化 / bag"]
  B --> C["C 第一个有界控制节点"]
  B --> D0["D0 静态图像与离线检测"]
  C --> D["D1 低速视觉巡线"]
  D0 --> D
  D --> E["E 锥桶识别/定位/单障碍避让"]
  E --> F["F 横道、标志、变道、停车、状态机"]
  F --> G["G 完整回归与实车接口分析"]
  ENG["从A起记录日志/参数；后学最小Git"] -.-> C
  ENG -.-> F
```

TF/CameraInfo、低速停车和录放是视觉/避障前置。基本实验记录从A就开始，Git基础在C前补上；不等到G才保存结果。实车迁移仍放最后。

## A 熟悉现有平台（不新增代码）

目标：认识当前车型、两个包、ROS消息如何使车运动；能独立开关并解释终端分工。

- A1：按05启动平坦场，只执行node list、一次odom echo。必要概念：Node、Topic、Message、反馈；观察车生成日志与静止odom。
- A2：低速键盘i/k，另一个终端观察cmd_vel和odom。解释命令速度与实际速度、阿克曼不能原地转、0.3s租约。通过：位姿确有小变化、停车后速度近0、可正常结束。
- A3：用已有drive.py发一次有时限的直行/小转角，解释m/s与rad，切换前确保键盘退出。通过：能预测转角符号，不把angular.z当方向盘角。
- 当前可复用：全部已有工具；预计增加：仅用户实验记录，不增加包。

**导师第一回合只教A1。**成功标准：用户提供节点列表、odom的frame/child_frame和坐标，以及说明车当前是否静止。若失败，先排查source/域/日志，等待反馈再继续。

## B ROS工具与可复现数据（不新增算法）

目标：看到数据来源、帧关系、实际频率，能录一个短实验再安全回放。

- B1：topic info/echo/hz；查看RELIABLE相机与BEST_EFFORT clock，观察发布者数。通过：分清topic存在、消息到达和频率。
- B2：rqt_image_view先看前相机再云台，确认rgb8、尺寸和各自CameraInfo；保存一帧观察白线/遮挡。通过：能指出当前640或1280模式。
- B3：RViz Fixed Frame=odom，加TF、RobotModel、Image、Odometry逐项验收，保存并重载配置。通过：无错误帧；能解释为何没有/scan。
- B4：rqt_graph与04图对应；rqt_plot画目标与实际速度；rqt_console找一次command_timeout。通过：能从命令追到执行和反馈。
- B5：先静止5秒bag，再录小段键盘运动；检查metadata，独立域只回放状态，使用单一clock。通过：读出topic/count，解释静止bag为什么不显示运动，不向在线车回放控制。
- 预计增加：用户自己的小bag、记录和RViz配置；不要立即引入数据库/容器/完整监控平台。

## C 第一个简单控制程序（到此才开始代码）

前置：会A/B，掌握最小Python函数/循环、读日志，了解Git基本提交。

- C1：先解释rclpy Node、Publisher、Timer与退出保护，设计一个限时低速控制节点，只选 `/ackermann_cmd`。明确输入配置、输出、50Hz定时发送、停止条件；让用户亲自实现小段代码，不一次生成完整工程。
- C2：订阅odom作记录，比较命令与响应；练习时间同步。通过：无人操作时有界结束，finally发零，异常仍能依赖插件租约停车。
- C3：只在平坦短段做“已知距离前进并停车”的简单闭环，解释位置/速度反馈与阈值，不用瞬移达成目标。通过：误差定义、单位正确、重复3次并记录，不能凭一次成功认定稳定。
- 预计新增：一个小型学习节点及参数/日志；是否新建教学包待用户与导师设计，当前不存在该包。只修改解释清楚且用户愿意亲自学习的文件。

## D 视觉巡线

前置：会图像/录放、相机坐标、误差符号和有界控制。

- D0：从已存PNG/短bag离线读一帧，解释RGB/BGR；画ROI与阈值结果，先人工检验白线而不发任何车辆命令。
- D1：分离左右白边、估计中心，记录像素误差；处理只看见一侧线的失败，区分黄胶带/横道。
- D2：解释当前K和TF，再选图像域误差或地面投影路径。先P，观察振荡/延迟，再讨论是否需要PID或Pure Pursuit；不是先套复杂框架。
- D3：低速短直道闭环→大弯道→红蓝背景与不同起点。通过：预先定义误差/压线/失线停车标准，录图与控制，至少多个重复实验。
- 预计新增：未来感知节点、单一跟踪节点、离线测试样例和参数文件；具体Topic/Message未定义。

## E 锥桶与避障

前置：巡线稳定、车体/轮胎包络、相机地面投影与最小转弯半径。

- E1：静止图像按红/蓝/黄分色，统计bbox与不同距离召回，先搞清楚黄色引导和单障碍的语义。
- E2：单目相对位置估计，使用Gazebo真值**只评分**，不让控制读取隐藏真值冒充感知。
- E3：手工已知目标位置的简单可行绕行轨迹→接入视觉→回到巡线，考虑车宽/转向速率与不压线。
- 通过：检测/定位误差可量化，保持最小净距、不碰锥桶、恢复车道；固定训练seed外，再选独立seed验证。
- 预计新增：未来目标定位、局部规划、跟踪参数与回归样例；不新增不存在的激光雷达作为捷径。

## F 剩余任务，拆开验收再组合

顺序建议：横道检测/停车→等待/语音→左右标志分类→单独变道归道→A/B分类→指定框低速停车→统一状态机。

每项先静态感知，再已知目标动作，最后连接；输入输出/触发/失败回退写成小表。横道等待秒数与跨线口径先确认规则；暂停实验要区分ROS计时和墙钟看门狗。

通过标准：误识别/重复触发有测试，左右/A/B混淆矩阵可读，停车用四轮接地判断，状态机能按顺序完成并有明确退出/超时。完整运行才统计完成率、碰撞/压线、耗时；当前工程尚无完整通关记录。

预计新增：未来任务协调、感知事件、语音适配、控制仲裁与集成日志。比赛区域2 QR不自动增加成未规定任务。

## G 工程化与实车迁移

完整仿真前已有最小Git、参数副本和实验记录；本阶段完善包分层、单元/集成回归、独立验证集、性能指标与技术手册。

先做软件接口清单：图像编码/K/TF/频率、速度/转角单位与时延、里程计真实性、驱动拓扑/传感器差异；实车相机固定安装与官方设备要求另外确认。新增独立硬件适配后端只能在明确授权后做，不能把Gazebo参数或PWM说明直接接到实体执行器。

通过：同一高层模块在已定义接口下可替换仿真/硬件；有标定、急停、超时和分阶段验证计划。先台架、小场低速，再扩展；不以仿真成功推断实车安全。

## 每次实验固定交付

1. 用户用自己的话说明目的和必要概念。
2. 记录环境/域/场景/seed、版本或hash、参数、命令。
3. 留可观察证据：节点/Topic、图、短bag、量化结果与错误日志。
4. 对照通过标准，失败先定位原因，成功也说明限制。
5. 复盘一个知识点，更新实验记录，再决定下一小步。
''')
save('09_ENGINEERING_PRACTICES.md',r'''
# 09 软件工程与实验管理现状

初始任务只调查；本次用户另行授权更新30fps/50Hz配置、指令定时发布与TF上限，未重构包或重装依赖；当前上传任务另获用户授权，已初始化Git/main并整理忽略规则。

## 现有基础

| 项目 | 当前事实 | 如何使用/边界 |
|---|---|---|
| Git | [V] 当前工作区Git仓库分支main；已推送至GitHub私有仓库JamieK32/remote5g_ws | 用git status、git log、git remote -v查看状态；远端地址及可见性验证见snapshot/repository_setup.json |
| 源码分层 | [S] sim与competition两包分离，插件/模型/场景/测试分目录 | 仿真复用sim，不复制车辆代码；视觉/任务包尚未创建 |
| 配置 | [S] vehicle.yaml、competition.yaml、parameter_sources.yaml、派生resolved.yaml | 机械和任务场景参数分开；来源性质已标注，未实测不得改称实测 |
| 依赖 | [V/S] package.xml/CMake明确ROS依赖；QR脚本固定缓存包版本 | 部分依赖在工程外用户缓存，缺乏完整机器可复现环境锁；本次不安装 |
| 构建/启动 | [V/S] colcon/ament，已有install；两个launch，轨道独立脚本 | 修改源码后才编译；单开轨道不等于插车 |
| 日志 | [V/S] colcon log、launch控制台、diagnostics、rosout、reports日志 | 当前可用于排障，缺统一实验索引/保留策略 |
| 单元测试 | [S/H] control_math_test.cpp；test_geometry.py；model_check.py | 历史有成功证据，本次没有重跑colcon test |
| 仿真集成 | [S/H] scripts/test_xtnetrc.sh、regression/smoke、run_matrix、integration/CLI | 会发控制/建删夹具，只在独立仿真运行；不盲目在用户在线场景启动 |
| 本次小验证 | [V] 两场景启动、真实接口、低速Ack/键盘、短bag录放、GUI启动 | 不是完整比赛回归，更不是自主算法成功 |
| 录放数据 | [N/S] 原data基本预留；本次新增303KiB审计bag和方法 | 尚无按任务/版本组织的视觉数据集 |
| 参数版本 | [S] source分类、resolved YAML、随机seed和坐标输出 | 原调查以文件hash留证；当前Git仓库建立后，新实验再记录对应commit |
| 结果管理 | [S/H] reports/xtnetrc、reports/competition含成功/失败历史 | 应读本次/指定修正版，不能只挑PASS或混入旧布局结果 |
| 性能/验收 | [S/H] 物理/几何/接触/四轮停车及相机检查 | 尚无白线误差、避障完成率、端到端任务性能基线 |
| CI/分支流程 | [V/N] 已有本地main；未建立CI或正式协作分支约定 | 先本地小测试，不第一天引入复杂流水线 |

## 最小改进顺序（建议，未执行）

1. 从下一次小实验开始，在data/experiments下用“日期_阶段_编号”建记录；留一张图、短日志/短bag和结论即可。
2. C阶段编码前，导师先解释Git工作区/暂存/commit，在现有main仓库练习git status/diff/log并审阅.gitignore，不重复初始化。别把data大bag、build/install/log、密钥和海量生成资源全提交。
3. 第一份简单程序只增加一个清晰模块和少量参数；先写输入输出/单位/停止条件，再代码/单元测试。
4. 检查通过后再commit，记录commit或源文件hash到实验目录。失败实验也保留，不改旧日志伪装成功。
5. 等有多个算法模块时，再规划感知/规划/控制/任务包、统一接口/仲裁和回归入口，不现在大改已有两包。

建议后续简易分支：主干保留可复现状态，某实验一个短功能分支，小commit说明“问题、变化、如何验证”。main已在本次GitHub上传整理时建立；功能分支和团队协作约定仍为未来建议。

## 测试分层与命令状态

[S/H 未在本次执行]：

```bash
colcon test --packages-select remote5g_sim remote5g_competition --event-handlers console_direct+
colcon test-result --verbose
```

只在专用空闲实例跑已有仿真回归：`scripts/test_xtnetrc.sh`或 `ros2 run remote5g_competition run_matrix.py ...`。测试脚本有端口检查、超时和拥有进程组清理，仍应先理解它们会如何驱动车辆/操作夹具。不要把所有测试当只读命令。

初始调查没有重新编译；当前频率修订已经colcon build两包，并执行现有两项单元/几何测试及独立运行复测。尚未证明干净新机器可重复构建，也未重跑完整比赛矩阵。

## 最小实验记录模板

```text
实验ID / 日期 / 问题与假设
源码commit或文件hash：
world / vehicle配置 / 相机尺寸 / seed：
ROS_DOMAIN_ID / Gazebo端口 / use_sim_time：
唯一控制源 / 命令 / 初始条件：
预期与通过标准：
数据路径（短bag/日志/图）：
实测指标、失败现象与限制：
结论、知识点、下一步：
```

流程：需求→概念学习→小方案→用户独立实现→仿真测试→数据记录→分析→Git提交→文档归档→下一阶段。

## 本次文档隔离与文件安全

- 初始调查只创建交接资料；本次授权变更清单见snapshot/frequency_update_20261015.json，涉及6个源码/配置/说明文件。几何、原world、执行器限制和依赖环境未改。
- 初始调查保存986个文件hash；当前仅允许频率修订清单中的6项变化，其他初始文件继续核验。交接源副本被COLCON_IGNORE隔离，不参与工作区包发现。
- 只导出允许列表环境变量/源码配置；没读取密码、Token、SSH私钥、浏览器资料或用户完整env。实车PWM说明仅留用途/路径，不导出无关硬件值。
- 轻量交接包排除构建缓存、大DAE/纹理、旧分发zip及bag数据库；资源仅用路径/引用/hash描述，原始小bag留本地。
- `snapshot/collect_*`、`probe_runtime.py`等是本次审计工具，不是新增比赛算法；不会被ROS包构建或常驻运行。运行检查会建自己的仿真并发小指令，未经理解不要在别人在线实例上重复运行。
''')
save('10_OPEN_QUESTIONS.md',r'''
# 10 未知、未完成检查与需要补充的信息

以下不是阻止阅读知识库的理由，也不授权重装/改模型。当前启动、控制、传感器接口均有实测证据。

## 已解决的前提

- 用户已确认主工程是 `/home/jamiek/remote5g_ws`，不是autocar_ws。
- ROS2 Humble、Classic11、两个包、真实双控制入口、双RGB、实际节点与TF已确定。
- 键盘工具已装且可驱动车，不再把它写成“没有键盘控制能力”；只是没有本工程专用封装脚本。
- 回放在隔离域96收到odom与clock；不是只根据bag软件安装作推断。

## 应向用户或规则负责人确认

| 优先级 | 明确问题 | 为什么影响后续 |
|---|---|---|
| 比赛方案前 | 目标参加哪一年/届的赛事？本地2025年第20届PDF是否仍适用，有无新通知？ | 本次没有查最新赛季，旧PDF不是自动有效规则 |
| 横道计时实现前 | p5的3s与p2/p7的10s，以哪份最新正式解释为准？ | 不在教学示例中擅定比赛等待秒数 |
| 变道验收前 | 必须变道跨白线与p10“任意位置压线罚时”的适用阶段/判定口径是什么？ | 规划与评分器不能自行互相矛盾 |
| 制定教学节奏时 | 用户Bash、Python、C++、向量/三角函数的熟悉程度如何？ | 路线已保留基础补课，不默认用户都会 |
| 实车迁移之前 | 实际XT-NetRC版本、传动拓扑、相机型号/安装、轮距/负载半径、控制后端与标定是否可提供？ | PDF、上游参考与当前仿真有差异，目前没有硬件适配实现 |

用户已明确区域2二维码暂无明确车载识别任务，此事实已采纳，不再要求用户重复确认。将来规则变化再更新，不以当前测试码发明支付/自动扫码任务。

## 运行/工具检查的限制

| 检查 | 本次结果 | 后续如何补齐 |
|---|---|---|
| ros2 doctor --report | [U] 35s未结束，主动超时，原因未定位 | 有需要再单独查看卡在哪一项；先保留成功启动和日志，不直接重装 |
| RViz | [V] 程序/OpenGL启动，[U] RobotModel/TF/Image/Odom逐项人工配置、保存/重载未验收 | B阶段由用户亲自逐项设置并反馈 |
| rqt | [V] 插件枚举、image_view启动；[U] graph/plot/console/reconfigure GUI功能逐项未操作 | 按06一个插件一个实验 |
| 完全无显示环境 | [U] 本次运行依赖WSLg，即使gui=false | 若以后部署服务器再设计渲染方案；不现在安装新组件 |
| 相机实时性能 | [V/H] 前置当前目标30fps、云台15fps，最新实测见修订报告；旧配置低接收率窗口仅作历史 | 分别检查配置、仿真时间帧率、墙钟接收频率和RTF，不把目标帧率当硬实时保证 |
| 传输压缩 | [S] 广告compressed/theora/compressedDepth；[U] 未分别收包或解码 | 先学raw；compressedDepth广告不是深度传感器 |
| 四排引导实际通行 | [S/H] 回归曲线静态几何/历史加载；[U] 实际入口变道/全段驾驶尚未验证 | 完成学习D/E后先已知目标轨迹单测，不先宣称符合全部比赛评分 |
| 整套回归/重编译 | [V/H] 本次频率修订已构建并执行两项单元/几何测试；完整矩阵仍为历史 | 源码变更后另立专用实例执行；不抹去历史失败 |
| 干净环境复现 | [U] 使用现有install与用户QR缓存 | 未来明确依赖清单/锁定与新机复现；本次不安装 |
| 实体车与5G | [N/U] 当前工程未找到硬件执行器、5G视频/音频系统与实车验证 | 最后单独规划，不从GPIO说明文件推断可用 |

## 文档事实冲突及处理

1. 根README原有“无车辆包”旧文字已在GitHub发布整理时更新；初始调查记录仅作历史，当前以源码与新README为准。
2. 模型参考名与“上游measured”不等于本次对实车实测；轮距、质量、摩擦、机械限位有仿真假设。
3. PDF p13四轮驱动/相机130°等与当前后桥驱动假设/HFOV80°和70°不同；保存差异，不为了表面一致而改模型。
4. 所有车辆控制/测试脚本与实际操作能力不代表已有自主比赛算法；停车判定接口不是自动泊车程序。
5. 检查持续流工具使用有界超时，tf_echo/hz已拿到有效输出后结束；GUI启动检查也是主动结束。它们的124不能统一当作FAIL；doctor未完成则不能标PASS。
6. 初次导出源码副本曾被colcon扫描为重复包，已用交接目录COLCON_IGNORE隔离并复核只剩原src两包；该隔离处理没有修改原包。后续30fps/50Hz修订另经用户授权并已构建。

## 接手后的核对规则

若用户后续改了模型/配置/launch/话题，先重新查询实物和source，不照抄旧快照。问具体缺失信息，禁止自行补出传感器、改变ROS版本或选另一个工作区。当前资料是2026-10-15的快照，不是永久在线状态；本次全部检查进程已关闭。
''')
save('11_HANDOFF_TO_CHATGPT.md',r'''
# 11 可直接复制给网页版ChatGPT的说明

下面整段可作为新对话的第一条消息，配合00或合并文档上传。

---

请担任我的ROS2、机器人算法与工程实践导师。我的最终目标是全国大学生智能汽车竞赛室外5G远程驾驶，但我想真正掌握原理、工具、调试、实现和实验管理，不是直接拿一整套比赛代码。

**请把我上传的工程交接文档作为事实基线。**这些资料由实际本地检查导出，状态[V]本次运行、[S]源码、[H]历史、[U]待验证、[N]未找到。不要把历史记录/静态配置/未来建议冒充当前已实现。你不能直接访问我电脑，我会亲自执行命令并贴回结果；需要新事实时请让我查询，不要声称你已经在本地验证。

我的主工程是 `/home/jamiek/remote5g_ws`，Ubuntu22.04.5 WSL2，ROS2 Humble、Gazebo Classic11.10.2、colcon/ament_cmake，Python3.10/OpenCV4.5.4，两个包remote5g_sim与remote5g_competition。不要从零重建仿真，不要换ROS版本/车型，也不要混入另一个autocar_ws的单路相机/激光接口。stupid_car只是本地文档明确引用的上游参数参考，不是我的当前工程。

当前模型xtnetrc是4轮前轮阿克曼转向、后轮共同扭矩的仿真假设，主控制 `/ackermann_cmd`（AckermannDriveStamped，m/s和中心转角rad）；也有 `/cmd_vel`（Twist，vx与wz，由插件换算），不能原地旋转。0.3秒墙钟未收到新有效命令即减速停车，软件上限0.6m/s/25°。先用0.10m/s；同一时刻只运行一个控制源。

有两路RGB：`/camera/front/image_raw`与`/camera/gimbal/image_raw`，各自camera_info和optical TF，云台可控；无LiDAR、深度、IMU、GPS、SLAM或Nav2。`/odom`是Gazebo真值，`/joint_states`是真实关节状态。sim入口默认640×480，competition入口默认1280×720；不能把K混用。compressedDepth话题广告不等于深度传感器。

常驻节点有/gazebo、状态插件、/xtnetrc_control、/robot_state_publisher、两个camera_ros。当前已有动力学、跑道/比赛道具、手动/限时操作和测试；没有视觉巡线、锥桶识别避障、横道语音/等待、随机标志识别、自主变道/停车、统一比赛状态机或实车后端。不要一次生成这些模块。

我希望按A平台→B工具/TF/RViz/rqt/bag→C一个简单有界控制节点→D视觉巡线→E单锥桶避障→F其余任务与状态机→G工程化/迁移学习。实验记录从A开始，Git基础在C编码前补充。遇到Python/Linux/数学前置缺口，先教最小必要概念，不假定我已掌握。

请遵守教学节奏：

1. 每回合只布置一个适量实验，先讲目的、必要概念与原理，再给当前环境适用的操作。
2. 说明每条命令在哪个终端、是否要source、ROS域、单位、是否持续运行及如何停止。
3. 给出可观察的预期结果和通过标准，区分控制目标与实际反馈。
4. 给我时间亲自操作，等待我回复结果再继续；不要一口气把A到G全教完。
5. 出错先检查节点、Topic、消息、QoS、参数、TF和日志，不默认重装环境、删build或改模型。
6. 需要写/改代码时，先解释原因、文件、输入输出、单位、停止保护和验证方法，再让我逐步实现。
7. 视觉先从保存图像/短bag的ROI、颜色阈值、形态学、简单几何开始，先离线可解释验证，再闭环，之后才讨论更复杂算法。
8. RViz负责显示，ros2 bag负责录放；回放状态用独立ROS域和单一clock，排除控制指令，不向运行中车辆回放cmd_vel/ackermann。
9. 当前max_speed等是YAML/SDF配置，不是可任意rqt动态修改的ROS参数。
10. 每阶段帮我复盘概念、实验结果、限制、文件/参数/数据与Git记录，形成独立调试和工程管理能力。
11. 本地PDF是2025年第20届，未确认当届适用；横道3s/10s与变道跨白线口径待确认。用户已明确区域2二维码无明确车载识别任务，不追加虚构任务。
12. 当前模型与官方设备表的传动/相机规格有差异，实车还需真实标定和独立后端；不将仿真成功直接等同实车可用。

**现在请只带我完成A1：启动平坦练习场，确认节点与一条odom反馈。**我不需要你重新设计工程，也先不要让我写巡线算法。

推荐启动环境与命令：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=75
ros2 launch remote5g_sim sim.launch.py scene:=flat gui:=true
```

你先解释Gazebo与Node/Topic/Message各扮演什么，再给少量检查步骤与预期，等我反馈后进入键盘控制。完整命令/证据以05/06及snapshot为准；gui=true平坦入口本次未重测，平坦服务端与比赛GUI分别有实际成功记录。

---

## 给用户的上传建议

最少先传 `00_START_HERE.md`和本文件；完整上下文直接传同目录 `CHATGPT_FULL_CONTEXT.md`。需定位源码再补轻量text.zip或snapshot/source中的特定文件；包内保留相对目录，不包含秘密、大型模型或整工作区。

导师应优先看05_RUNBOOK与10_OPEN_QUESTIONS，先确认用户没有旧实例/不同ROS域，不凭快照猜实时运行状态。当前检查创建的Gazebo和控制工具都已正常结束。
''')
# Keep the directory tree literal and avoid an ambiguous brace suffix.
p=D/'02_PROJECT_STRUCTURE.md';p.write_text(p.read_text().replace('test/{control_math_test.cpp,model_check,regression,smoke}.py','test/（control_math_test.cpp, model_check.py, regression.py, smoke.py）'))
p=D/'06_ROS_TOOLING.md';t=p.read_text();a=t.index('在同一域96另开RViz');b=t.index('\n\n用rqt_plot',a)
t=t[:a]+'''在同一域96另开RViz（use_sim_time=true、Fixed Frame=odom）可练习看Odometry/TF；本次没有人工验收回放RViz效果。RobotModel还需要URDF描述：这份bag没有robot_description，单放TF不会自动出现车模型。初学样例先只看Odometry/TF；以后录制时一并录 `/robot_description`，确认其TRANSIENT_LOCAL持久性，再练习离线RobotModel。不要为显示模型重启整套Gazebo，也不要在回放TF的同时另起RSP重复发布同一批TF。''' +t[b:];p.write_text(t)
print('12 handoff documents written to',D)

# APPLY_FREQUENCY_REVISION_NOTICE
notice="配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。"
for file in D.glob('[0-9][0-9]_*.md'):
 text=file.read_text();first,rest=text.split('\n',1);file.write_text(first+'\n\n'+notice+'\n'+rest)
