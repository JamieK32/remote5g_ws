# 01 实际开发环境

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

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
