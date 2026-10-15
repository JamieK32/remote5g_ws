# XT-NetRC 智能车 Gazebo 仿真与学习工程

基于现有 **ROS 2 Humble + Gazebo Classic 11** 的四轮阿克曼车仿真平台，用于学习机器人调试工具、视觉与控制，并逐步准备全国大学生智能汽车竞赛室外5G远程驾驶无人车赛。

当前工程位于 `/home/jamiek/remote5g_ws`。这是独立仿真工程；`stupid_car`仅是已有文档记录的几何/参数参考，不是本仓库的实车运行后端。

## 当前能力

- `remote5g_sim`：`xtnetrc`车辆、真实物理转向/轮胎接触、双RGB摄像头、两轴云台、里程计/关节状态/TF、有限时长驾驶工具与测试。
- `remote5g_competition`：红蓝400m跑道的独立比赛道具布局；蓝色发车挡板、单个随机弯道障碍、横道、左右标志、四排引导锥桶、A/B停车区和非支付测试二维码。
- 前置采集目标 **30fps**，速度/转角目标及反馈 **50Hz目标**，`drive.py`以 **50Hz墙钟定时发布**。云台RGB仍15fps。仿真时间频率与墙钟接收频率取决于RTF和负载，不保证硬实时。
- 本地实际验证过场景启动、双相机/CameraInfo/TF、有限低速驾驶、键盘控制、停车和短bag录放；当前路径跟踪、视觉巡线、避障与比赛状态机尚未实现。
- `learning/`是用户的阶段性练习草稿，不作为已验收的比赛算法；其中参数与行为应在运行前自行核对。

## 环境与目录

本机验证环境：Ubuntu22.04.5 WSL2、Python3.10、OpenCV4.5.4、GCC11、colcon/ament_cmake。不是ROS1、catkin或新Gazebo。

```text
src/remote5g_sim/          车辆、插件、参数、启动与测试
src/remote5g_competition/  比赛模型、布局生成、管理与测试
simulation/              原红蓝跑道模型、world、参考线CSV
config/                  原跑道几何参数
scripts/                 跑道与回归辅助脚本
learning/                用户练习草稿
docs/chatgpt_handoff/    工程知识库、学习上下文与小型证据
docs/official/          用户提供的比赛规则与轨道参考PDF
```

`build/`、`install/`、`log/`、`reports/`、大bag、编辑器索引、交接运行时资源和分发压缩包是本地产物，Git忽略它们。保留小型源码、配置、模型与交接证据；不要把Git当运行日志或大数据存储。

## 编译与启动

已有ROS2 Humble/Gazebo Classic11依赖的WSL Ubuntu终端中：

```bash
cd /home/jamiek/remote5g_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select remote5g_sim remote5g_competition --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=75
ros2 launch remote5g_sim sim.launch.py scene:=flat gui:=true
```

该命令加载已有平坦场与车辆，默认静止。正常退出在launch终端按Ctrl+C，等子进程结束。

另一个终端加载相同环境及域75，可执行：

```bash
ros2 topic echo /odom --once
ros2 run remote5g_sim drive.py 0.10 0 --seconds 1
```

主接口 `/ackermann_cmd` 是 AckermannDriveStamped：speed=m/s、steering_angle=中心转角rad。兼容 `/cmd_vel` Twist，经插件转换；不能原地旋转。只运行一个驾驶命令源；0.3秒墙钟无有效车辆指令后按限制减速停车。

比赛场景另一次启动：

```bash
export ROS_DOMAIN_ID=85
ros2 launch remote5g_competition competition.launch.py track_color:=red start_side:=left gui:=true
```

车辆与比赛入口默认Gazebo端口11375/11385。并行实例还需不同ROS域、端口和输出目录。比赛入口生成独立overlay，不覆盖原轨道world。

## 克隆与依赖注意

当前launch、比赛YAML和部分工具中保留本机 `/home/jamiek/remote5g_ws` 默认路径。本仓库尚未完成任意路径的全局移植：换用户/路径时先核对workspace参数与配置，再由生成器重新生成派生资源，不能认为旧absolute file URI自动可用。

源树中的部分模型/派生场景包含本机绝对URI。比赛launch会在输出目录重新生成道具资源；原跑道使用model://引用。未上传的 `reports/`历史图片/大日志链接只在原工作区有效，交接目录保留便携的小型文本证据。

比赛布局用qrcode8.2、zxing-cpp2.3.0；本机使用用户缓存 `.cache/remote5g_scene_libs`。新环境需先审阅 `src/remote5g_competition/scripts/bootstrap_deps.sh`及依赖说明；本仓库不包含用户缓存、密码、Token或实体车凭据。

## 文档与学习入口

- [工程交接入口](docs/chatgpt_handoff/00_START_HERE.md)
- [完整ChatGPT上下文](docs/chatgpt_handoff/CHATGPT_FULL_CONTEXT.md)
- [给ChatGPT的教学说明](docs/chatgpt_handoff/11_HANDOFF_TO_CHATGPT.md)
- [30fps / 50Hz修订与实测](docs/chatgpt_handoff/FREQUENCY_UPDATE_20261015.md)
- [车辆说明](src/remote5g_sim/README.md)
- [比赛场景说明](src/remote5g_competition/README.md)

从启动、Node/Topic/Message、键盘、RViz/rqt、TF和ros2 bag开始，再逐步实现算法。交接目录的COLCON_IGNORE用于防止源码副本被当成重复ROS包，必须保留。旧调查与旧测试有明确历史标签，不应混作当前配置。

本地PDF是2025年第20届资料，不代表已经确认适用当前赛季。原文的等待时长/压线等冲突及仿真—实车差异见交接手册。30fps参考用户提供产品参数；50Hz为用户选定的仿真设计值，并非手册规定的控制频率。

## 测试与资料权利

```bash
colcon test --packages-select remote5g_sim remote5g_competition --event-handlers console_direct+
colcon test-result --verbose
```

集成测试会启动仿真、发控制并操作夹具，先读各包README，使用空闲隔离实例。不要在正在驾驶的场景盲目运行回归。

两个新增ROS包的代码许可见各包LICENSE。用户提供PDF、PDF内嵌标志和第三方参考资料保留其各自权利，不因代码许可而重新授权。
