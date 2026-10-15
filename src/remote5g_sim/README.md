# XT-NetRC 阿克曼 Gazebo Classic 仿真

独立工作区 `/home/jamiek/remote5g_ws`，ROS 2 Humble / ament_cmake / colcon，Ubuntu 22.04 WSL2，Gazebo Classic 11.10.2。
新包 `remote5g_sim` 集中维护模型、C++ 物理控制和测试，不复用 `/home/jamiek/autocar_ws` 的源码或 install 环境，不修改现有 400m 跑道。
验收数据、限制与故障修复见工作区 `docs/GAZEBO_TEST_REPORT.md`。

## 环境与文件

依赖为现有安装的 gazebo11/libgazebo-dev、gazebo_ros、gazebo_plugins、rclcpp/rclpy、ackermann_msgs、geometry_msgs、nav_msgs、sensor_msgs、std_msgs/std_srvs、diagnostic_msgs、tf2_ros、robot_state_publisher、xacro、Qt5、Python yaml/numpy/opencv。未使用 sudo 安装。

```text
src/remote5g_sim/
  urdf/{car,common,wheels,cameras}.xacro  车辆结构/惯性/碰撞/传感器
  config/vehicle.yaml                   所有主要机械、控制和相机参数（SI）
  config/parameter_sources.yaml         measured/reference_document/source_default/simulation_assumption
  config/hardware_reference_NOT_LOADED.yaml  仅供文档核对的实车 PWM；启动不读取
  src/vehicle_plugin.cpp                Gazebo ModelPlugin，ROS 2 控制与实际状态
  src/gui_capture.cpp                   可选 gzclient 原生视口截图工具
  include/remote5g_sim/control_math.hpp  独立几何/限速计算
  launch/sim.launch.py                  平坦/红跑道/蓝跑道入口
  worlds/flat.world                     平面、非对称视觉标记、实际碰撞
  scripts/drive.py                      有时限的 50Hz 车辆操作
  scripts/inspect_cameras.py            保存两路实时图像并检查 CameraInfo
  test/{control_math_test.cpp,model_check.py,regression.py,smoke.py}
  docs/{REFERENCE_REVIEW.md,reference_manifest.json}
scripts/test_xtnetrc.sh                 300秒总超时/进程组清理的服务端回归
reports/xtnetrc/                        日志、JSON、实际轨迹 CSV、相机/GUI 图像
```

## 坐标、动力学与转向

X 前、Y 左、Z 上，单位 m/rad。`base_footprint` 位于后轴正下方地面参考点，`base_link` 在同一后轴中心、垂直偏移一个轮半径 0.055315m；两者没有第二个平面车辆原点。
平坦地面上的 footprint 是地面投影；遇坡或碰撞时本模型使用刚性体轴下参考点并保留真实 roll/pitch，不假装始终位于水平地面。
`odom -> base_footprint` 读取实际 Gazebo base_link 世界位姿并减去旋转后的轮半径偏移；速度也含角速度引起的参考点修正。没有理想运动学积分，没有 SetWorldPose 驱动车辆。其 odom 是仿真 ground truth，不是实车编码器估计。
robot_state_publisher 根据实际 `/joint_states` 发布关节 TF 及固定 TF，避免重复发布。

四个圆柱轮胎都有惯性和碰撞；前滚动关节为被动轮，前两个 Z 轴转向关节使用限扭矩 PD，前轮接触约束真实改变运动方向。车架采用参考尺寸盒，并有前后轮轴横梁与立柱连接；支架/相机也有碰撞和惯性；主要接触设置 mu=1、kp=1e6、kd=10，世界步长 1ms。
未实测的底盘盒高度/质心、轮距、质量和简单支架几何都属于仿真假设。该盒不是精确 CAD，当前整车含轮胎/云台的包络大于 410×190×105mm。

对自行车中心转角 δ：`k=tan(δ)/L`，`δ_left=atan2(tan(δ),1-T*tan(δ)/(2L))`，`δ_right=atan2(tan(δ),1+T*tan(δ)/(2L))`。
左转内轮更大，右转相反；中心转角上限 25° 是软件初值，两转向关节仿真硬限位 ±0.6rad 可容纳内轮角。
后桥采用单虚拟电机/开放差速器的仿真假设：读取两轮平均速度，输出完全相同扭矩给两半轴，转弯时不同滚动速度由地面物理约束产生；没有左右轮独立命令，没有差速转向或原地旋转。

## 参数与参考来源

上游固定为 [stupid_car commit bec5112](https://github.com/kxkxkxa767/stupid_car/tree/bec5112ea11dca1516251327ce3ad35e3f5a6647)。源码审查见 `docs/REFERENCE_REVIEW.md`。
未找到工程授权声明，所以没有复制其代码/模型/图像/标定文件，仿真代码独立实现。

| 参数 | 值 | 来源性质 |
|---|---:|---|
| 轴距 L | 0.23861m | measured，上游 motion_default.json |
| 无负载几何半径/轮宽 | 0.055315 / 0.04952m | measured，上游规格中的卡尺记录，本次未检查原照片；负载半径待测 |
| 底盘参考盒 | 0.410×0.190×0.105m | reference_document，任务/规则与上游规格 |
| 前/后轮距 | 0.24/0.24m | simulation_assumption |
| 车架质量/轮胎质量 | 3.0/0.18kg | simulation_assumption |
| 中心转角/转角变化率 | 25° / 120°/s | source_default，未确认机械极限 |
| 最高速度/加减速 | 1.5 / 1.0 / 1.5（SI） | simulation_assumption，2026-10-15用户选定的仿真目标限制 |
| 曲率限速 | max(0.18,1.5/(1+0.8×abs(k))) | 保留曲率限速公式；仍不超过输入目标速度 |
| 命令租约/目标更新周期 | 0.3s墙钟 / 0.02s仿真时间 | 租约为参考默认值；50Hz目标/反馈为2026-10-15用户选择的仿真设计，不是厂商控制频率 |
| Pan/Tilt 仿真限位 | ±π/2 / [-0.6,+0.8]rad | simulation_assumption，未由 PWM 推导 |

逐项来源和设置理由均在 `config/parameter_sources.yaml`，可替换实测值后经 `config:=/绝对路径/vehicle.yaml` 启动。
仿真不加载 pigpio，不写 GPIO12/13/22/23，不停止真实服务，不连接实体车。实车 PWM 中心 72/73 和软件包络保留为说明数据，绝不解释为角度。

## 摄像头与云台

XWF 1080P PC Camera：前部 fixed joint，默认在后轴原点的 `[0.30,0,0.235315]` m，高度含 base_link 偏移；俯视 0.45rad。默认 640×480，HFOV=80°、30Hz；30fps依据用户提供产品资料选择，视场角及安装仍为仿真假设。
H65 USB CAMERA：位于 Pan/Tilt 末端，默认 640×480，HFOV=70°、15Hz，独立话题/内参；1280×720 仅作为可选参考运行配置，不声称硬件模式已核实。
两相机 optical frame 均用 RPY `[-π/2,0,-π/2]`，其 X 右、Y 下、Z 前。Gazebo 输出独立针孔 CameraInfo（零畸变），不使用 XWF 实车单应矩阵或设备身份不明的历史 K。
分辨率变化由 Gazebo 重算 K；外参由 TF 提供。算法接入地面投影时应按当前 K/TF 求射线与地面交点，不能加载实车 ground_projection.json。

Pan 正值左看；Tilt 轴为 -Y，正值抬头。默认启动回中，绝对角命令限位后按 1rad/s 平滑移动，关节使用限扭矩 PD。`/gimbal/center` 设置两轴目标 0rad。重力使 Tilt 存在约 0.006rad 稳态偏差，回中验收容差 0.03rad。

## 控制接口

| 话题/服务 | 类型 | 行为 |
|---|---|---|
| `/ackermann_cmd` | ackermann_msgs/msg/AckermannDriveStamped | speed m/s、steering_angle 中心转角 rad；acceleration 和 steering_angle_velocity 非负，0 使用默认，正值只可减小对应变化率；jerk 有限数检查但不实施 jerk 限制 |
| `/cmd_vel` | geometry_msgs/msg/Twist | δ=atan(L×omega/v)，保留倒车符号；拒绝非平面分量、NaN/Inf、abs(v)<1e-4 且非零 omega |
| `/gimbal/pan_cmd`、`/gimbal/tilt_cmd` | std_msgs/msg/Float64 | 弧度绝对角目标，独立于停车/转向 |
| `/gimbal/center` | std_srvs/srv/Trigger | 平滑回中 |
| `/odom` | nav_msgs/msg/Odometry | Gazebo 实际状态，odom/base_footprint |
| `/joint_states` | sensor_msgs/msg/JointState | 实际位置/速度、施加扭矩（前轮被动努力值为0） |
| `/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | active、command_timeout、invalid_command、unrealizable_twist、controller_exception 及目标/扭矩/租约年龄 |
| `/camera/front/image_raw`、`/camera/front/camera_info` | Image / CameraInfo | 固定循迹相机 |
| `/camera/gimbal/image_raw`、`/camera/gimbal/camera_info` | Image / CameraInfo | 云台相机 |
| `/tf`、`/tf_static` | tf2_msgs/msg/TFMessage | 实际关节和固定结构 |

Ackermann 和 Twist 共用同一最新有效命令租约，后收到的命令生效；上层应只运行一个命令源。
非法车辆指令会清空目标并减速回中。300ms 墙钟无有效指令后清除旧目标，按减速度停车；停车不是瞬间冻结物理位姿。后续必须收到新的有效指令才可恢复。
暂停仿真仍会令墙钟租约过期，恢复时停车；物理 reset 清空车辆及云台目标。配置不合法时插件拒绝加载；运行异常锁定控制并清空目标。

## 编译和启动（本机已执行验收）

新终端：

```bash
source /opt/ros/humble/setup.bash
cd /home/jamiek/remote5g_ws
colcon build --packages-select remote5g_sim --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=75
ros2 launch remote5g_sim sim.launch.py gui:=true
```

该入口自行展开模型、发布 robot_description、启动 Gazebo 并 spawn，无需再次生成车辆。默认 Gazebo 端口 11375，不占用已有跑道的 11365；在该窗口 Ctrl+C 可结束此次 launch 所有进程。
`gui:=false` 启动无 gzclient 服务端，但相机仍需要渲染环境；本次无GUI测试使用 WSLg 的 DISPLAY=:0，不代表完全无显示服务器的部署已验收。

现有跑道独立入口（保持已生成模型/几何）：

```bash
ros2 launch remote5g_sim sim.launch.py scene:=red gui:=false
ros2 launch remote5g_sim sim.launch.py scene:=blue gui:=false
```

默认放在 `(0,-37.2,0.006)`，朝 +X，在内道底直线且避开内侧路沿；可用 `x:=... y:=... yaw:=...` 覆盖。不同入口不可同时使用同一端口；独立实例用 `port:=11376` 和不同 ROS_DOMAIN_ID。

高分辨率和原生 GUI 截图的已测配置：

```bash
ros2 launch remote5g_sim sim.launch.py gui:=true front_width:=1280 front_height:=720 gimbal_width:=1280 gimbal_height:=720 capture_dir:=/home/jamiek/remote5g_ws/reports/xtnetrc/manual_gui
```

capture_dir 是可选验收辅助：让 gzclient 相机跟随车辆斜前侧视角并最多保存8张实际视口图像；不改变机器人位姿，不是生成式图片。

## 操作（另一终端加载同一环境与 ROS_DOMAIN_ID）

`drive.py` 用定时器按50Hz墙钟发送，默认3秒后停车；Ctrl+C也会发停车，异常退出由租约保护。以下入口在实测 smoke 测试中执行了相同参数、1秒时长：

```bash
ros2 run remote5g_sim drive.py 0.2 0 --seconds 1
ros2 run remote5g_sim drive.py -0.2 0 --seconds 1
ros2 run remote5g_sim drive.py 0.2 0.25 --seconds 1
ros2 run remote5g_sim drive.py 0.2 -0.25 --seconds 1
ros2 run remote5g_sim drive.py 0 0 --seconds 1
ros2 topic pub --once /gimbal/pan_cmd std_msgs/msg/Float64 '{data: 0.6}'
ros2 topic pub --once /gimbal/tilt_cmd std_msgs/msg/Float64 '{data: 0.3}'
ros2 service call /gimbal/center std_srvs/srv/Trigger '{}'
ros2 run remote5g_sim inspect_cameras.py --output /home/jamiek/remote5g_ws/reports/xtnetrc/cameras
```

inspect_cameras 保存 front.png/gimbal.png，可直接打开查看同时输出尺寸/帧名/CameraInfo 检查。Gazebo 内部 ROS 图像是 RGB；保存 PNG 会转 BGR，不改变图像像素。

## 自动测试

先关闭使用11375的本包仿真；无需关闭原先11365的跑道。

```bash
ros2 run remote5g_sim model_check.py
colcon test --packages-select remote5g_sim --event-handlers console_direct+
colcon test-result --verbose
./scripts/test_xtnetrc.sh
```

测试覆盖 spawn/TF、静止、前进/后退、左右轮几何、实际轨迹拟合、禁止原地旋转、双图像/K、云台限位/视角隔离、NaN/Inf、墙钟租约、指令恢复、倒车 Twist 和实体障碍碰撞。每个等待/轨迹有界；脚本全程300秒超时，以非零退出码报告失败。
清理只向本次创建的进程组发信号，不用 killall/pkill。端口已占用时拒绝开始；可设 `XTNETRC_TEST_PORT=... XTNETRC_TEST_DOMAIN=...`。
额外场景/高分辨率 smoke：

```bash
ros2 run remote5g_sim smoke.py --scene flat --gui --output /home/jamiek/remote5g_ws/reports/xtnetrc/gui_highres
ros2 run remote5g_sim smoke.py --scene red --output /home/jamiek/remote5g_ws/reports/xtnetrc/track_red
ros2 run remote5g_sim smoke.py --scene blue --output /home/jamiek/remote5g_ws/reports/xtnetrc/track_blue
```

smoke 使用域76/端口11376并清理其launch进程组；建议外加 `timeout --signal=INT --kill-after=10s 150s`。截图只证明已展示的形状和帧；数值物理验收以 CSV/JSON 为准。

## 后续接入与实车差异

感知层读取两路图像/K/TF，固定相机用于 OpenCV 白线识别和当前模型的地面投影；H65 云台用于锥桶/障碍识别。
决策层输出轨迹、速度和状态，Pure Pursuit 可直接以 measured 轴距将曲率转换到 `/ackermann_cmd`。
控制层只接收 speed/steering，执行器层由本 Gazebo 插件负责；未来真实电调/舵机适配应作为单独可选包和单独 launch，不能复用本仿真启动来输出硬件。
GPS/IMU 融合尚未实现；当前 odom 是精确仿真真值，后续可加入噪声和独立状态估计。
比赛整圈、Pure Pursuit 算法、视觉循迹与锥桶状态机未在本阶段实现/验收。现有红蓝400m场景已可生成和控制车辆，不等于比赛能力验收。
真实轮距、负载半径、质量/质心、前后桥驱动拓扑、机械转角、PWM—转角/速度、云台 PWM—角度、镜头内参/安装外参及轮胎摩擦都需要实体车标定；当前盒形车架无悬挂/软胎/精确传动/细节CAD。

## 2026-10-15 频率配置修订

前置RGB目标30fps；速度/转角目标、odom/joint_states/diagnostics及车体TF目标50Hz（control_period=0.02s仿真时间）；两个launch的robot_state_publisher上限50Hz。云台RGB仍15fps，物理步与低层每步力矩反馈不改。drive.py定时发布50Hz墙钟，正常到时和Ctrl+C均取消定时器并发零；目标更新和发布使用不同时间基，RTF偏离1时墙钟观测频率会不同。现有键盘工具仍按键事件发布。路径跟踪算法尚未实现，50Hz是其未来接口/目标预算，并非已有自主跟踪器。

更新后的ChatGPT手册：docs/chatgpt_handoff；实测：reports/frequency_30fps_50hz。旧运行实例必须重启才会加载新参数。
