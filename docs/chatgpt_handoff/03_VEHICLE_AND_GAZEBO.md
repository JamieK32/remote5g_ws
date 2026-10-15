# 03 车辆模型、Gazebo与传感器

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

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
