# Gazebo Classic XT-NetRC 仿真验收报告

日期：2026-10-14（Asia/Shanghai）。工作区 `/home/jamiek/remote5g_ws`。

最终物理回归12/12 PASS，额外高分辨率/GUI及红蓝跑道 smoke PASS。以下均为本次执行数据，不等同实体车验收。

## 环境、版本和文件保护

ROS 2 Humble、ament_cmake/colcon、Gazebo Classic 11.10.2，WSL Ubuntu 22.04.5。实际版本与依赖路径见 [environment.log](../reports/xtnetrc/environment.log)。工作区没有Git仓库，未初始化/提交/推送，故无本次commit hash。未执行sudo安装；现有README、脚本、配置及400m场景全部保留。

参考仓库 commit `bec5112ea11dca1516251327ce3ad35e3f5a6647`，只读源码及授权审查见 [REFERENCE_REVIEW](../src/remote5g_sim/docs/REFERENCE_REVIEW.md)。本包独立实现，不复制未知授权上游代码；没有真实GPIO/硬件输出。

## 验收矩阵

| 验收项目 | 状态 | 证据（均相对 reports/xtnetrc） |
|---|---|---|
| ROS/Gazebo 环境 | PASS | environment.log：Ubuntu 22.04.5 / ROS 2 Humble / Gazebo 11.10.2；依赖全部已安装 |
| A 编译 | PASS | build.log；colcon build 返回0，C++ 车辆/GUI 插件及安装成功 |
| A 控制数学单测 | PASS | unit_test.log：1个 CTest、0错误/失败/跳过；内外轮、倒车、NaN/Inf、零速和斜坡断言 |
| A URDF/Xacro | PASS | model_check.log/json/urdf：19 links、18 joints、4滚动+4转动、正定惯性、光学坐标系 |
| A/B Gazebo生成与插件 | PASS | final/gazebo.log：Successfully spawned xtnetrc、C++ plugin loaded、双camera publishers |
| B 四轮接地/TF/odom | PASS | 四轮实际 z≈0.05521m；Gazebo与odom水平误差 2.93e-09m；真实关节状态8项 |
| B 静止动力学 | PASS | 3.037秒漂移 0.000001277m，z范围 2.19e-11m |
| C 前进后退 | PASS | 前进 0.934917m / 4.049s；后退 -0.932192m / 4.038s；停车后速度接近0 |
| D 阿克曼左转 | PASS | 左右轮 0.270986/0.213004rad；半径 0.987563m，误差 5.68% |
| E 阿克曼右转 | PASS | 左右轮 -0.215068/-0.271707rad；半径 0.972981m，误差 4.12% |
| F 原地旋转拒绝 | PASS | v=0,omega=1；位移 0.00008229m，yaw变化 4.18e-07rad；diagnostic=unrealizable_twist |
| G XWF/H65 双摄像头 | PASS | 640×480两路同时持续RGB；独立CameraInfo/K、零畸变、正确光学帧；final/front.png、gimbal.png |
| G 高分辨率 | PASS | gui_verified/smoke.log：两相机1280×720，同时CameraInfo有效；未宣称实车H65硬件模式 |
| H 双轴云台 | PASS | 左右Pan±0.6、Tilt上下、两轴极限、回中；gimbal图像变化、固定图像差0；转向角保持回中 |
| I 物理碰撞 | PASS | 向静态墙驾驶6s，实际位移 0.275030m（自由行驶预期约1.1m）；末速度 -1.53e-05m/s；z变化 2.48e-06m |
| J 安全停车/非法输入 | PASS | 300ms租约配置；停止发布后停车；NaN/Inf拒绝；无旧目标自恢复；新有效命令恢复；速度/转角限幅 |
| 倒车 Twist | PASS | v=-0.2,omega=+0.2；实际yaw变化 0.512421rad，转角为负，符号正确 |
| K 自动化清理 | PASS | test_xtnetrc.sh返回0，12/12；每等待有超时，300秒外层上限；仅清理本次创建的进程组 |
| GUI 可视化 | PASS | gui_verified：实际gzclient6张视口PNG；斜侧整车、四轮顶视、两相机与云台结构已逐图检查 |
| 现有400m红/蓝跑道 | PASS | track_red_final/track_blue_final：spawn、约0.632m运动、1280×720双图像；原几何未修改 |

## 转向和轨迹实测

每次以 speed=0.25m/s、中心转角 δ=±0.25rad 持续约8秒仿真时间。L=0.23861m、T=0.24m，理论曲率±1.070122/m、理论半径0.934472m。两后轮扭矩实时完全相同，转向由前轮关节/接地约束完成。轨迹取实际Gazebo后轴参考点，舍弃初始1.5秒斜坡后拟合圆，不使用理想运动学积分。

| 转向 | 理论左右轮 rad | 实际左右轮 rad | 实际半径 m | 半径相对误差 | 圆拟合RMS m |
|---|---|---|---|---|---|
| 左 | 0.284988, 0.222536 | 0.270986, 0.213004 | 0.987563 | 5.68% | 0.00021732 |
| 右 | -0.222536, -0.284988 | -0.215068, -0.271707 | 0.972981 | 4.12% | 0.00044718 |

验收界限写在回归脚本中：轮角误差<0.035rad、轨迹半径误差<20%、圆拟合RMS<0.03m；本次实际误差如表。PD限扭矩和刚性轮胎接触导致模型偏离理想几何；这不是精确实车传动/悬挂标定。

![实际Gazebo轨迹](../reports/xtnetrc/final/ackermann_trajectories.png)

## 相机与云台实测

640×480模式：XWF fx=381.362467px，H65 fx=457.007594px，均独立针孔K、零畸变。2秒仿真时间接收固定/云台图像22/18帧；这是本次订阅接收数量，并非保证墙钟帧率。高分辨率两路1280×720也通过。

| 云台动作 | 实际Pan / Tilt rad | 云台图像平均像素变化 | 固定相机变化 |
|---|---|---|---|
| pan_left | 0.600000 / -0.005928 | 13.731 | 0.000 |
| pan_right | -0.600000 / -0.005928 | 31.395 | 0.000 |
| tilt_up | 0.000005 / 0.448957 | 41.860 | 0.000 |
| tilt_down | -0.000009 / -0.409278 | 80.789 | 0.000 |
| upper_limit | 1.570797 / 0.800000 | 41.984 | 0.000 |
| lower_limit | -1.570796 / -0.600000 | 60.798 | 0.000 |

回中后 Pan/Tilt=0.00000614/-0.00592792rad；Tilt受重力有小稳态误差，验收<0.03rad。正Tilt抬头，软件角度限位属于 simulation_assumption，未将PWM转角度。

## 安全与碰撞

300ms墙钟租约到期后清除旧速度/转角目标，按1.2m/s²减速；本次2秒观测窗口末速度为约-4.48e-6m/s，继续等待不自恢复。NaN速度、Inf转角均被拒绝；新有效指令重新起步。中心转角被夹到0.436332rad，急弯9m/s/9rad输入在曲率限速后目标为0.234063m/s。实体静态墙有collision，车辆有真实轮胎/底盘collision，未通过关闭动力学/碰撞解决问题。

![看门狗后的实际制动](../reports/xtnetrc/final/watchdog_braking.png)

## GUI 与证据获取

Windows computer-use 工具因当前WSL cwd的URI映射错误无法调用；X11直接抓窗返回黑图，均不计为GUI通过。随后使用自编译可选GUI插件，调用正在运行的gzclient实际UserCamera保存视口，取得6张非空原生截图并逐图检查，故最终GUI标为PASS。该工具仅移动观察相机，未改车辆物理位置；图像不是生成式渲染。

![车辆与双相机](../reports/xtnetrc/gui_verified/gui_view_1.png)

![四轮与云台顶视](../reports/xtnetrc/gui_verified/gui_view_5.png)

## 故障修复与重测记录

- 包maintainer邮箱格式校验失败，修正元数据；ROS Humble不支持此处泛型lambda回调推导，改为显式消息/服务类型后编译通过。失败日志 build_attempt1/2 保留。
- run1发现Gazebo Camera将camera_name再次加到namespace，导致话题多一层；改为准确重映射后双路通过。
- run2发现Python ROS消息要求float，整数赋值造成测试失败；修正测试脚本后run3完整12项通过。
- GUI检查促使新增前后轮轴横梁/支柱；更新模型后final重新跑完整12项，通过。
- GUI smoke中一次快速重复启动命令源出现DDS匹配超过3秒的失败；有界发现等待改为10秒，gui_verified所有CLI重测成功。没有修改Gazebo物理断言来掩盖失败。
- 额外跑道 smoke曾因空capture_dir launch参数失败；仅在请求GUI时传该参数，并在launch提前退出时立即报告错误；红蓝重测均通过。

## 未执行或尚未验收

| 项目 | 状态 | 原因 |
|---|---|---|
| 实体XT-NetRC车辆标定/运行 | NOT RUN | 本次仅仿真，没有连接实车 |
| GPIO/PWM后端 | NOT RUN | 未实现/加载硬件适配，保持隔离 |
| Pure Pursuit/视觉完整循迹/400m整圈 | NOT RUN | 后续算法阶段，当前仅验证接口与场景生成/短程运动 |
| GPS/IMU融合 | NOT RUN | 本阶段未添加；odom是Gazebo真值 |
| 完全无DISPLAY的相机渲染部署 | NOT RUN | 无GUI测试使用WSLg DISPLAY=:0 |
| 控制器异常注入/进程崩溃测试 | NOT RUN | 代码提供异常清空/锁定分支，本次只测输入非法与租约超时 |
| 不同负载/高速/坡地/软胎悬挂 | NOT RUN | 当前低速刚性轮胎平地模型 |

真实待测：前后轮距、负载半径、质量/质心、实际驱动桥/差速器、机械最大转角、PWM—轮角/速度/云台角、镜头安装位姿/内参、轮胎摩擦。25°和0.6m/s为软件初值。源仓库轮胎卡尺原照片未公开于检出目录，本次仅引用其记录。

## 复现与交付

启动、编译、车辆操作、云台、双图像及测试完整命令见 [车辆README](../src/remote5g_sim/README.md)。实际执行结果均在下列目录：

- [最终构建](../reports/xtnetrc/build.log)、[单元测试](../reports/xtnetrc/unit_test.log)、[模型检查](../reports/xtnetrc/model_check.json)
- [12项物理结果JSON](../reports/xtnetrc/final/results.json)、[物理日志](../reports/xtnetrc/final/regression.log)、[Gazebo日志](../reports/xtnetrc/final/gazebo.log)
- 轨迹CSV：final/forward.csv、reverse.csv、left.csv、right.csv、timeout.csv、collision.csv、twist_reverse.csv
- [GUI/高分辨率结果](../reports/xtnetrc/gui_verified/smoke_result.json)、[CLI日志](../reports/xtnetrc/gui_verified/smoke.log)
- [红跑道结果](../reports/xtnetrc/track_red_final/smoke_result.json)、[蓝跑道结果](../reports/xtnetrc/track_blue_final/smoke_result.json)

Gazebo音频设备不可用告警仅禁用音频，图像/物理仍工作；KDL根惯性告警不影响本包固定TF/关节TF（已验证）。GUI清理时gzclient对SIGINT退出较慢，由本次launch向其自己的子进程升级SIGTERM；原先16801/16802跑道进程保持运行。
