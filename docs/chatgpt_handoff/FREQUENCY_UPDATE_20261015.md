# 2026-10-15：前置30fps、速度/转角目标50Hz修订

用户明确授权修改当前模型配置与发布脚本，并更新ChatGPT教学手册。30fps依据用户提供产品介绍中的摄像头I模式；50Hz是用户选择的控制设计值，产品介绍没有规定控制频率。

## 现在的配置

| 环节 | 当前配置 | 时间基/说明 |
|---|---|---|
| 前置RGB | 30fps | 仿真时间，仍使用原HFOV80°和安装姿态；基础640×480、比赛1280×720 |
| 云台RGB | 15fps | 本次未改；不能从产品介绍推断摄像头II帧率 |
| 速度/中心转角/云台目标及状态反馈 | 50Hz目标，control_period=0.02s | 仿真时间，现有按物理步阈值调度，允许量化与浮点误差 |
| drive.py | Timer周期0.02s，50Hz | 当前节点墙钟，持续时间与命令租约也是墙钟；不是把spin_once超时当周期 |
| robot_state_publisher | publish_frequency=50.0 | 两个launch均设置，是关节TF上限，依赖实际joint_states |
| 物理力矩/接触 | 原物理步更新 | 不把物理内环降为50Hz |

这是现有目标接口/底层控制与数据发布频率，**没有新增路径跟踪、巡线或比赛状态机**。现有系统键盘工具仍按键事件发布；本次只将drive.py定时化，没有修改系统teleop包。

## 实际验证

隔离ROS域97、Gazebo端口11407，未停止或发送指令到用户已运行的域75平坦场。两个源码包构建成功；colcon现有两项单元/几何测试0错误、0失败。

| 场景 | 实测流 | 仿真时间Hz | 墙钟接收Hz |
|---|---|---:|---:|
| flat | `/camera/front/image_raw` | 29.99 | 29.83 |
| flat | `/odom` | 48.51 | 48.26 |
| flat | `/joint_states` | 48.51 | 48.27 |
| flat | `tf:front_left_knuckle` | 48.51 | 48.27 |
| competition | `/camera/front/image_raw` | 30.00 | 24.26 |
| competition | `/odom` | 48.82 | 39.39 |
| competition | `/joint_states` | 48.82 | 39.39 |
| competition | `tf:front_left_knuckle` | 48.82 | 39.36 |

平坦场drive.py正常运行的指令频率约50.24Hz，Ctrl+C测试约49.90Hz；两次最后均收到零速度，实际车速已降至接近0，进程退出码均为0。SIGINT时保持ROS context用于finally发零，取消定时器后再停车，保留0.3s墙钟租约保护。

状态流实测约48.5–48.9仿真Hz，属于0.02s目标在原物理步阈值调度下的近似值，不宣称硬实时严格50.000Hz。比赛场1280×720前图按仿真stamp约30Hz，墙钟约24Hz；RTF/渲染/两套仿真并存负载使墙钟速率降低，不应把配置30fps当实机或任意负载下稳定实时30fps的保证。

首轮观察器使用depth=1同时监听图像和状态，状态样本被覆盖导致计数约26.7Hz。将**检查工具**的状态接收队列改为200后复测，控制生产代码未为此更改；原失败日志仍保留在snapshot/frequency_update/first_attempt.log。图像保持depth=1，避免积压旧帧。

原始结果：reports/frequency_30fps_50hz/results.json；便携副本：[结果JSON](snapshot/frequency_update/results.json)。完整构建与单测记录见snapshot/frequency_update，参数来源见snapshot/product_reference_20261015.md。

## 使用与后续教学

运行命令不变。旧Gazebo实例在启动时加载了旧URDF/参数，源文件修改不会热更新：先退出控制，在原launch终端Ctrl+C，再重新source install/setup.bash并启动。不要为了应用配置killall/pkill，也不要向用户旧实例发检查指令。

新启动后查看 `/odom`、`/joint_states`及drive运行中的 `/ackermann_cmd` 频率；TF总话题包含多个发布者，不以其总Hz判断单条变换。需要进一步严格定时或全仿真时间同步时，另立任务，不能把本次目标频率理解为硬实时保证。

ChatGPT先按原A1启动/观察课程教学，再介绍50Hz Timer与30fps采集；图像不必和控制同频。初始调查的raw runtime、bag、旧运动量和截图继续作为历史，不重写为新版本证据。当前快照文件、展开URDF、源码副本、合并手册与轻量ZIP均按本修订刷新。
