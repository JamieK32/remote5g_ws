# 参考源码审查（2026-10-14）

只读参考：https://github.com/kxkxkxa767/stupid_car ，commit `bec5112ea11dca1516251327ce3ad35e3f5a6647`。
源码检出位于 `/home/jamiek/.cache/xtnetrc-reference/stupid_car`，不进入 colcon 源码树。
本次独立实现仿真代码，没有复制其 C++、Python、模型、标定矩阵或图片。
仓库未找到 LICENSE/COPYING 或明确覆盖该工程的授权声明，因而不将“公开可查看”视为可复用授权。
本包新代码采用 Apache-2.0；这不改变参考仓库文件的权利。
已检查文件及 SHA-256 见 `reference_manifest.json`；该清单是源文件证据索引，不声称运行过上游硬件代码。

| 已读取内容 | 发现及仿真处理 |
|---|---|
| README、AGENTS、motion_control/controller.cpp、motion_default.json | 后轴原点、正转角左转、实测轴距 0.23861；软件 25°、120°/s、0.6m/s、加减速 0.8/1.2、曲率降速公式；独立实现阿克曼几何和限幅 |
| car_runtime/control.cpp、app.cpp、runtime_2023.json | 20Hz、单独转向/电调输出；实车 5秒/3m 是调试上限，不套入持续仿真；执行器标定未确认 |
| hardware/vehicle.cpp、vehicle.hpp、pigpio_driver.cpp | 单电调输入，300ms 独立租约、异常回中性；新插件仅虚拟扭矩，不访问 GPIO 或 pigpio |
| camera_gimbal/gimbal.cpp、gimbal_default.json | PWM 中心 72/73、软件范围 28–122/58–122、步进 20ms；没有可靠角度曲线，不转换 PWM 为角度 |
| camera_calibration/README、摄像头身份与用途、ground_projection.json | XWF 固定循迹、H65 云台；XWF 640×480 原始像素四点投影是特定装配外参，历史内参设备关联不足；不复制到新相机 |
| camera_io/frame_source.cpp、v4l2_camera.cpp | 角色、by-id、分辨率与新鲜度检查；仿真使用命名 topic/optical frame 保持双路隔离 |
| ground_projection/ground_projector.cpp | 单应投影、像素去畸变、范围和有限数检查；保留后轴坐标，后续使用仿真 K/TF 生成地面映射 |
| reference/05_XT-NetRC车辆规格/XT-NetRC车辆规格与控制建模.md | 记录无负载外径 110.63mm、宽 49.52mm；原照片未收录，测量溯源属于上游记录；轮距/质量/质心/机械极限待测 |

驱动资料“四轮差速驱动”不能证明四独立电机，亦不能确定这辆实体车的驱动桥。
本阶段明确采用前轮转向、后桥驱动、单虚拟电机和开放差速器的仿真假设：两半轴相同扭矩，转弯轮速由接地约束自然产生。

已检查 ROS Gazebo 官方插件源码 `gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp`（ros2 分支，只用于接口评估）。
其 `OnCmdVel` 将 angular.z 存为转角而非角速度，倒车时又乘速度符号；通过 PID SetForce 驱动转向和后轴，但没有本任务需要的 NaN 检查、命令看门狗和 AckermannDriveStamped 入口。
因此不直接采用该插件，也不复制其代码。

仿真配置 `vehicle.yaml` 与 `parameter_sources.yaml` 一一对应。实际 GPIO/PWM 仅保存在 `hardware_reference_NOT_LOADED.yaml`，所有 launch/plugin 均不加载。
