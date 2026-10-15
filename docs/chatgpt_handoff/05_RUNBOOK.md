# 05 启动、观察、控制与停止操作手册

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

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
