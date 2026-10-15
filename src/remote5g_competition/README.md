# Remote5G 比赛道具与场景

独立Gazebo Classic11/ROS2 Humble包，保留已有400m跑道和阿克曼车辆。

- [场景、坐标、运行和管理](docs/COMPETITION_SCENE.md)
- [尺寸、PDF依据与仿真假设](docs/MODEL_DIMENSIONS.md)
- [实际测试报告](docs/SCENE_TEST_REPORT.md)
- config：布局参数及逐项来源；models：19种独立SDF、简单网格、真实二维码及PDF标志素材
- worlds：红蓝、左右起点场景；scripts：生成、验证、移除/恢复与四轮停车检查
- 工作区reports/competition：实际坐标、日志、原生GUI截图和相机扫码证据

```bash
source /opt/ros/humble/setup.bash
cd /home/jamiek/remote5g_ws
colcon build --packages-select remote5g_competition --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch remote5g_competition competition.launch.py track_color:=red start_side:=left lane_change:=left gui:=true
```

本机已具备qrcode/zxing-cpp用户缓存，无需sudo。新机器可用scripts/bootstrap_deps.sh准备默认用户缓存，不替换ROS的OpenCV/NumPy。没有Git仓库，未初始化、提交或推送。完整自主驾驶尚未实施。
