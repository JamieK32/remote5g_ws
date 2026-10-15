# Gazebo Classic 标准田径跑道模型

工作区当前实际位置为 `/home/jamiek/remote5g_ws`。

## 使用

从工作区运行：

```bash
cd /home/jamiek/remote5g_ws
./scripts/launch_track.sh red
./scripts/launch_track.sh blue
```

默认启动完整、实际尺寸的场景。无需 ROS 节点、插件、在线模型库或 Blender。
独立模型为 `simulation/models/wa_400m_track_red/` 和 `simulation/models/wa_400m_track_blue/`，每个包含 `model.config`、`model.sdf` 和自带的 Collada 几何文件。两个版本的几何、碰撞、分道线完全相同，只有跑道材质颜色不同。

也可将模型加入自己的 Gazebo Classic world：

```bash
export GAZEBO_MODEL_PATH="/home/jamiek/remote5g_ws/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

```xml
<include>
  <uri>model://wa_400m_track_red</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

将 `red` 改成 `blue` 即使用蓝色模型。模型是 SDF 1.6 静态环境，可独立放入其他工作区使用；附带有限尺寸的承重地面，不需要另行加载 ground_plane。

## 来源与尺寸

依据用户提供的 `docs/official/World-Athletics-Track-and-Field-Facilities-Manual-2019-Edition.pdf`，第 2.2.1.1 节、图 2.2.1.1a/b、表 2.2.1.1（印刷页 29–31，对应 PDF 页 27–29）。

| 参数 | 模型值 |
| --- | --- |
| 半圆弯道内缘半径 | 36.50 m |
| 两段平行直道长度 | 各 84.39 m |
| 跑道数量 | 8 |
| 每道宽度（包含外侧白线） | 1.22 m |
| 白色分道线宽 | 0.05 m |
| 总跑道宽 | 9.76 m |
| 最外缘半径 | 46.26 m |
| 椭圆外包络 | 176.91 × 92.52 m |
| 第一道计量线半径 | 36.80 m |
| 第一道计量线周长 | 400.0012193 m |
| 内缘路缘石 | 宽 0.05 m、高 0.05 m，位于内缘以内 |
| 直跑道含准备和缓冲区长度 | 130 m（3 + 110 + 17 m） |

手册允许多种道数，这里选用图示的八道布局。每道的外侧白线包含在 1.22 m 内，未额外增加跑道宽度。红、蓝颜色是供本赛项视觉仿真选择的材质颜色，并非手册规定的色号。

弯道每半圆使用 1024 段网格，最外缘最大弦高误差约 0.055 mm。修正版 1.1 将草地、彩色跑道、白色标线与外围地面划分为互不重叠的平面区域，全部位于 Z=0；取消可见的整块底面和标线叠层，也不再使用 depth_bias。白线仍为真实 0.05 m 宽的几何区域。地表按 16 m 分块三角化，避免内场使用贯穿全场的细长放射状三角面。碰撞地面仍为连续刚性盒体，接触高度 Z=0；路缘石保留独立碰撞。每版模型自带 geometry_audit.json，记录地表区域交叠面积与覆盖误差。

## 机器人坐标与路径

- 原点位于跑道中心，X 沿直道长轴，Y 沿短轴，Z 向上，单位米；此坐标轴不绑定地理方向。
- 两个半圆圆心为 `(±42.195, 0)`。
- 第一条道中心位置为 `(0, -37.11, 0)`，沿 +X 行驶即为逆时针绕场。
- 机器人初始 Z 应加上自身车轮接触几何所需高度，不能将整车中心直接置于地面。
- `simulation/maps/lane_1_centerline.csv` 至 `lane_8_centerline.csv` 给出各道中心线，采样间距不超过 0.25 m，字段为 `s_m,x_m,y_m,z_m,yaw_rad`，首尾位置相同。
- 第一道中心线比官方 400 m 计量线更靠外，长度约 401.949 m；规划时不要把“道中心线”和“计量线”混为一谈。
- 参数和来源记录见 `config/athletics_track.json`。摩擦系数 0.9 为仿真初值，未由手册或实测标定。

## 当前范围

这是机器人仿真使用的标准跑道基础环境，包含绿地内场、八道椭圆、直跑道延伸、分道线、100/110 m 起点线和终点线。没有声称完整复现田径场的全部赛事标记、跳投设施，也没有添加本赛项的锥桶、斑马线、停车区、交通标识或 5G 通信逻辑。赛项布局可作为独立模型叠加，避免改变标准跑道几何。

## 重建

```bash
python3 /home/jamiek/remote5g_ws/scripts/generate_athletics_tracks.py
```

生成器使用 Python 标准库，并通过 ctypes 调用 WSL 系统 GEOS C 库（要求 GEOS >= 3.10；本机已安装 3.10.2）。没有 GEOS 时可安装 `libgeos-c1v5`。此依赖仅用于重建网格；加载已生成的 Gazebo 模型不需要 Python 或 GEOS。此命令会重新生成本套模型、场景、中心线路径和参数文件。

启动脚本默认使用独立 Gazebo master 端口 11365，与现有 autocar 仿真分开。需要同时启动两个本赛项场景时，可使用 `REMOTE5G_GAZEBO_MASTER_URI=http://127.0.0.1:11366 ./scripts/launch_track.sh blue`。

模型更新后请关闭并重新打开对应场景，Gazebo 已加载的网格不会自动刷新。
