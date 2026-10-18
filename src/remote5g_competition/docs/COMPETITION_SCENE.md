# 比赛场景、坐标与运行

独立新增 `remote5g_competition`；现有 `remote5g_sim`、400m红蓝网格/SDF/world/材质及配置只读复用。生成器解析原world创建新overlay，继续include原跑道，仅在新场景添加道具、状态服务和GUI/灯光设置。

环境：WSL Ubuntu22.04、ROS2 Humble、colcon/ament_cmake、Gazebo Classic11.10.2。未使用sudo或更换系统版本。用户缓存依赖qrcode8.2、zxing-cpp2.3.0；OpenCV4.5.4未链接QUIRC，所以使用独立解码器验证。PyMuPDF1.26.7仅用于此次PDF查阅，不是运行依赖。

## 路线

世界原点在操场中心，X沿直道、Y沿短轴、Z向上，半圆中心(±42.195,0)，单位米。已检查world/model/mesh的原点、单位和变换；生成时再次验证原include为单位变换。

`Route.pose(s,d)`的s从下直道进入对应半圆前3m起算，d始终为行进方向左侧。右下起点沿+X、绕右半圆，上直道沿-X到左上停车；左下起点镜像，先-X绕左半圆，再沿上直道+X到右上停车，符合图1的1→2→3。

默认第3道，参考坐标域211.561m，起点线到停车框约206m，属于规则约200m的布局近似；图1没有绝对坐标。区域2在上直道圆弧出口后4m，停车区距参考域终点3m。站位、引导距离和曲线都是仿真假设，具体坐标由 `layout.json`、`coordinates.csv` 输出。

车辆原点仍是已有后轴基准。默认按车架最前沿靠近发车黄线内缘20mm计算位置，yaw来自路线切线，Z=0.006为原spawn裕量。`vehicle_spawn`配置或launch `vehicle_x/vehicle_y/vehicle_z/vehicle_yaw`可覆盖，未编辑车辆模型。

## 配色与引导

红底用蓝色弯道障碍和两排原车道引导锥桶；蓝底相应使用红色。外侧两排均为黄色。按用户完整四排图片及PDF图11的两种方向组合，原车道两条白线各有一排短直线锥桶，两侧相邻车道各有一排黄色外边界。黄色边界在尾段弯回原车道的对应白线并继续直行。变道标志选择走哪侧；切换标志不会删除另一侧边界。

横道后紧邻变道标志，二维码支架保留在其后的区域2下游端；引导入口距区域2端0.20m、距标志中心约1.40m，移除了旧版10m空段。原车道两排长3m；黄色排晚1m开始，先直行1m，用4m五次平滑曲线从相邻车道返回，再沿原车道直行1m。以上纵向距离是可调仿真假设，PDF没有规定绝对站位。每排按曲线弧长0.50m采样；四排共40个引导锥桶，加1个弯道障碍及其他道具共53个实例（以输出清单为准）。旧版两排及错误的向外偏移路径已替换。

车辆宽度、偏航车长、原车鼻端及锥桶底径参与几何校验。原车道直线边界在黄色回归段收窄前结束，提供旁路转回空间。参考曲率小于已有阿克曼车最大转向曲率；这只是静态几何验证，尚未执行自主变道或完整通关。

`obstacle_seed`独立控制弯道随机位置；固定模式用 `obstacle_mode: fixed`、`obstacle_s`、`obstacle_lateral_offset`。左右标志和停车A/B可设random，使用独立sign_seed；输出选定值和坐标，同配置可复现。

## 编译和启动

```bash
source /opt/ros/humble/setup.bash
cd ~/remote5g_ws  # 换成实际项目目录
colcon build --packages-select remote5g_sim remote5g_competition --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch remote5g_competition competition.launch.py track_color:=red start_side:=left lane_change:=left gui:=true
```

默认Gazebo11385，区别于旧跑道11365和车辆包11375。入口加载新overlay、道具，并保留robot_state_publisher和gazebo_ros spawn_entity.py机制。操作终端应加载同一环境及ROS_DOMAIN_ID。

```bash
ros2 launch remote5g_competition competition.launch.py track_color:=blue start_side:=right lane_change:=right gui:=false
ros2 launch remote5g_competition competition.launch.py track_color:=blue start_side:=right spawn_vehicle:=false cone_static:=true barrier_present:=false gui:=false
```

场景依次启动；并存要用不同port和ROS域。无gzclient测试仍借助WSLg渲染，完全无DISPLAY部署未验收。默认输出 `~/.ros/remote5g/reports/competition/current`，可用output参数更换。原world不改。

## 生成和单独加载

```bash
ros2 run remote5g_competition generate_layout.py --output $HOME/.ros/remote5g/reports/competition/custom_seed --track-color blue --start-side right --lane-change right --obstacle-seed 42 --qr-content REMOTE5G_SIM_CUSTOM
ros2 run remote5g_competition validate_layout.py $HOME/.ros/remote5g/reports/competition/custom_seed/layout.json
ros2 run gazebo_ros spawn_entity.py -entity comp_demo_cone -file "$(ros2 pkg prefix --share remote5g_competition)/models/comp_cone_red/model.sdf" -x 0 -y 0 -z 0.001 -timeout 15
```

以上入口已执行。输出world、模型资源、每实例SDF、解析YAML、JSON/CSV。包models目录有19种可重用资产，worlds/{left,right}_{red,blue}含四种场景；只include原跑道。运行时重新生成资源可解决移动工作区后的绝对URI变化。资源/插件路径由launch设置，重新source后也可加载。

## 管理与停车检查

layout路径应指向当前输出；本机对相同入口的测试输出目录实际执行了以下操作。

```bash
ros2 run remote5g_competition manage_props.py remove --layout $HOME/.ros/remote5g/reports/competition/current/layout.json --name comp_start_barrier
ros2 run remote5g_competition manage_props.py restore --layout $HOME/.ros/remote5g/reports/competition/current/layout.json --name comp_start_barrier
ros2 run remote5g_competition manage_props.py reset --layout $HOME/.ros/remote5g/reports/competition/current/layout.json --name comp_guide_inner_000
ros2 run remote5g_competition parking_check.py --layout $HOME/.ros/remote5g/reports/competition/current/layout.json
```

remove/restore提供barrier_removed/barrier_present状态，不改变车辆位姿。name=all仅管理清单中的comp道具，不操作轨道或车辆；restore/reset返回保存的初始实例位置，包含可恢复的挡板模板。材质有Gazebo缓存，修改内容后重启场景。

停车检查读取Gazebo四个真实wheel link世界姿态，验证宽0.04952m、长0.02m的保守接触矩形角点。接触长度是刚性轮胎仿真假设。A在局部d>0、B在d<0，排除分隔线；不以base_link中心代替四轮检查。JSON输出每轮角点与两槽结果。

## 二维码与素材

二维码暂无明确规定的车载识别任务；扫码测试仅验证纹理质量，不作为区域2比赛任务。主码REMOTE5G_SIM_QR_01、停车REMOTE5G_SIM_PARK_A/B，均非支付测试码，没有微信支付功能。660×660 PNG含4模块静区，完整显示面10cm。主码UV通过实际相机模块比对验证，朝向0°且无镜像。资源文件名唯一，防止Ogre缓存造成箭头/A/B/QR混用。

近距离正前方会被现有车体/相机挡住部分地面码；原车不改。报告保留失败图像，停车码侧向云台测试位姿属于夹具，不是自动泊车。

materials/source_signs保存PDF内嵌原始位图和哈希，不宣称取得可编辑、高分辨率官方打印源文件。渲染纹理增加约2mm打印白边，属仿真假设。替换源图后重新生成。PDF原图权利不因本包Apache-2.0代码许可改变。

## 自动测试

```bash
colcon test --packages-select remote5g_competition --event-handlers console_direct+
colcon test-result --verbose
ros2 run remote5g_competition run_matrix.py --gui --output $HOME/.ros/remote5g/reports/competition/regression
```

几何CTest覆盖红蓝×左右起点×左右变道8组合、seed复现、尺寸/资源、真实QR解码与停车反例。矩阵覆盖四个颜色/起点及无车固定道具模式；左红、右蓝执行物理接触和实际相机扫码，其余验证加载/稳定性。各等待有超时，每场270秒总限制，建议整次外加750秒timeout；只清理自己启动的进程组，不用killall/pkill，已占用端口拒绝启动。

COMPETITION_TEST_PORT/COMPETITION_TEST_DOMAIN可改隔离实例。run_matrix.py --single left_red或--single right_blue均执行全物理检查。SetEntityState只准备测试夹具位置，不作为自主驾驶、变道或泊车证据。

## 未验收范围

自动起跑、避障、视觉等待、Pure Pursuit和完整约200m通关均AUTONOMOUS_DRIVING_NOT_TESTED。PDF p5写停3s，p2/p7写10s，存在冲突；本阶段不实现等待状态机，不宣称规则统一。质量、摩擦、支架细节、路线绝对站位、通道曲线和接触斑均是可调仿真假设。

路径说明：默认从 `remote5g_sim` 安装目录读取跑道；比赛输出为当前用户的 `~/.ros/remote5g/reports/competition/current/`。换目录或用户后重新编译、加载 `install/setup.bash` 即可。需要自定义跑道时传 `workspace:=/实际工作区路径`，自定义输出时传 `output:=/可写目录`。
