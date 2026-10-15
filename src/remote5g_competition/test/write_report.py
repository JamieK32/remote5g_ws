#!/usr/bin/env python3
"""Summarize recorded results; never substitute for actually running tests."""
import json,hashlib
from pathlib import Path
root=Path('/home/jamiek/remote5g_ws');package=root/'src/remote5g_competition';reports=root/'reports/competition';revision=reports/'revised'
matrix=json.loads((revision/'matrix_results.json').read_text())
assert set(matrix)=={'left_red','left_blue','right_red','right_blue','props_only'}
assert all(r['status']=='PASS' for r in matrix.values())
cli=json.loads((reports/'commands_final/result.json').read_text());assert cli['status']=='PASS'
saved=json.loads((reports/'original_files_sha256.json').read_text())
changed=[p for p,h in saved.items() if hashlib.sha256((root/p).read_bytes()).hexdigest()!=h];assert not changed
geometry=json.loads((revision/'left_red/geometry.json').read_text());layout=json.loads((revision/'left_red/layout.json').read_text())
rows='\n'.join(f'| {name} | {r["status"]} | {r["elapsed_wall_seconds"]:.2f}s | {len(r["results"])} 项真实 Gazebo 检查 |' for name,r in matrix.items())
checks='\n'.join(f'| {name} | {r["status"]} |' for name,r in matrix['left_red']['results'].items())
load=matrix['left_red']['results']['A_loading_performance']['evidence'];obstacle=load['bend_obstacle'];pos=obstacle['actual_xyz']
barrier=matrix['left_red']['results']['D_barrier_tape_reset']['evidence'];cone=matrix['left_red']['results']['D_cone_contact']['evidence'];stand=matrix['left_red']['results']['D_qr_stand_collision']['evidence']
qr=matrix['left_red']['results']['E_simulated_camera_qr']['evidence']
fail_views=[f'{a["camera"]}@{a["rear_axle_distance_m"]:.2f}m' for a in qr['all_attempts'] if not a['decoded']]
text=f'''# 比赛场景实际测试报告（四排引导修正版）

验收日期：2026-10-14，WSL Ubuntu22.04 / ROS2 Humble / Gazebo Classic11.10.2。以下结论读取实际日志、Gazebo服务及相机图像；测试夹具瞬移不计作自主驾驶。

## 本次布局纠正

实际重新阅读规则PDF第3–9页及用户两张完整四排图。原车道两排短直线蓝/红锥桶，加两侧相邻车道两排黄色弯回锥桶，共四排40个。两排黄色均收回原车道，原道两排在回归收窄前结束。入口紧随区域2端0.20m，距变道标志中心{geometry['guide_sign_gap_m']:.4f}m；删除旧版10m空段。此四排整体依用户图片组合图11的两个方向，精确纵向长度仍为仿真假设。

弯道障碍独立命名comp_obstacle，每场恰好一个；随机种子{obstacle['seed']}，路线s={obstacle['route_s']:.6f}m、d={obstacle['local_d']:.6f}m。左红场景Gazebo实测世界位置({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})m，XY误差{obstacle['position_error_m']:.3g}m。不能依整体俯视图中8cm锥桶的像素大小判断它是否加载，提供原生近景及实体状态证据。

横道 → 随机左右标志 → 二维码支架 → 四排引导 → 随机A/B停车标志 → 四轮停车检查。二维码没有用户规定的车载识别任务；下面扫码仅是纹理可读性检查。左右变道和停车选择默认random、sign_seed可复现，矩阵用显式方向覆盖两种选择。

## 编译、几何与文件保护

| 检查 | 结果 | 证据 |
|---|---|---|
| colcon build，新增包 | PASS | revised_build.log |
| colcon test，几何8组合及停车反例 | PASS | revised_unit_test.log；0 errors/failures |
| 19种独立模型gz sdf -k | PASS | revised_sdf_checks.json |
| 固定seed复现；不同seed仅移动一个弯道障碍 | PASS | test/test_geometry.py |
| 四排、颜色、紧邻标志、黄色归位 | PASS | revised/*/geometry.json；实际加载名单 |
| 原跑道/车辆及已有文件共{len(saved)}个SHA256 | PASS | revised_original_integrity.json，0个变化 |
| 独立加载、移除/恢复挡板、重置锥桶、停车CLI、手动spawn覆盖 | PASS | commands_final/result.json / commands.log |

实际白线间净宽W={geometry['clear_lane_width']:.3f}m、发车/停车总长{geometry['zone_length']:.3f}m；横道5片，纸片0.297×0.105m、净间隙0.105m。锥桶0.080m高、0.078m最大底径。引导弧长步长0.500m，弯曲后的两点直线距离范围{geometry['guide_spacing_min_max'][0]:.6f}–{geometry['guide_spacing_min_max'][1]:.6f}m。

车体保守需求宽{geometry['required_corridor_width_conservative']:.4f}m，回归路径在蓝/红锥桶尾端含车鼻裕量的最小估计宽{geometry['corridor_width_min_conservative']:.4f}m；参考回归曲率最大{geometry['max_curve_curvature']:.4f}/m，原车约束{geometry['vehicle_max_curvature']:.4f}/m。仅校验回归段理论几何；入口变道及完整通道的实际驾驶未验收。

## 实际Gazebo矩阵

| 场景 | 结果 | 墙钟耗时 | 范围 |
|---|---|---|---|
{rows}

左红/右蓝完整检查，其余加载与3秒仿真稳定性检查。props_only无车、固定锥桶、挡板预移除。每次使用独立ROS域/Gazebo端口及本次拥有的进程组，清理不使用killall/pkill。每模型经GetEntityState核对存在和接地，包括全部四排及弯道障碍。数值与完整模型名单在physics_results.json。

| 左红检查 | 结果 |
|---|---|
{checks}

挡板存在时车位移{barrier['blocked_displacement_m']:.4f}m，移除后同控制位移{barrier['removed_displacement_m']:.4f}m；移除瞬间位姿变化{barrier['remove_pose_change_m']:.6f}m，恢复成功。轮胎实际接触测试锥桶并推动{cone['cone_actual_motion_m']:.4f}m，测试后恢复弯道的原唯一障碍。支架实际阻挡车运动：位移{stand['blocked_displacement_m']:.4f}m，相同自由控制预计约1.1m。贴地纸片/胶带无实体碰撞，实车通过。

停车判定取四个真实wheel link姿态及轮胎接触矩形角点。A、B正确接受；骑分隔线与base_link中心已入但轮胎越界的反例均拒绝。四轮接触斑长度0.02m是假设，非真实胎压模型。

## 图像、扫码与已知失败

主码在原车front/gimbal实际1280×720图像可解析，成功图方向0°、模块误差0（无镜像或倒置）。A/B非支付测试码均由实际云台侧视图解析，内容互异。近距离失败视图：{', '.join(fail_views)}。部分近景码被原车车体/相机遮挡或裁切，保留全部成功与失败图像；没有改车来消除这些限制。

截图是gzclient原生UserCamera SaveFrame，未用示意渲染代替。保存overview、start、parking、qr_stand、crosswalk、guide_cones、bend_obstacle；另有最终现场斜视bend_obstacle_close。旧版尝试日志也保留，旧布局及历史失败不作为当前验收证据。

![四排引导原生截图](../../../reports/competition/revised/left_red/screenshots/guide_cones.png)

![弯道单个障碍原生俯视截图](../../../reports/competition/revised/left_red/screenshots/bend_obstacle.png)

其他截图与相机帧：reports/competition/revised/left_red；矩阵汇总：reports/competition/revised/matrix_results.json；每个场景有coordinates.csv、layout.json及resolved.yaml。

## NOT RUN 与交付边界

自主发车、白线巡线、不压边界、弯道自动避障、语音播报、人行横道自动等待、视觉随机方向识别、实际变道及自主A/B停车/完整约200m通关：NOT RUN。本次是道具与场景，不把接触/扫码测试冒充算法完成。PDF p5等待3s和p2/p7等待10s有冲突，本包未实现等待状态机。

规则未规定绝对站位、所有架体尺寸、质量/摩擦、回归长度和有限接触斑；见MODEL_DIMENSIONS.md和parameter_sources.yaml。完全无DISPLAY环境未测试；当前无gzclient服务端测试仍依赖WSLg渲染。

没有Git仓库，未初始化、commit或push。仅新增竞赛包、报告/文档及打包文件；已有车辆包、轨道模型及world未修改。移动工作区后应修改workspace参数并由launch重新生成资源绝对URI。
'''
(package/'docs/SCENE_TEST_REPORT.md').write_text(text)
for name in ('COMPETITION_SCENE','MODEL_DIMENSIONS','SCENE_TEST_REPORT'):
 p=root/f'docs/{name}.md'
 p.write_text(f'# {name}\n\n完整内容见 [竞赛包文档](../src/remote5g_competition/docs/{name}.md)。\n')
print('Recorded-result report written; originals unchanged:',len(saved))
