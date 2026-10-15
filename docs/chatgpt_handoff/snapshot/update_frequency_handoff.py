#!/usr/bin/env python3
"""Refresh teaching context after the explicitly authorized 30fps / 50Hz update."""
from pathlib import Path
import json,shutil,hashlib,sys
D=Path(__file__).resolve().parent.parent;S=D/'snapshot';W=D.parents[1]
allowed=['src/remote5g_sim/config/vehicle.yaml','src/remote5g_sim/config/parameter_sources.yaml','src/remote5g_sim/scripts/drive.py','src/remote5g_sim/launch/sim.launch.py','src/remote5g_competition/launch/competition.launch.py','src/remote5g_sim/README.md']
results=json.loads((W/'reports/frequency_30fps_50hz/results.json').read_text());assert set(results)=={'flat','competition'} and all(v['status']=='PASS' for v in results.values())
revision={'date':'2026-10-15','authorization':'User explicitly requested front RGB 30fps, speed/steering targets 50Hz and updated ChatGPT handoff','authorized_changed_files':allowed,'front_camera_target_fps':30.0,'gimbal_camera_target_fps':15.0,'control_period_sim_seconds':.02,'drive_command_wall_hz':50.0,'robot_state_publisher_max_hz':50.0,'path_tracking_algorithm':'NOT_IMPLEMENTED','manufacturer_control_frequency':'NOT_SPECIFIED','current_running_user_instance':'Existing user launch was left running with its previously loaded configuration; restart required','results':results}
(S/'frequency_update_20261015.json').write_text(json.dumps(revision,indent=2,ensure_ascii=False))
out=S/'frequency_update';out.mkdir(exist_ok=True)
for src,dest in [(W/'reports/frequency_30fps_50hz_build.log',out/'build.log'),(W/'reports/frequency_30fps_50hz_unit.log',out/'unit.log'),(W/'reports/frequency_30fps_50hz/results.json',out/'results.json'),(W/'reports/frequency_30fps_50hz_first_attempt.log',out/'first_attempt.log')]:
 if src.exists():shutil.copyfile(src,dest)
for relative in allowed:
 dest=S/'source'/relative
 if dest.exists():shutil.copyfile(W/relative,dest)
# Re-expand the same vehicle; geometry and resolutions are unchanged.
import xacro,xml.etree.ElementTree as ET
for name,mapping in [('vehicle_default',{}),('vehicle_competition',{'front_width':'1280','front_height':'720','gimbal_width':'1280','gimbal_height':'720'})]:
 doc=xacro.process_file(str(W/'src/remote5g_sim/urdf/car.xacro'),mappings=mapping);(S/f'{name}.urdf').write_text(doc.toprettyxml())
 inv=json.loads((S/f'{name}_inventory.json').read_text());tree=ET.fromstring(doc.toxml());inv['sensors']=[{'reference':g.attrib['reference'],'sensor':s.attrib,'camera':ET.tostring(s,encoding='unicode')} for g in tree.findall('gazebo') for s in g.findall('sensor')];(S/f'{name}_inventory.json').write_text(json.dumps(inv,indent=2))
(S/'product_reference_20261015.md').write_text('''# 用户提供的XT-NetRC产品参数依据

来源：2026-10-15用户在本聊天粘贴的“5G远程驾驶无人车XT-NetRC产品介绍”，用户称官方参数；本次未独立验证产品版本或实机模式。

摄像头I：1280×720、30帧MJPG/YUV、视角120°（水平/垂直/对角未说明）。摄像头II：1280×720，未给帧率；不得假设它也支持30或60fps。当前仿真把前置相机目标设30fps；640×480基础模式与1280×720比赛模式原样保留，云台仍15fps，两个HFOV暂不改。

资料未规定控制更新频率。50Hz是用户本次选定的仿真速度/转角目标和未来路径跟踪预算，不是从厂商手册直接读出的数值，也不代表已经实现路径跟踪算法。
''')
p=S/'write_documents.py';s=p.read_text()
replacements={
 '目标每0.05仿真秒更新':'目标每0.02仿真秒更新',
 '`drive.py`以约20Hz发布':'`drive.py`通过Timer按50Hz墙钟发布',
 '20Hz仿真目标':'50Hz仿真目标',
 '目标约20Hz仿真时间':'目标约50Hz仿真时间',
 'publish_frequency=20':'publish_frequency=50',
 '0.05秒仿真目标更新':'0.02秒仿真目标更新',
 '输出、20Hz发送、停止条件':'输出、50Hz定时发送、停止条件',
 '模型配置更新率15Hz（仿真时间）':'模型配置更新率为前置30Hz、云台15Hz（仿真时间）',
 '实际运动证据：平坦场':'初始调查的历史运动证据：平坦场',
 '本次后续hz窗口约19.8Hz；这是观测值，不承诺所有负载恒定20Hz。':'初始调查旧配置的hz窗口约19.8Hz；当前目标50Hz，新测量见频率修订报告。',
 '本次平坦场位移0.0941m。':'初始调查平坦场位移0.0941m；当前版本另有频率与停车复测。',
 '本次通过PTY发送12次i':'初始调查通过PTY发送12次i',
 '实测publisher可靠性为RELIABLE（见image_qos.txt）':'初始调查实测publisher可靠性为RELIABLE（见image_qos.txt）',
 '初始多话题/GUI采样窗口收到':'旧配置初始多话题/GUI采样窗口收到',
 '后续先量实际帧率。':'当前30fps前置采集的实测另见FREQUENCY_UPDATE_20261015.md。',
 '当前调查创建的所有仿真、GUI与录放进程已经停止，原工程未修改。':'初始调查创建的仿真、GUI与录放进程已停止；本次经用户授权修改频率配置与发布脚本，未改几何模型或比赛算法。',
 '本次只调查与建议，没有初始化Git、重构包、重装依赖或修改控制器。':'初始任务只调查；本次用户另行授权更新30fps/50Hz配置、指令定时发布与TF上限，未初始化Git、重构包或重装依赖。',
 '本次没有改动build/install/log，也没有为“验证构建”擅自重编译。读取历史构建与测试日志和成功运行已足以证明当前已有产物可启动，但无法证明在干净新机器上可重复构建。':'初始调查没有重新编译；当前频率修订已经colcon build两包，并执行现有两项单元/几何测试及独立运行复测。尚未证明干净新机器可重复构建，也未重跑完整比赛矩阵。',
 '- 只创建 `docs/chatgpt_handoff/`及其检查输出；core源码、world、YAML和依赖未修改。':'- 初始调查只创建交接资料；本次授权变更清单见snapshot/frequency_update_20261015.json，涉及6个源码/配置/说明文件。几何、原world、执行器限制和依赖环境未改。',
 '- 导出小文本前记录986个既有文件hash，结束复核。':'- 初始调查保存986个文件hash；当前仅允许频率修订清单中的6项变化，其他初始文件继续核验。',
 '本次未构建 | packages.txt、CMakeLists与历史build日志':'本次频率修订已构建 | snapshot/frequency_update/build.log',
 '本次只查询版本未重编译':'工具版本已核对，本次两包构建成功',
 '本次未执行构建，防止改动已有产物':'初始调查未构建，本次用户授权频率修订已成功构建',
 '[H] 有历史结果；本次未重跑/未构建':'[V/H] 本次频率修订已构建并执行两项单元/几何测试；完整矩阵仍为历史',
 '[V] 数据/图像存在；初始best_effort多话题接收约3–4Hz，CameraInfo约12Hz，配置为15Hz仿真':'[V/H] 前置当前目标30fps、云台15fps，最新实测见修订报告；旧配置低接收率窗口仅作历史',
 '分离RTF、单话题/可靠性/带宽/渲染负载测量，不能直接承诺实时15Hz':'分别检查配置、仿真时间帧率、墙钟接收频率和RTF，不把目标帧率当硬实时保证',
 '明确输入配置、输出、20Hz发送':'明确输入配置、输出、50Hz定时发送',
 '本次没有编写以上任何比赛算法，只有调查、受控检查、文档与小型证据。':'当前仍未编写上述比赛算法；此次仅更新前置采集、现有低层目标/反馈与指令发布频率。',
}
for a,b in replacements.items():s=s.replace(a,b)
marker='# APPLY_FREQUENCY_REVISION_NOTICE'
if marker not in s:
 s+='''\n# APPLY_FREQUENCY_REVISION_NOTICE\nnotice="配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。"\nfor file in D.glob('[0-9][0-9]_*.md'):\n text=file.read_text();first,rest=text.split('\\n',1);file.write_text(first+'\\n\\n'+notice+'\\n'+rest)\n'''
p.write_text(s)
# Update the report packer without rewriting the original read-only-audit baseline.
p=S/'finalize_handoff.py';s=p.read_text().replace('assert not changed,changed',"revision_path=S/'frequency_update_20261015.json'\nrevision=json.loads(revision_path.read_text()) if revision_path.exists() else {}\nallowed=set(revision.get('authorized_changed_files',[]))\nassert set(changed)==allowed,(changed,allowed)")
s=s.replace("'scope':'User confirmed remote5g_ws only; no algorithm/model/config/environment changes'","'scope':'Initial read-only audit followed by user-authorized 30fps / 50Hz configuration and publisher update; geometry, worlds and race algorithms unchanged'")
s=s.replace("# Standalone JSON/CSV interfaces for tools and future teachers.","summary['frequency_revision']=revision\nsummary['initial_runtime_evidence']='runtime/competition and runtime/flat are the original pre-update audit'\n(S/'verification_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))\n# Standalone JSON/CSV interfaces for tools and future teachers.")
s=s.replace("# Record old test results as historical only.","facts=json.loads((S/'key_facts.json').read_text());facts['frequency_configuration_current']=revision;facts['initial_audit_control_observation']=facts.pop('actual_control_observation');(S/'key_facts.json').write_text(json.dumps(facts,indent=2,ensure_ascii=False))\n# Record old test results as historical only.")
s=s.replace('已核实、源码、历史、待确认状态以00和verification_summary.json为准。','已核实、源码、历史、待确认状态以00和verification_summary.json为准。前置30fps/目标50Hz为后续授权修订，见frequency_update_20261015.json；runtime中旧图快照仍保留，不重写历史记录。')
s=s.replace('原相对目录保留；不是可单独运行的完整源码仓库。','原相对目录保留，已刷新授权变更文件；不是可单独运行的完整源码仓库。')
s=s.replace('导出前986个既有文件hash；不是整个home目录清单。','初始导出986个既有文件hash；当前允许修订清单6项变化，其余复核；不是整个home目录清单。')
s=s.replace("combined += ['\\n---\\n\\n# 附录：关键事实机器快照", "combined += ['\\n---\\n', (D/'FREQUENCY_UPDATE_20261015.md').read_text()] if revision else []\ncombined += ['\\n---\\n\\n# 附录：关键事实机器快照")
s=s.replace("'baseline_changes':changed,","'baseline_changes':changed,'authorized_changes_only':True,")
s=s.replace("selected=list(numbered)+[D/'CHATGPT_FULL_CONTEXT.md',D/'COLCON_IGNORE']","selected=list(numbered)+[D/'CHATGPT_FULL_CONTEXT.md',D/'COLCON_IGNORE']\nif revision:selected += [D/'FREQUENCY_UPDATE_20261015.md',S/'frequency_update_20261015.json',S/'product_reference_20261015.md']+list((S/'frequency_update').glob('*'))")
p.write_text(s)
print('Generator, source copies, URDF and revision metadata refreshed; run writer/packer after revision report is written.')
