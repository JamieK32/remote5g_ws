#!/usr/bin/env python3
from pathlib import Path
import json,hashlib,re,zipfile,subprocess,csv,xml.etree.ElementTree as ET
D=Path(__file__).resolve().parent.parent;S=D/'snapshot';W=D.parents[1]
base=json.loads((S/'original_file_hashes.json').read_text());changed=[p for p,x in base.items() if not (W/p).exists() or hashlib.sha256((W/p).read_bytes()).hexdigest()!=x['sha256']]
revision_path=S/'frequency_update_20261015.json'
revision=json.loads(revision_path.read_text()) if revision_path.exists() else {}
repo_path=S/'repository_setup.json'
repository=json.loads(repo_path.read_text()) if repo_path.exists() else {}
allowed=set(revision.get('authorized_changed_files',[]))|set(repository.get('additional_baseline_changes',[]))
assert set(changed)==allowed,(changed,allowed)
comp=json.loads((S/'runtime/competition/runtime.json').read_text());flat=json.loads((S/'runtime/flat/runtime.json').read_text());play=json.loads((S/'playback_observation.json').read_text());checks=json.loads((S/'runtime_checks.json').read_text())
assert comp['status']==flat['status']==play['status']=='PASS'
classified={}
for name,r in checks.items():
 code=r['exit_code'];body=(S/r['file']).read_text()
 if name=='tf_echo':state='VERIFIED_STREAM_BOUNDED_STOP' if '- Translation:' in body else 'INCOMPLETE'
 elif name=='odom_hz':state='VERIFIED_STREAM_BOUNDED_STOP' if 'average rate:' in body else 'INCOMPLETE'
 elif name in ('rviz_start','rqt_image_start'):state='STARTUP_ONLY_BOUNDED_STOP'
 else:state='PASS' if code==0 else 'INCOMPLETE'
 classified[name]=dict(r,status=state)
summary={'snapshot_date':'2026-10-15 Asia/Shanghai','project':str(W),'scope':'Initial read-only audit followed by user-authorized 30fps / 50Hz configuration and publisher update; geometry, worlds and race algorithms unchanged','verified':['ROS2 Humble package resolution','competition launch with real gzclient captures','flat launch and physical small Ackermann motion','installed teleop tool real PTY movement and stop','two RGB images and CameraInfo','odom/joints/diagnostics/TF/clock','get actual vehicle state and center gimbal service','short ros2 bag record/info/play and subscriber reception','RViz OpenGL and rqt_image_view startup'], 'unverified':['doctor report timed out','complete RViz/rqt GUI interactions and saved configuration','clean-machine build','full race algorithms','hardware/5G/audio','headless rendering without WSLg'],'original_files_checked':len(base),'original_files_changed':changed,'audit_ros_domain':95,'audit_gazebo_port':11405,'playback_ros_domain':96,'all_owned_runtime_groups_stopped':True,'checks':classified}
(S/'verification_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
summary['frequency_revision']=revision
summary['repository_setup']=repository
summary['initial_runtime_evidence']='runtime/competition and runtime/flat are the original pre-update audit'
(S/'verification_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
# Standalone JSON/CSV interfaces for tools and future teachers.
with (S/'topics.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['topic','types','publisher_nodes','reliability','durability'])
 for topic,r in sorted(comp['topics'].items()):writer.writerow([topic,';'.join(r['types']),';'.join(p['node'] for p in r['publishers']),';'.join(p['reliability'] for p in r['publishers']),';'.join(p['durability'] for p in r['publishers'])])
(S/'ros_graph.json').write_text(json.dumps(comp['graph'],indent=2))
(S/'tf_tree.json').write_text(json.dumps(comp['tf'],indent=2))
(S/'key_facts.json').write_text(json.dumps({'scope':str(W),'date':'2026-10-15','ros':'ROS2 Humble','gazebo':'Classic11.10.2','model':'xtnetrc','source_packages':['remote5g_sim','remote5g_competition'],'control':{'/ackermann_cmd':'ackermann_msgs/msg/AckermannDriveStamped','/cmd_vel':'geometry_msgs/msg/Twist'},'constant_nodes':list(comp['graph']),'sensors':['front RGB','gimbal RGB'],'no_model_found':['LiDAR','depth','IMU','GPS'],'command_watchdog_wall_seconds':.3,'odom_is_ground_truth':True,'vehicle_yaml':'src/remote5g_sim/config/vehicle.yaml','vehicle_joint_inventory':'vehicle_default_inventory.json','actual_control_observation':{k:flat[k] for k in ('ackermann_motion','keyboard_motion')},'bag_playback':play},indent=2,ensure_ascii=False))
facts=json.loads((S/'key_facts.json').read_text());facts['frequency_configuration_current']=revision;facts['repository_setup']=repository;facts['initial_audit_control_observation']=facts.pop('actual_control_observation');(S/'key_facts.json').write_text(json.dumps(facts,indent=2,ensure_ascii=False))
# Record old test results as historical only.
history=[]
for p in [W/'reports/competition/revised/matrix_results.json',W/'docs/GAZEBO_TEST_REPORT.md',W/'reports/competition/revised_unit_test.log']:
 if p.exists():
  r={'source':str(p.relative_to(W)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'classification':'HISTORICAL_NOT_RERUN'}
  if p.suffix=='.json':r['case_statuses']={n:v.get('status') for n,v in json.loads(p.read_text()).items()}
  history.append(r)
(S/'historical_evidence_index.json').write_text(json.dumps(history,indent=2))
(S/'README.md').write_text('''# Snapshot 使用说明

这是2026-10-15当前remote5g_ws的事实与证据快照。已核实、源码、历史、待确认状态以00和verification_summary.json为准。前置30fps/目标50Hz为后续授权修订，见frequency_update_20261015.json；runtime中旧图快照仍保留，不重写历史记录。

- key_facts.json：关键事实与真实运动/回放结果。
- packages.json：2个源码包与依赖；project_tree.txt：精简工程树。
- topics.csv、ros_graph.json、tf_tree.json：当前运行图、话题/可靠性、TF边。
- vehicle_default.urdf / vehicle_competition.urdf及inventory.json：同一模型两个相机尺寸配置，非新建车辆。
- resources.json：803条资源路径/引用/大小，含派生副本；不复制大型DAE/纹理。
- source/：43份小文本源码/配置副本，原相对目录保留，已刷新授权变更文件；不是可单独运行的完整源码仓库。
- evidence/：本地命令、帮助、退出码和日志；packages_before_snapshot_ignore.txt是修复文档副本重复发现前的历史，不是最终包清单。
- runtime/competition/runtime.json与flat/runtime.json：实际数据摘要，图像只存PNG，不展开百万字节原始数组。
- runtime/competition/sample_bag：本地303KiB样例，metadata和info可读，db3留本地；静止样例不包含图像与控制运动。
- runtime/competition/screenshots：gzclient原生截图，可选择性分享；原始相机PNG在对应runtime目录。
- rules_excerpt.txt、rules_metadata.json：本地规则节选与hash，不声称当届最新。
- original_file_hashes.json：初始导出986个既有文件hash；当前允许修订清单6项变化，其余复核；不是整个home目录清单。
- collect_*.py、probe_runtime.py、verify_runtime.py、verify_playback.py：本次审计辅助；verify_runtime会启动专用仿真并在平坦场发小指令。未经理解不要重跑，不能用于在线实车。
- write_documents.py、finalize_handoff.py：仅生成本交接文档和轻量导出，不是比赛算法。

不含SSH私钥、Token、个人完整环境、真实支付/5G凭据。硬件PWM说明不复制。COLCON_IGNORE隔离整个交接目录，必须保留。

轻量text.zip包含12份Markdown、合并文件和关键文本/源码，不含bag数据库、大资源或运行时派生模型。ChatGPT若需要进一步核对，只上传所需小文件；不能因为看到本机路径就假定能直接读取用户电脑。
''')
# Portable full-context document: all requested docs plus compact machine summaries.
numbered=sorted(D.glob('[0-9][0-9]_*.md'));assert len(numbered)==12
combined=['# Gazebo智能车工程：完整ChatGPT交接上下文\n','日期2026-10-15；范围用户已确认remote5g_ws。下文合并12份文档；Markdown链接指向原导出目录，可在资料包内定位。\n']
for p in numbered:combined += ['\n---\n\n<!-- '+p.name+' -->\n',p.read_text()]
combined += ['\n---\n', (D/'FREQUENCY_UPDATE_20261015.md').read_text()] if revision else []
combined += ['\n---\n\n# 附录：关键事实机器快照\n\n```json\n',(S/'key_facts.json').read_text(),'\n```\n']
(D/'CHATGPT_FULL_CONTEXT.md').write_text(''.join(combined))
# Integrity/document checks; no installation or rebuilding.
errors=[];mermaid_count=0
for p in numbered+[D/'CHATGPT_FULL_CONTEXT.md',S/'README.md']:
 text=p.read_text();fences=[line for line in text.splitlines() if line.startswith('```')]
 if len(fences)%2:errors.append(str(p.name)+': unclosed code fence')
 for link in re.findall(r'\]\(([^)]+)\)',text):
  if link.startswith(('http:','https:','#','mailto:')):continue
  q=link.split('#')[0]
  if not (p.parent/q).exists():errors.append(str(p.name)+': missing link '+link)
 for diagram in re.findall(r'```mermaid\n(.*?)\n```',text,re.S):
  mermaid_count+=1
  if not diagram.startswith(('flowchart ','graph ')):errors.append(str(p.name)+': unsupported diagram type')
  if diagram.count('[')!=diagram.count(']') or diagram.count('"')%2:errors.append(str(p.name)+': unbalanced Mermaid brackets/quotes')
 for bad in ('PLACEHOLDER','TODO_FILL','TF_GRAPH_PLACEHOLDER'):
  if bad in text:errors.append(str(p.name)+': unresolved template '+bad)
assert len(comp['tf'])==19
assert len(comp['graph'])==6
assert set(comp['samples'])>={'/odom','/clock','/camera/front/image_raw','/camera/gimbal/image_raw','/camera/front/camera_info','/camera/gimbal/camera_info'}
assert flat['ackermann_motion']['distance_m']>.02 and flat['keyboard_motion']['distance_m']>.02
assert play['received_counts']['odom']>0 and play['received_counts']['clock']>0
# Credential checks over whitelisted export text, excluding detector source itself.
credential_patterns=[r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----',r'AKIA[0-9A-Z]{16}',r'gh[pousr]_[A-Za-z0-9]{30,}',r'(?<![\w-])sk-[A-Za-z0-9]{25,}']
for p in list((S/'source').rglob('*'))+numbered:
 if p.is_file():
  text=p.read_text(errors='replace')
  if any(re.search(pattern,text) for pattern in credential_patterns):errors.append('possible credential pattern: '+str(p.relative_to(D)))
assert not errors,errors
validation={'status':'PASS','documents':12,'standalone_full_context':True,'link_and_fence_checks':'PASS','mermaid_diagrams_unique':mermaid_count//2,'mermaid_blocks_including_combined':mermaid_count,'mermaid_validation':'Structural checks and source comparison; no external renderer run','source_packages_after_ignore':2,'baseline_file_hashes_checked':len(base),'baseline_changes':changed,'authorized_changes_only':True,'credential_pattern_scan':'No matches in selected exports; only whitelisted files copied','runtime_processes':'All owned groups cleaned','errors':errors}
(S/'document_validation.json').write_text(json.dumps(validation,indent=2,ensure_ascii=False))
# Lightweight package: export only readable text, selected ROS summaries and source.
selected=list(numbered)+[D/'CHATGPT_FULL_CONTEXT.md',D/'COLCON_IGNORE']
if repository:selected.append(S/'repository_setup.json')
if revision:selected += [D/'FREQUENCY_UPDATE_20261015.md',S/'frequency_update_20261015.json',S/'product_reference_20261015.md']+list((S/'frequency_update').glob('*'))
selected+=list((S/'source').rglob('*'))
selected += [S/n for n in ['README.md','key_facts.json','packages.json','project_tree.txt','resources.json','topics.csv','ros_graph.json','tf_tree.json','vehicle_default.urdf','vehicle_competition.urdf','vehicle_default_inventory.json','vehicle_competition_inventory.json','rules_excerpt.txt','rules_metadata.json','verification_summary.json','document_validation.json','historical_evidence_index.json','environment_before_source.json','environment_after_source.json','playback_observation.json','original_file_hashes.json','source_copies.json']]
selected += list((S/'evidence').glob('*.txt'))+[S/'runtime/competition/runtime.json',S/'runtime/flat/runtime.json',S/'runtime/competition/sample_bag/metadata.yaml']
selected=sorted(set(p for p in selected if p.is_file()))
manifest={'date':'2026-10-15','files':{str(p.relative_to(D)):hashlib.sha256(p.read_bytes()).hexdigest() for p in selected},'excludes':['bag *.db3','large meshes/materials/images','build/install/log','private home files']}
mp=D/'handoff_manifest.json';mp.write_text(json.dumps(manifest,indent=2,ensure_ascii=False));selected.append(mp)
archive=D/'chatgpt_handoff_text.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
 for p in selected:z.write(p,p.relative_to(D))
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
(D/'chatgpt_handoff_text.zip.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
print(json.dumps({'validation':validation,'zip_files':len(selected),'zip_bytes':archive.stat().st_size,'combined_bytes':(D/'CHATGPT_FULL_CONTEXT.md').stat().st_size},indent=2))
