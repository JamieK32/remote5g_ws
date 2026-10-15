#!/usr/bin/env python3
"""Read-only project inventory. Writes only inside this handoff directory."""
from pathlib import Path
import os,json,hashlib,subprocess,sys,xml.etree.ElementTree as ET,shutil
root=Path(__file__).resolve().parents[3];out=Path(__file__).resolve().parent
excluded={'build','install','log','.git','__pycache__','chatgpt_handoff'}
files=[]
for folder in ('src','scripts','config','simulation','docs'):
 for path in (root/folder).rglob('*'):
  if path.is_file() and not any(x in excluded for x in path.relative_to(root).parts):files.append(path)
files+=[root/'README.md']
manifest={str(p.relative_to(root)):{'size':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}
(out/'original_file_hashes.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False))
packages=[]
for path in (root/'src').glob('*/package.xml'):
 x=ET.parse(path).getroot();packages.append({'name':x.findtext('name'),'version':x.findtext('version'),'path':str(path.parent.relative_to(root)),'dependencies':[{'kind':e.tag,'name':e.text} for e in x if 'depend' in e.tag]})
(out/'packages.json').write_text(json.dumps(packages,indent=2))
# Only a whitelist of small source/config text, excluding credentials and large resources.
selected=[p for p in (root/'src').rglob('*') if p.is_file() and not any(x in ('worlds','models','materials','__pycache__') for x in p.relative_to(root/'src').parts) and p.suffix in ('.py','.cpp','.hpp','.xacro','.xml','.yaml','.txt') and p.stat().st_size<90000]
selected += [root/'config/athletics_track.json',root/'scripts/launch_track.sh',root/'scripts/test_xtnetrc.sh']
selected += list((root/'simulation/worlds').glob('*.world'))+list((root/'simulation/models').glob('*/model.sdf'))
copied=[]
for p in selected:
 if not p.exists():continue
 if p.name=='hardware_reference_NOT_LOADED.yaml':continue
 dest=out/'source'/p.relative_to(root);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest);copied.append(str(p.relative_to(root)))
(out/'source_copies.json').write_text(json.dumps(copied,indent=2))
# Full resource inventory records paths/sizes/references, not mesh payload.
resources=[]
for p in files:
 if p.suffix.lower() not in ('.sdf','.world','.xacro','.dae','.png','.jpg','.jpeg','.material','.rviz'):continue
 r={'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'kind':p.suffix.lower()}
 if p.suffix in ('.sdf','.world'):
  try:
   x=ET.parse(p).getroot();r['uris']=[e.text for e in x.findall('.//uri')];r['plugins']=[e.attrib for e in x.findall('.//plugin')];r['includes']=[e.findtext('uri') for e in x.findall('.//include')]
  except ET.ParseError as e:r['error']=str(e)
 resources.append(r)
(out/'resources.json').write_text(json.dumps(resources,indent=2,ensure_ascii=False))
lines=['/home/jamiek/remote5g_ws/']
for p in sorted(root.iterdir()):
 if p.name=='.git':continue
 lines.append(f'├── {p.name}'+('/' if p.is_dir() else ''))
 if p.is_dir() and p.name not in ('build','install','log','reports','artifacts','deliverables'):
  for q in sorted(p.iterdir()):
   if q.name=='chatgpt_handoff':continue
   lines.append(f'│   ├── {q.name}'+('/' if q.is_dir() else ''))
   if q.is_dir() and p.name=='src':
    for z in sorted(q.iterdir()):
     if z.name!='__pycache__':lines.append(f'│   │   ├── {z.name}'+('/' if z.is_dir() else ''))
(out/'project_tree.txt').write_text('\n'.join(lines)+'\n')
allow=['SHELL','PATH','ROS_DISTRO','ROS_VERSION','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','DISPLAY','WAYLAND_DISPLAY','WSL_DISTRO_NAME','VIRTUAL_ENV','CONDA_DEFAULT_ENV','CONDA_PREFIX','GAZEBO_MASTER_URI','GAZEBO_MODEL_PATH','GAZEBO_PLUGIN_PATH']
(out/'environment_before_source.json').write_text(json.dumps({k:os.environ.get(k) for k in allow},indent=2))
print('Project',root,'protected files',len(manifest),'packages',len(packages),'small source copies',len(copied),'resources',len(resources))
