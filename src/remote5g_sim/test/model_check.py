#!/usr/bin/env python3
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default='/home/jamiek/remote5g_ws/reports/xtnetrc/model_check.json'); a=p.parse_args()
    share=Path(get_package_share_directory('remote5g_sim'))
    doc=xacro.process_file(str(share/'urdf/car.xacro')); root=ET.fromstring(doc.toxml())
    links={l.attrib['name'] for l in root.findall('link')}; joints=root.findall('joint'); children=[]
    for j in joints:
        assert j.find('parent').attrib['link'] in links and j.find('child').attrib['link'] in links
        children.append(j.find('child').attrib['link'])
    assert len(children)==len(set(children)) and len(links)-len(joints)==1
    assert links-set(children)=={'base_footprint'}
    for l in root.findall('link'):
        i=l.find('inertial')
        if i is None: assert l.attrib['name'].endswith('optical_frame'); continue
        assert float(i.find('mass').attrib['value'])>0
        v=i.find('inertia').attrib
        m=np.array([[float(v['ixx']),float(v['ixy']),float(v['ixz'])],[float(v['ixy']),float(v['iyy']),float(v['iyz'])],[float(v['ixz']),float(v['iyz']),float(v['izz'])]])
        assert min(np.linalg.eigvalsh(m))>0
    assert sum(j.attrib['type']=='continuous' for j in joints)==4
    assert sum(j.attrib['type']=='revolute' for j in joints)==4
    for name in ('front','gimbal'):
        j=root.find(f"joint[@name='{name}_optical_joint']")
        assert np.allclose([float(x) for x in j.find('origin').attrib['rpy'].split()],[-np.pi/2,0,-np.pi/2])
    sensors=root.findall('.//sensor'); assert len(sensors)==2
    output=Path(a.output); output.parent.mkdir(parents=True,exist_ok=True)
    output.with_suffix('.urdf').write_text(doc.toprettyxml())
    output.write_text(json.dumps({'status':'PASS','links':sorted(links),'joints':[j.attrib for j in joints],'positive_inertia':True,'dual_cameras':True},indent=2))
    print(f'PASS: {len(links)} links, {len(joints)} joints, valid inertia and optical frames')

if __name__=='__main__': main()
