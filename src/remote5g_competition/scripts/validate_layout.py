#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlsplit
import cv2
import numpy as np
from scene_core import Route,change_path,decode_qr


def validate(path):
 p=Path(path);data=json.loads(p.read_text());cfg=data['config'];route=Route(cfg['workspace'],cfg['start_side'],cfg['lane']);coords=data['coordinates'];lookup={x['model_name']:x for x in coords};assert len(lookup)==len(coords)
 def resource_path(uri):
  parsed=urlsplit(uri)
  if parsed.scheme=='file':return Path(unquote(parsed.path))
  if parsed.scheme=='model':return p.parent/'models'/parsed.netloc/unquote(parsed.path.lstrip('/'))
  raise ValueError('Unsupported resource URI: '+uri)
 assert abs(route.width-data['route']['clear_lane_width'])<1e-10
 assert abs(data['spawn_pose']['s']+data['vehicle_config']['chassis_center_x']+data['vehicle_config']['chassis_length']/2-(cfg['zone_length_ratio']*route.width-cfg['tape_width']-.02))<1e-9
 assert data['region2']['crosswalk_s']<data['region2']['sign_s']<data['region2']['qr_s']
 assert data['region2']['crosswalk_s']-cfg['paper_long']/2>=data['region2']['start_s']+cfg['tape_width']
 assert abs(data['region2']['qr_upper_projection_s']-(data['region2']['end_s']-cfg['tape_width']))<1e-9
 assert data['parking']['slots']['A'][0]>0 and data['parking']['slots']['B'][1]<0
 assert route.straight(lookup['comp_parking_zone']['s'])
 colors='blue' if cfg['track_color']=='red' else 'red'
 assert lookup['comp_obstacle']['model_type']=='comp_cone_'+colors and not route.straight(lookup['comp_obstacle']['s'])
 assert abs(lookup['comp_obstacle']['d'])+cfg['cone_diameter']/2<route.width/2
 world=ET.parse(data['world']).getroot()
 assert len(world.findall(".//model[@name='comp_obstacle']"))==1
 assert len([r for r in coords if 'cone' in r['model_type'] and not r['model_name'].startswith('comp_guide_')])==1
 assert len(data['guide']['rows'])==4
 assert abs(data['guide']['start_s']-data['region2']['end_s']-cfg['guide_start_gap'])<1e-9
 assert data['guide']['start_s']-data['region2']['sign_s']<2.0, 'Guide must immediately follow task2 signs, not a 10m gap'
 sizes={};qr_results={};seen_mesh=set()
 for row in coords:
  sdf=ET.parse(row['sdf']).getroot();m=sdf.find('model');assert m.attrib['name']==row['model_name']
  if 'cone' in row['model_type'] or row['model_type'] in ('comp_start_barrier','comp_qr_stand'):assert m.findall('.//collision')
  else:assert not m.findall('.//collision')
  for mass in m.findall('.//mass'):assert float(mass.text)>0
  for uri in m.findall('.//uri'):
   if uri.text.startswith(('file://','model://')):assert resource_path(uri.text).exists(),uri.text
  for box in m.findall('.//visual/geometry/box/size'):
   v=list(map(float,box.text.split()));sizes.setdefault(row['model_type'],[]).append(v)
  for mesh in m.findall('.//mesh/uri'):
   if mesh.text in seen_mesh:continue
   seen_mesh.add(mesh.text);mp=resource_path(mesh.text);root=ET.parse(mp).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
   vertices=np.array(list(map(float,root.find('.//c:source[@id="positions"]/c:float_array',ns).text.split()))).reshape(-1,3)
   extent=np.ptp(vertices,axis=0)
   if mp.name=='cone.dae':assert np.allclose(extent,[cfg['cone_diameter'],cfg['cone_diameter'],cfg['cone_height']],atol=1e-8)
   if mp.name.endswith('_qr.dae'):assert np.allclose(sorted(extent),[0,cfg['qr_size'],cfg['qr_size']],atol=1e-9)
   if mp.name.endswith('_sign.dae'):assert np.allclose(sorted(extent),[0,.210,.297],atol=1e-9)
 for typ in ('comp_start_zone','comp_region2','comp_parking_zone'):
  assert all(abs(v[2]-cfg['marking_thickness'])<1e-12 for v in sizes[typ])
  assert np.allclose(sizes[typ][0],[.1,route.width,cfg['marking_thickness']])
 assert np.allclose(sizes['comp_start_barrier'][0],[cfg['barrier_thickness'],.8,.5])
 paper=ET.parse(lookup['comp_crosswalk']['sdf']).getroot();visuals=paper.findall('.//visual');ys=[]
 for visual in visuals:
  size=list(map(float,visual.find('geometry/box/size').text.split()));assert np.allclose(size[:2],[.297,.105]);ys.append(float(visual.find('pose').text.split()[1]))
 assert len(ys)%2==1 and 0 in ys and max(abs(y) for y in ys)+.105/2<=route.width/2
 assert np.allclose(np.diff(sorted(ys))-.105,.105,atol=1e-9)
 detector=cv2.QRCodeDetector()
 for name,key in [('comp_qr_stand','qr_content'),('comp_qr_parking_A','parking_qr_A'),('comp_qr_parking_B','parking_qr_B')]:
  file=p.parent/'models'/name/'code.png';im=cv2.imread(str(file));value=decode_qr(im);assert value==cfg[key],(name,value,cfg[key]);qr_results[name]={'decoded':value,'pixels':list(im.shape[:2])}
 import re
 textures=[re.search(r'texture (\S+)',x.read_text()).group(1) for x in (p.parent/'models').glob('*/materials/scripts/*.material')];assert len(set(textures))==len(textures), 'Global Ogre texture names must be unique'
 vehicle=data['vehicle_config'];worst_width=vehicle['front_track']+vehicle['wheel_width']*math.cos(vehicle['joint_steering_limit'])+2*vehicle['wheel_radius']*math.sin(vehicle['joint_steering_limit'])
 max_sine=max(abs(change_path(u,cfg['guide_return_length'],data['guide']['offset'])[1])/math.sqrt(1+change_path(u,cfg['guide_return_length'],data['guide']['offset'])[1]**2) for u in np.linspace(0,1,1001));required=worst_width+vehicle['chassis_length']*max(.25,max_sine)+2*cfg['vehicle_clearance_margin']+cfg['cone_diameter']
 # The coloured boundary ends before the converging yellow boundary closes
 # the bypass. Include the existing car nose when bounding this clearance.
 nose=vehicle['chassis_center_x']+vehicle['chassis_length']/2
 last_u=np.clip((cfg['guide_block_length']+nose+cfg['cone_diameter']/2-cfg['guide_yellow_start_offset']-cfg['guide_yellow_entry_length'])/cfg['guide_return_length'],0,1)
 from scene_core import smoother
 remaining_offset=route.track['lane_width_m']*(1-smoother(last_u))
 clearance=min(route.width,route.track['lane_width_m'],2*(remaining_offset-route.width/2))
 assert clearance>required,(clearance,required)
 maxk=0
 for u in np.linspace(0,1,1001):maxk=max(maxk,abs(change_path(u,cfg['guide_return_length'],data['guide']['offset'])[2]))
 vehiclek=math.tan(vehicle['max_steering'])/vehicle['wheelbase'];assert maxk<vehiclek
 spacings=[]
 for row in data['guide']['rows']:
  direction=1 if row['side'] in ('inner','yellow_left') else -1
  if row['color']=='yellow':
   assert abs(row['d'][0]-direction*(route.width/2+route.track['lane_width_m']))<1e-9
   assert np.all(direction*np.diff(row['d'])<=1e-9)
   assert abs(row['d'][-1]-direction*route.width/2)<1e-9, 'Yellow must return to original lane'
  else:
   assert row['color']==colors and np.allclose(row['d'],direction*route.width/2)
  assert np.allclose(np.diff(row['arc_distances']),.5,atol=1e-12)
  xy=np.array([route.pose(s,d)[:2] for s,d in zip(row['s'],row['d'])]);dist=np.linalg.norm(np.diff(xy,axis=0),axis=1);assert np.max(np.abs(dist-.5))<.001
  spacings.extend(dist.tolist());assert all(route.straight(s) for s in row['s'])
 # Clear of the existing inner kerb and outer edge even in the neighbouring lane.
 for row in coords:
  if row['model_name'].startswith('comp_guide'):
   radius=abs(row['y']);assert route.track['inner_radius_m']+.05+.039<radius<route.track['outer_radius_m']-.039
 # All tangible props clear the spawn envelope; no centre-only heuristic.
 spawn=data['spawn_pose'];radius=math.hypot(vehicle['chassis_length'],worst_width)/2
 for row in coords:
  if row['collision_enabled']:assert math.hypot(row['x']-spawn['x'],row['y']-spawn['y'])>radius+.039
 result={'status':'GEOMETRY_PASS','model_instances':len(coords),'clear_lane_width':route.width,'zone_length':cfg['zone_length_ratio']*route.width,'crosswalk_papers':len(ys),'paper_gap':.105,'qr_textures':qr_results,'guide_row_count':4,'guide_sign_gap_m':data['guide']['start_s']-data['region2']['sign_s'],'bend_obstacle_count':1,'guide_spacing_min_max':[min(spacings),max(spacings)],'vehicle_width_envelope':worst_width,'corridor_width_min_conservative':float(clearance),'required_corridor_width_conservative':required,'max_curve_curvature':maxk,'vehicle_max_curvature':vehiclek,'min_vehicle_radius':1/vehiclek,'autonomous_driving':'NOT_TESTED'}
 return result

def main():
 p=argparse.ArgumentParser();p.add_argument('layout');p.add_argument('--output');a=p.parse_args();result=validate(a.layout);print(json.dumps(result,indent=2));
 if a.output:Path(a.output).write_text(json.dumps(result,indent=2))
if __name__=='__main__':main()
