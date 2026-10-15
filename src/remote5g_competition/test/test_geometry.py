#!/usr/bin/env python3
import sys,tempfile,json,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scene_core import load_config,generate,Route,parking_footprints
from validate_layout import validate
root=Path(__file__).resolve().parents[1];cfg=load_config(root/'config/competition.yaml')
results=[]
with tempfile.TemporaryDirectory(prefix='competition_geometry_') as temp:
 for color in ('red','blue'):
  for side in ('left','right'):
   for change in ('left','right'):
    c=copy.deepcopy(cfg);c.update(track_color=color,start_side=side,lane_change=change)
    out=Path(temp)/f'{color}_{side}_{change}';m=generate(c,root,out);r=validate(out/'layout.json');r.update(color=color,side=side,change=change);results.append(r)
    a=generate(c,root,out);assert a['coordinates']==m['coordinates'] and a['obstacle']==m['obstacle']
    alternate=copy.deepcopy(c);alternate['obstacle_seed']+=1;b=generate(alternate,root,out)
    assert b['obstacle']!=m['obstacle']
    assert [r for r in b['coordinates'] if r['model_name']!='comp_obstacle']==[r for r in m['coordinates'] if r['model_name']!='comp_obstacle']
 # Four boundaries exist in either sign state; only the chosen ground sign differs.
 for color in ('red','blue'):
  for side in ('left','right'):
   c=copy.deepcopy(cfg);c.update(track_color=color,start_side=side)
   a=generate(dict(c,lane_change='left'),root,Path(temp)/'sign_switch')
   b=generate(dict(c,lane_change='right'),root,Path(temp)/'sign_switch')
   assert [r for r in a['coordinates'] if r['model_name'].startswith('comp_guide')]==[r for r in b['coordinates'] if r['model_name'].startswith('comp_guide')]
 # Non-axis-aligned slot test and centre-only false positive.
 region={'x':3,'y':4,'yaw':1.2,'length':1.5,'slots':{'A':[.05,.585],'B':[-.585,-.05]}}
 from scene_core import transform
 for offset,expected in [(.30,'A'),(-.30,'B'),(0,None),(.49,None)]:
  wheels={}
  for name,x,y in [('fl',.23861,.12),('fr',.23861,-.12),('rl',0,.12),('rr',0,-.12)]:
   px,py=transform(x,y+offset,[3,4],1.2);wheels[name]=[px,py,1.2]
  actual=parking_footprints(wheels,region,.04952,.02)
  assert [k for k,v in actual.items() if v['all_four_patches_inside']]==([expected] if expected else [])
print(json.dumps({'status':'GEOMETRY_PASS','configurations':results,'parking_footprint_counterexample':'PASS'},indent=2))
