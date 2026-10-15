#!/usr/bin/env python3
import argparse
from pathlib import Path
from scene_core import generate,load_config

def main():
 p=argparse.ArgumentParser();p.add_argument('--config');p.add_argument('--output',required=True);p.add_argument('--start-side',choices=['left','right']);p.add_argument('--track-color',choices=['red','blue']);p.add_argument('--lane-change',choices=['left','right','random']);p.add_argument('--obstacle-seed',type=int);p.add_argument('--obstacle-mode',choices=['random','fixed']);p.add_argument('--obstacle-s',type=float);p.add_argument('--obstacle-lateral-offset',type=float);p.add_argument('--qr-content');a=p.parse_args()
 from ament_index_python.packages import get_package_share_directory
 package=Path(get_package_share_directory('remote5g_competition'));override={k:v for k,v in vars(a).items() if v is not None and k not in ('config','output')};cfg=load_config(a.config or package/'config/competition.yaml',override);m=generate(cfg,package,a.output);print(f"Generated {len(m['coordinates'])} props; W={m['route']['clear_lane_width']:.3f}m, route={m['route']['route_length']:.3f}m; {m['world']}")
if __name__=='__main__':main()
