"""Rule-aware stadium frame and contact-footprint utilities. Units metres/radians."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

class Route:
    def __init__(self,workspace,side='left',lane=3):
        self.workspace=Path(workspace); self.side=side; self.lane=lane
        self.track=json.loads((self.workspace/'config/athletics_track.json').read_text())
        assert side in ('left','right') and 1<=lane<=self.track['lanes']
        self.half=self.track['straight_length_m']/2
        self.radius=self.track['inner_radius_m']+(lane-0.5)*self.track['lane_width_m']
        # Recover the painted boundaries from the source geometry: lane 1 has a 5cm inner white stripe.
        self.inner=self.track['inner_radius_m']+(lane-1)*self.track['lane_width_m']+(self.track['line_width_m'] if lane==1 else 0)
        self.outer=self.track['inner_radius_m']+lane*self.track['lane_width_m']-self.track['line_width_m']
        self.width=self.outer-self.inner
        self.center=(self.inner+self.outer)/2
        self.radius=self.center
        self.arc_length=math.pi*self.radius
        self.runup=3.0
        self.total=self.runup+self.arc_length+2*self.half
        self.ground=self.track['surface_collision_z_m']
        # Existing worlds must include the existing mesh at identity; fail rather than silently misplace.
        for color in ('red','blue'):
            world=ET.parse(self.workspace/f'simulation/worlds/athletics_400m_{color}.world').getroot().find('world')
            include=world.find('include'); pose=include.findtext('pose','0 0 0 0 0 0')
            assert all(abs(float(x))<1e-12 for x in pose.split()), 'Track world transform is not identity'
            assert include.findtext('uri')==f'model://wa_400m_track_{color}'
    def pose(self,s,d=0):
        # Start at lower mid-straight; take the LEFT or RIGHT half of the oval and end at upper mid-straight.
        assert 0<=s<=self.total
        if s<=self.runup:
            x=self.half-self.runup+s; y=-self.radius; yaw=0
        elif s<=self.runup+self.arc_length:
            theta=-math.pi/2+(s-self.runup)/self.radius
            x=self.half+self.radius*math.cos(theta); y=self.radius*math.sin(theta); yaw=theta+math.pi/2
        else:
            x=self.half-(s-self.runup-self.arc_length); y=self.radius; yaw=math.pi
        if self.side=='left': x=-x; yaw=math.pi-yaw
        # d is always local left, irrespective of world Y or traversal half.
        x-=d*math.sin(yaw); y+=d*math.cos(yaw)
        return [x,y,self.ground,math.atan2(math.sin(yaw),math.cos(yaw))]
    def straight(self,s): return s<self.runup or s>self.runup+self.arc_length
    def metadata(self):
        return {'half_straight':self.half,'radius':self.radius,'clear_lane_width':self.width,
                'nominal_lane_width':self.track['lane_width_m'],'inner_boundary_radius':self.inner,
                'outer_boundary_radius':self.outer,'route_length':self.total,'side':self.side,'lane':self.lane}

def transform(x,y,origin,yaw):
    return [origin[0]+x*math.cos(yaw)-y*math.sin(yaw),origin[1]+x*math.sin(yaw)+y*math.cos(yaw)]

def parking_footprints(wheels,region,tyre_width,contact_length):
    """Four real wheel link poses + tyre contact rectangles; reject centre-only acceptance.
    contact_length is a documented finite-patch assumption for this rigid-cylinder Gazebo model.
    """
    result={}; c=math.cos(region['yaw']); s=math.sin(region['yaw'])
    for slot,(lo,hi) in region['slots'].items():
        inside=[]; coords={}
        for name,pose in wheels.items():
            corners=[]
            for dx in (-contact_length/2,contact_length/2):
                for dy in (-tyre_width/2,tyre_width/2):
                    xy=transform(dx,dy,pose,pose[2]); px=xy[0]-region['x'];py=xy[1]-region['y']
                    corners.append([c*px+s*py,-s*px+c*py])
            coords[name]=corners
            inside.append(all(-region['length']/2<=x<=region['length']/2 and lo<=y<=hi for x,y in corners))
        result[slot]={'all_four_patches_inside':len(wheels)==4 and all(inside),'wheel_corners_local':coords}
    return result

def smoother(u): return 10*u**3-15*u**4+6*u**5

def change_path(u,length,offset):
    y=offset*smoother(u);dy=offset*(30*u**2-60*u**3+30*u**4)/length
    ddy=offset*(60*u-180*u**2+120*u**3)/length**2
    return y,dy,ddy/(1+dy*dy)**1.5

import copy
import hashlib
import csv
import random
import sys
import yaml
from PIL import Image

def element(parent,tag,value=None,**attrs):
    e=ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    if value is not None:e.text=str(value)
    return e

def write_xml(path,root):
    Path(path).parent.mkdir(parents=True,exist_ok=True);ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=Path(path).suffix!='.sdf')

def material(parent,color):
    m=element(parent,'material');element(m,'ambient',color);element(m,'diffuse',color);element(m,'specular','0 0 0 1')

def box(link,name,size,pose,color,collision=False):
    visual=element(link,'visual',name=name);element(visual,'pose',' '.join(map(str,pose)))
    geom=element(visual,'geometry');element(element(geom,'box'),'size',' '.join(map(str,size)));material(visual,color);element(visual,'cast_shadows','false')
    if collision:
        c=element(link,'collision',name=name);element(c,'pose',' '.join(map(str,pose)));c.append(copy.deepcopy(geom))
    return visual

def model(name,static=True,mass=0.1,cog=(0,0,0.04),inertia=(0.001,0.001,0.001)):
    sdf=ET.Element('sdf',version='1.6');m=element(sdf,'model',name=name);element(m,'static',str(static).lower());l=element(m,'link',name='body')
    i=element(l,'inertial');element(i,'pose',f'{cog[0]} {cog[1]} {cog[2]} 0 0 0');element(i,'mass',mass);matrix=element(i,'inertia')
    for key,val in zip(('ixx','iyy','izz','ixy','ixz','iyz'),(*inertia,0,0,0)):element(matrix,key,val)
    return sdf,m,l

def mesh_file(path,vertices,triangles,uv=None):
    # Small Collada mesh with explicit UVs; native geometry, no generated-image assets.
    ns='http://www.collada.org/2005/11/COLLADASchema';ET.register_namespace('',ns)
    def t(parent,tag,val=None,**attrs):return element(parent,'{'+ns+'}'+tag,val,**attrs)
    root=ET.Element('{'+ns+'}COLLADA',version='1.4.1');asset=t(root,'asset');t(asset,'unit',name='meter',meter='1');t(asset,'up_axis','Z_UP')
    geometries=t(root,'library_geometries');g=t(geometries,'geometry',id='geometry');mesh=t(g,'mesh')
    source=t(mesh,'source',id='positions');flat=[a for v in vertices for a in v];t(source,'float_array',' '.join(map(str,flat)),id='position_array',count=len(flat));acc=t(t(source,'technique_common'),'accessor',source='#position_array',count=len(vertices),stride='3')
    for name in ('X','Y','Z'):t(acc,'param',name=name,type='float')
    if uv:
        s=t(mesh,'source',id='uv');flat=[a for v in uv for a in v];t(s,'float_array',' '.join(map(str,flat)),id='uv_array',count=len(flat));acc=t(t(s,'technique_common'),'accessor',source='#uv_array',count=len(uv),stride='2')
        for name in ('S','T'):t(acc,'param',name=name,type='float')
    vs=t(mesh,'vertices',id='vertices');t(vs,'input',semantic='POSITION',source='#positions')
    tri=t(mesh,'triangles',count=len(triangles));t(tri,'input',semantic='VERTEX',source='#vertices',offset='0')
    if uv:t(tri,'input',semantic='TEXCOORD',source='#uv',offset='1',set='0')
    inds=[i for triangle in triangles for i in triangle];t(tri,'p',' '.join(str(j) for i in inds for j in ((i,i) if uv else (i,))))
    scene=t(root,'library_visual_scenes');v=t(scene,'visual_scene',id='scene');t(t(v,'node',id='node'),'instance_geometry',url='#geometry');t(t(root,'scene'),'instance_visual_scene',url='#scene');write_xml(path,root)

def textured(link,root,name,length,width,texture,pose=(0,0,0,0,0,0),upright=False,white_margin=0):
    dest=Path(root);(dest/'meshes').mkdir(parents=True,exist_ok=True);(dest/'materials/scripts').mkdir(parents=True,exist_ok=True);(dest/'materials/textures').mkdir(parents=True,exist_ok=True)
    texture_name=dest.name+'_'+name+'_'+hashlib.sha256(Path(texture).read_bytes()+str((length,width,white_margin)).encode()).hexdigest()[:8]+'.png'
    image=Image.open(texture).convert('RGB')
    if white_margin:
        px=round(image.width*white_margin/width);py=round(image.height*white_margin/length)
        interior=image.resize((image.width-2*px,image.height-2*py),Image.LANCZOS);paper=Image.new('RGB',image.size,'white');paper.paste(interior,(px,py));image=paper
    image.save(dest/f'materials/textures/{texture_name}')
    # Image right points vehicle-right (-Y); top points +X for ground or +Z for upright.
    if upright:verts=[(0,width/2,length/2),(0,-width/2,length/2),(0,-width/2,-length/2),(0,width/2,-length/2)]
    else:verts=[(length/2,width/2,0),(length/2,-width/2,0),(-length/2,-width/2,0),(-length/2,width/2,0)]
    mesh_file(dest/f'meshes/{dest.name}_{name}.dae',verts,[(0,1,2),(0,2,3)],[(0,1),(1,1),(1,0),(0,0)])
    matname='competition/'+dest.name+'/'+name
    (dest/f'materials/scripts/{name}.material').write_text(f'material {matname}\n{{\n technique {{ pass {{ lighting off\n cull_hardware none\n cull_software none\n texture_unit {{ texture {texture_name}\n filtering none\n }} }} }}\n}}\n')
    visual=element(link,'visual',name=name);element(visual,'pose',' '.join(map(str,pose)));geometry=element(visual,'geometry');element(element(geometry,'mesh'),'uri',f'model://{dest.name}/meshes/{dest.name}_{name}.dae')
    mat=element(visual,'material');script=element(mat,'script');element(script,'uri',f'model://{dest.name}/materials/scripts');element(script,'uri',f'model://{dest.name}/materials/textures');element(script,'name',matname);element(visual,'cast_shadows','false')

def assets(cfg,width,root,source):
    root=Path(root);source=Path(source);root.mkdir(parents=True,exist_ok=True);templates={}
    def save(name,sdf):
        folder=root/name;folder.mkdir(parents=True,exist_ok=True);write_xml(folder/'model.sdf',sdf)
        config=ET.Element('model');element(config,'name',name);element(config,'version','1.0');element(config,'sdf','model.sdf',version='1.6');element(config,'description','Remote5G rule traced simulation prop');write_xml(folder/'model.config',config)
        templates[name]=sdf.find('model')
    length=cfg['zone_length_ratio']*width;line=cfg['tape_width'];th=cfg['marking_thickness'];z=cfg['marking_bottom_z']+th/2
    for name in ('comp_start_zone','comp_region2','comp_parking_zone'):
        sdf,m,l=model(name)
        for i,x in enumerate((-length/2+line/2,length/2-line/2)):box(l,f'yellow_{i}',(line,width,th),(x,0,z,0,0,0),'1 0.85 0 1')
        if name=='comp_parking_zone':box(l,'yellow_divider',(length-line*2,line,th),(0,0,z,0,0,0),'1 0.85 0 1')
        save(name,sdf)
    sdf,m,l=model('comp_start_barrier');box(l,'barrier',(cfg['barrier_thickness'],cfg['barrier_width'],cfg['barrier_height']),(0,0,cfg['barrier_height']/2,0,0,0),'0.02 0.05 0.95 1',True);save('comp_start_barrier',sdf)
    n=int(math.floor((width+cfg['paper_gap'])/(cfg['paper_short']+cfg['paper_gap'])));n=n if n%2 else n-1
    sdf,m,l=model('comp_crosswalk')
    for i in range(-(n//2),n//2+1):box(l,f'paper_{i+n//2}',(cfg['paper_long'],cfg['paper_short'],th),(0,i*(cfg['paper_short']+cfg['paper_gap']),z,0,0,0),'1 1 1 1')
    save('comp_crosswalk',sdf)
    for color,rgb in [('red','1 0.02 0.02 1'),('blue','0.02 0.25 1 1'),('yellow','1 0.85 0 1')]:
        name='comp_cone_'+color;dest=root/name;dest.mkdir(parents=True,exist_ok=True)
        radius=cfg['cone_diameter']/2;h=cfg['cone_height'];seg=cfg['cone_segments'];vertices=[];tris=[]
        profile=[(0,radius),(0.004,radius),(0.004,radius*0.88),(h,0.008)]
        for height,r in profile:
            vertices.extend((r*math.cos(2*math.pi*i/seg),r*math.sin(2*math.pi*i/seg),height) for i in range(seg))
        for j in range(len(profile)-1):
            for i in range(seg):
                a=j*seg+i;b=j*seg+(i+1)%seg;c=b+seg;d=a+seg;tris.extend([(a,b,c),(a,c,d)])
        vertices.extend([(0,0,0),(0,0,h)])
        for i in range(seg):tris.extend([(len(vertices)-2,(i+1)%seg,i),(len(vertices)-1,3*seg+i,3*seg+(i+1)%seg)])
        mesh_file(dest/'meshes/cone.dae',vertices,tris)
        mass=cfg['cone_mass'];inertia=(mass*(3*radius**2+h*h)/12,)*2+(mass*radius**2/2,)
        sdf,m,l=model(name,cfg['cone_static'],mass,(0,0,0.025),inertia)
        for typ in ('visual','collision'):
            shape=element(l,typ,name='cone');geometry=element(shape,'geometry');element(element(geometry,'mesh'),'uri',f'model://{dest.name}/meshes/cone.dae')
            if typ=='visual':material(shape,rgb)
            else:
                surface=element(shape,'surface');ode=element(element(surface,'friction'),'ode');element(ode,'mu',cfg['cone_mu']);element(ode,'mu2',cfg['cone_mu']);ode=element(element(surface,'contact'),'ode');element(ode,'kp',100000);element(ode,'kd',2);element(ode,'max_vel',0.1);element(ode,'min_depth',0.00005)
        save(name,sdf)
    for track in ('red','blue'):
        for sign in ('left','right','A','B'):
            name=f'comp_sign_{sign}_{track}';sdf,m,l=model(name)
            image=next(source.glob(sign+'_'+track+'.*'));letter=sign in ('A','B')
            a,b=(cfg['sign_width'],cfg['sign_length']) if letter else (cfg['sign_length'],cfg['sign_width'])
            textured(l,root/name,'sign',a,b,image,(0,0,z+th,0,0,0),white_margin=cfg['print_white_margin']);save(name,sdf)
    try:import qrcode
    except ImportError:
        sys.path.insert(0,str(Path.home()/'.cache/remote5g_scene_libs'));import qrcode
    for name,key,stand in [('comp_qr_stand','qr_content',True),('comp_qr_parking_A','parking_qr_A',False),('comp_qr_parking_B','parking_qr_B',False)]:
        folder=root/name;folder.mkdir(parents=True,exist_ok=True);qr=qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M,box_size=20,border=4);qr.add_data(cfg[key]);qr.make(fit=True);png=folder/'code.png';qr.make_image(fill_color='black',back_color='white').save(png)
        sdf,m,l=model(name,True,cfg['stand_mass']);angle=math.radians(cfg['qr_pitch_deg']);height=cfg['qr_center_height'];size=cfg['qr_size']
        if stand:
            # Upright plane normal -X rotated +pitch about Y: inclined up and downstream.
            pitch=math.pi/2-angle
            panel=cfg['stand_panel_height'];center=height
            box(l,'front_panel',(cfg['stand_thickness'],cfg['stand_width'],panel),(0,0,center,0,pitch,0),'1 1 1 1',True)
            bottom=-panel*math.sin(pitch)/2;top=panel*math.sin(pitch)/2
            support=max(height-panel*math.cos(pitch)/2,0.004)
            box(l,'front_support',(0.006,cfg['stand_width'],support),(bottom,0,support/2,0,0,0),'0.15 0.15 0.15 1',True)
            # Rear folding leg and small grounded base; dimensions are simulation assumptions.
            box(l,'rear_leg',(cfg['stand_thickness'],cfg['stand_width'],0.20),(0.11,0,0.10,0,-0.45,0),'0.15 0.15 0.15 1',True)
            box(l,'base',(0.26,cfg['stand_width'],0.004),(0.04,0,0.002,0,0,0),'0.2 0.2 0.2 1',True)
            # Offset toward incoming vehicle so texture is not buried in the display board.
            offset=-cfg['stand_thickness']/2-0.0003
            textured(l,folder,'qr',size,size,png,(offset*math.cos(pitch),0,height-offset*math.sin(pitch),0,pitch,0),True)
        else:textured(l,folder,'qr',size,size,png,(0,0,z+2*th,0,0,0))
        save(name,sdf)
    return templates

def default_workspace():
    """Locate installed track data, with a fallback for source-only geometry tests."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed=Path(get_package_share_directory('remote5g_sim'))/'track'
        if (installed/'config/athletics_track.json').is_file():return installed
    except (ImportError, KeyError):
        pass
    for candidate in Path(__file__).resolve().parents:
        if (candidate/'config/athletics_track.json').is_file() and (candidate/'simulation/worlds').is_dir():return candidate
    raise FileNotFoundError('Track data unavailable. Build remote5g_sim and source install/setup.bash.')

def load_config(path,override=None):
    path=Path(path).expanduser().resolve()
    cfg=yaml.safe_load(path.read_text());cfg.update(override or {})
    if cfg.get('workspace'):
        workspace=Path(cfg['workspace']).expanduser()
        if not workspace.is_absolute():workspace=path.parent/workspace
    else:
        workspace=default_workspace()
    cfg['workspace']=str(workspace.resolve())
    return cfg

def vehicle_config_path(workspace):
    # Explicit source workspaces retain their own vehicle parameters.
    source=Path(workspace)/'src/remote5g_sim/config/vehicle.yaml'
    if source.is_file():return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('remote5g_sim'))/'config/vehicle.yaml'

def generate(cfg,package,output):
    rules={'tape_width':.10,'zone_length_ratio':1.5,'barrier_width':.80,'barrier_height':.50,'barrier_gap':.10,'qr_size':.10,'paper_long':.297,'paper_short':.105,'paper_gap':.105,'cone_height':.080,'cone_diameter':.078,'guide_spacing':.50,'sign_width':.210,'sign_length':.297}
    for key,value in rules.items():
        if abs(float(cfg[key])-value)>1e-10:raise ValueError(f'{key} must retain rule dimension {value}; use separate experimental models to change official sizes')
    package=Path(package);output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    route=Route(cfg['workspace'],cfg['start_side'],cfg['lane']);w=route.width;length=cfg['zone_length_ratio']*w
    templates=assets(cfg,w,output/'models',package/'materials/source_signs');rng=random.Random(cfg['obstacle_seed']);srng=random.Random(cfg['sign_seed'])
    change=srng.choice(['left','right']) if cfg['lane_change']=='random' else cfg['lane_change'];target=srng.choice(['A','B']) if cfg['parking_target']=='random' else cfg['parking_target']
    assert change in ('left','right') and target in ('A','B')
    sign=1 if change=='left' else -1;colored='blue' if cfg['track_color']=='red' else 'red'
    area2_start=route.runup+route.arc_length+cfg['region2_top_straight_offset'];area2_end=area2_start+length
    cross_s=area2_start+cfg['tape_width']+cfg['paper_long']/2+0.01;arrow_s=cross_s+cfg['paper_long']/2+cfg['sign_length']/2
    parking_s=route.total-cfg['parking_end_margin']-length/2
    guide_start=area2_end+cfg['guide_start_gap']
    guide_length=cfg['guide_yellow_start_offset']+cfg['guide_yellow_entry_length']+cfg['guide_return_length']+cfg['guide_straight_length']
    assert cfg['guide_start_gap']>=0 and cfg['guide_block_length']>0
    assert 0<=cfg['guide_yellow_start_offset'] and cfg['guide_yellow_entry_length']>=0
    assert cfg['guide_return_length']>0 and cfg['guide_straight_length']>0
    assert guide_start+guide_length<parking_s-length/2-cfg['parking_sign_distance']
    world=ET.parse(Path(cfg['workspace'])/f"simulation/worlds/athletics_400m_{cfg['track_color']}.world").getroot();worldnode=world.find('world');worldnode.set('name','competition_scene');worldnode.find('scene/shadows').text='false'
    instance_dir=output/'instances';instance_dir.mkdir(exist_ok=True);coords=[]
    def add(name,typ,s,d=0,z=0,yaw_offset=0,reason='rule dimension, configurable simulation station'):
        pose=route.pose(s,d);m=copy.deepcopy(templates[typ]);m.set('name',name);p=element(m,'pose',f'{pose[0]} {pose[1]} {pose[2]+z} 0 0 {pose[3]+yaw_offset}');worldnode.append(m)
        wrapper=ET.Element('sdf',version='1.6');wrapper.append(copy.deepcopy(m));write_xml(instance_dir/f'{name}.sdf',wrapper)
        dimensions={'zone_length':length,'lane_width':w,'yellow_width':cfg['tape_width']}
        if 'cone' in typ:dimensions={'height':cfg['cone_height'],'diameter':cfg['cone_diameter']}
        elif 'barrier' in typ:dimensions={'width':cfg['barrier_width'],'height':cfg['barrier_height'],'thickness':cfg['barrier_thickness']}
        elif 'crosswalk' in typ:dimensions={'paper_long':cfg['paper_long'],'paper_short':cfg['paper_short'],'gap':cfg['paper_gap']}
        elif 'qr' in typ:dimensions={'qr_width':cfg['qr_size'],'qr_height':cfg['qr_size'],'stand_panel_width':cfg['stand_width'] if 'stand' in typ else 0}
        elif 'sign' in typ:dimensions={'A4_short':cfg['sign_width'],'A4_long':cfg['sign_length']}
        coords.append({'model_name':name,'model_type':typ,'x':pose[0],'y':pose[1],'z':pose[2]+z,'roll':0,'pitch':0,'yaw':pose[3]+yaw_offset,'s':s,'d':d,'dimensions':dimensions,'collision_enabled':bool(m.findall('.//collision')),'source_or_assumption':reason,'sdf':str(instance_dir/f'{name}.sdf')})
    add('comp_start_zone','comp_start_zone',length/2)
    board_s=length+cfg['barrier_gap']+cfg['barrier_thickness']/2
    # Save barrier instance even in removed state, for repeatable restoration.
    add('comp_start_barrier','comp_start_barrier',board_s)
    if not cfg['barrier_present']:worldnode.remove(worldnode.find("model[@name='comp_start_barrier']"))
    add('comp_region2','comp_region2',area2_start+length/2,reason='PDF fig3 annotated yellow region interpreted after crosswalk; its exact extent is an assumption')
    add('comp_crosswalk','comp_crosswalk',cross_s)
    add('comp_lane_change_sign',f'comp_sign_{change}_{cfg["track_color"]}',arrow_s)
    # Upper QR texture edge projection equals downstream yellow INNER edge.
    pitch=math.pi/2-math.radians(cfg['qr_pitch_deg']);offset=-cfg['stand_thickness']/2-0.0003
    upper_delta=(cfg['qr_size']/2*math.sin(pitch)+offset*math.cos(pitch))*math.cos(math.radians(cfg['qr_yaw_offset_deg']))
    qr_s=area2_end-cfg['tape_width']-upper_delta
    add('comp_task2_qr','comp_qr_stand',qr_s,yaw_offset=math.radians(cfg['qr_yaw_offset_deg']),reason='Fig3 interpretation: following task2 crosswalk/sign, leaning display 45deg; upper edge aligned inner yellow edge')
    add('comp_parking_zone','comp_parking_zone',parking_s)
    add('comp_parking_instruction',f'comp_sign_{target}_{cfg["track_color"]}',parking_s-length/2-cfg['parking_sign_distance'])
    for slot,d in [('A',(w+cfg['tape_width'])/4),('B',-(w+cfg['tape_width'])/4)]:
        add(f'comp_parking_label_{slot}',f'comp_sign_{slot}_{cfg["track_color"]}',parking_s-length/2+0.40,d)
        add(f'comp_parking_qr_{slot}',f'comp_qr_parking_{slot}',parking_s+length/2-0.23,d)
    if cfg['obstacle_mode']=='random':
        obs_s=rng.uniform(route.runup+1,route.runup+route.arc_length-1);obs_d=rng.uniform(-w/2+0.12,w/2-0.12)
    else:obs_s=float(cfg['obstacle_s']);obs_d=float(cfg['obstacle_lateral_offset'])
    assert route.runup<obs_s<route.runup+route.arc_length and abs(obs_d)+cfg['cone_diameter']/2<w/2
    add('comp_obstacle',f'comp_cone_{colored}',obs_s,obs_d,z=0.001)
    guide=[]
    # User's full four-boundary image combines both mirrored Fig11 alternatives:
    # two short straight coloured rows on the original lane boundaries, and
    # two yellow outer boundaries converging FROM adjacent lanes TO that lane.
    # The selected sign chooses the bypass; it does not remove the other row.
    for side,direction,yellow in [('inner',1,False),('outer',-1,False),('yellow_left',1,True),('yellow_right',-1,True)]:
        start=guide_start+(cfg['guide_yellow_start_offset'] if yellow else 0)
        row_length=(cfg['guide_yellow_entry_length']+cfg['guide_return_length']+cfg['guide_straight_length']) if yellow else cfg['guide_block_length']
        grid=np.linspace(0,row_length,4001)
        if yellow:
            u=np.clip((grid-cfg['guide_yellow_entry_length'])/cfg['guide_return_length'],0,1)
            d=direction*(w/2+route.track['lane_width_m']*(1-smoother(u)))
        else:d=np.full_like(grid,direction*w/2)
        distances=np.r_[0,np.cumsum(np.hypot(np.diff(grid),np.diff(d)))];samples=np.arange(0,distances[-1]+1e-8,cfg['guide_spacing'])
        xx=np.interp(samples,distances,grid);dd=np.interp(samples,distances,d)
        for i,(x,lateral) in enumerate(zip(xx,dd)):
            s=start+float(x);add(f'comp_guide_{side}_{i:03}',f'comp_cone_{"yellow" if yellow else colored}',s,float(lateral),z=0.001,reason='PDF Fig11 and user four-row image; local boundary topology, adjustable longitudinal extent')
        guide.append({'side':side,'color':'yellow' if yellow else colored,'start_s':start,'end_s':start+row_length,'s':[start+float(x) for x in xx],'d':dd.tolist(),'arc_distances':samples.tolist()})
    veh=yaml.safe_load(vehicle_config_path(cfg['workspace']).read_text());nose=veh['chassis_center_x']+veh['chassis_length']/2
    spawn_s=length-cfg['tape_width']-0.02-nose;spawn=route.pose(spawn_s);spawn[2]+=0.006
    parkpose=route.pose(parking_s)
    region={'x':parkpose[0],'y':parkpose[1],'yaw':parkpose[3],'length':length-2*cfg['tape_width'],'width':w,'slots':{'A':[cfg['tape_width']/2,w/2],'B':[-w/2,-cfg['tape_width']/2]}}
    plugin=element(worldnode,'plugin',name='competition_state',filename='libgazebo_ros_state.so');element(element(plugin,'ros'),'namespace','/gazebo');element(plugin,'update_rate',20)
    gui=worldnode.find('gui/camera/pose');gui.text=f'{spawn[0]} {spawn[1]} 5 0 {math.pi/2} {math.pi/2}'
    write_xml(output/'competition.world',world)
    spawn_dict={'x':spawn[0],'y':spawn[1],'z':spawn[2],'yaw':spawn[3],'s':spawn_s,'source':'route_derived'}
    for key,value in cfg.get('vehicle_spawn',{}).items():
        if value is not None:spawn_dict[key]=float(value);spawn_dict['source']='explicit_simulation_override'
    manifest={'config':cfg,'resolved_lane_change':change,'resolved_parking_target':target,'route':route.metadata(),'spawn_pose':spawn_dict,'parking':region,'obstacle':{'seed':cfg['obstacle_seed'],'s':obs_s,'d':obs_d},'region2':{'start_s':area2_start,'end_s':area2_end,'crosswalk_s':cross_s,'sign_s':arrow_s,'qr_s':qr_s,'qr_upper_projection_s':qr_s+upper_delta,'stop_reference_s':cross_s-cfg['paper_long']/2-0.30},'guide':{'pattern':'four_boundaries_dual_bypass_return','start_s':guide_start,'length':guide_length,'offset':sign*route.track['lane_width_m'],'rows':guide},'coordinates':coords,'world':str(output/'competition.world'),'vehicle_config':veh}
    (output/'layout.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False));(output/'resolved.yaml').write_text(yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False))
    with (output/'coordinates.csv').open('w') as f:
        keys=['model_name','model_type','x','y','z','roll','pitch','yaw','s','d','dimensions','collision_enabled','source_or_assumption'];writer=csv.DictWriter(f,keys,extrasaction='ignore');writer.writeheader();writer.writerows(coords)
    return manifest


def decode_qr(image):
    try:import zxingcpp
    except ImportError:
        sys.path.insert(0,str(Path.home()/'.cache/remote5g_scene_libs'));import zxingcpp
    result=zxingcpp.read_barcode(image,formats=zxingcpp.BarcodeFormat.QRCode)
    return result.text if result else ''


def qr_mapping_check(image,texture):
    try:import zxingcpp
    except ImportError:
        sys.path.insert(0,str(Path.home()/'.cache/remote5g_scene_libs'));import zxingcpp
    import cv2
    result=zxingcpp.read_barcode(image,formats=zxingcpp.BarcodeFormat.QRCode)
    if not result:return None
    pts=result.position
    corners=np.array([[p.x,p.y] for p in (pts.top_left,pts.top_right,pts.bottom_right,pts.bottom_left)],np.float32)
    original=cv2.imread(str(texture),cv2.IMREAD_GRAYSCALE);n=original.shape[0]//20-8
    matrix=(original[90:original.shape[0]-80:20,90:original.shape[1]-80:20]<128)
    size=n*10;H=cv2.getPerspectiveTransform(corners,np.array([[0,0],[size,0],[size,size],[0,size]],np.float32))
    gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY) if image.ndim==3 else image
    warp=cv2.warpPerspective(gray,H,(size,size));actual=warp[5::10,5::10]<128
    return {'orientation_deg':result.orientation,'module_error_fraction':float(np.mean(actual!=matrix)),'decoded':result.text}
