#!/usr/bin/env python3
"""Generate self-contained Gazebo Classic SDF 1.6 models; metres, Z up."""
import csv
from lib.geos_planar import Planar
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
R, L, W, N, LINE = 36.50, 84.39, 1.22, 8, 0.05
H, OUTER, SEGMENTS = L / 2, R + N * W, 1024
COLORS = {'red': (0.55, 0.105, 0.075, 1), 'blue': (0.035, 0.30, 0.56, 1)}

class Mesh:
    def __init__(self):
        self.v, self.t = [], []
    def quad(self, a, b, c, d):
        k = len(self.v)
        self.v.extend([a, b, c, d])
        self.t.extend([(k,k+1,k+2), (k,k+2,k+3)])
    def ring(self, ri, ro, z):
        # CCW outer capsule and inner capsule, identical indexing.
        a, b = capsule(ri), capsule(ro)
        for i in range(len(a)):
            j = (i+1) % len(a)
            self.quad((*a[i],z), (*b[i],z), (*b[j],z), (*a[j],z))
    def rect(self, x0, x1, y0, y1, z):
        self.quad((x0,y0,z),(x1,y0,z),(x1,y1,z),(x0,y1,z))
    def solid_ring(self, ri, ro, height):
        self.ring(ri,ro,height)
        a,b = capsule(ri), capsule(ro)
        for i in range(len(a)):
            j=(i+1)%len(a)
            self.quad((*b[i],0),(*b[j],0),(*b[j],height),(*b[i],height))
            self.quad((*a[j],0),(*a[i],0),(*a[i],height),(*a[j],height))
            self.quad((*a[j],0),(*b[j],0),(*b[i],0),(*a[i],0))
    def save(self, path):
        ns='http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('',ns)
        def el(parent,element_name,text=None,**attrs):
            child=ET.SubElement(parent,'{'+ns+'}'+element_name,attrs)
            if text is not None: child.text=str(text)
            return child
        root=ET.Element('{'+ns+'}COLLADA',{'version':'1.4.1'})
        asset=el(root,'asset'); el(asset,'created','2026-10-14T00:00:00Z'); el(asset,'modified','2026-10-14T00:00:00Z')
        el(asset,'unit',name='meter',meter='1'); el(asset,'up_axis','Z_UP')
        lib=el(root,'library_geometries'); g=el(lib,'geometry',id='geometry',name='geometry'); m=el(g,'mesh')
        source=el(m,'source',id='positions')
        el(source,'float_array',' '.join(f'{n:.9f}' for p in self.v for n in p),id='positions-array',count=str(len(self.v)*3))
        ac=el(el(source,'technique_common'),'accessor',source='#positions-array',count=str(len(self.v)),stride='3')
        for name in 'XYZ': el(ac,'param',name=name,type='float')
        vertices=el(m,'vertices',id='vertices'); el(vertices,'input',semantic='POSITION',source='#positions')
        triangles=el(m,'triangles',count=str(len(self.t))); el(triangles,'input',semantic='VERTEX',source='#vertices',offset='0')
        el(triangles,'p',' '.join(str(n) for t in self.t for n in t))
        vs=el(el(root,'library_visual_scenes'),'visual_scene',id='Scene',name='Scene')
        el(el(vs,'node',id='track',name='track',type='NODE'),'instance_geometry',url='#geometry')
        el(el(root,'scene'),'instance_visual_scene',url='#Scene')
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


def capsule(radius):
    points=[]
    for cx,start in [(H,-math.pi/2),(-H,math.pi/2)]:
        for i in range(SEGMENTS+1):
            a=start+math.pi*i/SEGMENTS
            points.append((cx+radius*math.cos(a),radius*math.sin(a)))
    return points


def tag(parent,element_name,text=None,**attrs):
    e=ET.SubElement(parent,element_name,attrs)
    if text is not None: e.text=str(text)
    return e


def box_geom(parent,size):
    tag(tag(tag(parent,'geometry'),'box'),'size',size)


def mesh_geom(parent,uri):
    tag(tag(tag(parent,'geometry'),'mesh'),'uri',uri)


def material(visual,color):
    m=tag(visual,'material'); rgba=' '.join(map(str,color))
    tag(m,'ambient',rgba); tag(m,'diffuse',rgba); tag(m,'specular','0.05 0.05 0.05 1')


def write_xml(path,root):
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


def make_model(color,rgba):
    name=f'wa_400m_track_{color}'
    dest=ROOT/'simulation/models'/name
    (dest/'meshes').mkdir(parents=True,exist_ok=True)
    planar=Planar()
    inner=planar.polygon(capsule(R))
    outer=planar.polygon(capsule(OUTER))
    footprint=planar.union(outer,planar.rect(-70.805,59.195,-OUTER,-R))
    running=planar.difference(footprint,inner)
    grass=planar.polygon(capsule(R-LINE))
    curb_footprint=planar.difference(inner,grass)
    white=None
    for lane in range(1,N+1):
        r=R+lane*W
        stripe=planar.difference(planar.polygon(capsule(r)),planar.polygon(capsule(r-LINE)))
        stripe=planar.union(stripe,planar.rect(-70.805,59.195,-r,-r+LINE))
        white=stripe if white is None else planar.union(white,stripe)
    white=planar.union(white,planar.rect(-70.805,59.195,-R-LINE,-R))
    for x in [H,-57.805,-67.805]:
        white=planar.union(white,planar.rect(x-LINE,x,-OUTER,-R))
    white=planar.intersection(white,running)
    colored=planar.difference(running,white)
    frame=planar.rect(-94,94,-52,52)
    surround=planar.difference(frame,footprint)
    regions={'track':colored,'infield':grass,'markings':white,'surround':surround}
    # All top surfaces tile Z=0; no underlying visible box top or decal layers.
    for part,geometry in regions.items():
        print('Triangulating',color,part,flush=True)
        mesh=Mesh()
        # Local tiles avoid long/slender global triangulation and bound triangulation memory.
        for x0 in range(-94,94,16):
            for y0 in range(-52,52,16):
                tile=planar.rect(x0,min(x0+16,94),y0,min(y0+16,52))
                clipped=planar.intersection(geometry,tile)
                cell_area=0.0
                for triangle in planar.triangles(clipped):
                    k=len(mesh.v); mesh.v.extend((x,y,0.0) for x,y in triangle)
                    mesh.t.append((k,k+1,k+2))
                    a,b,c=triangle
                    cell_area+=((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
                if abs(cell_area-planar.area(clipped))>1e-5:
                    raise RuntimeError(f'Triangulation lost area: {part} tile {x0},{y0}: {cell_area} vs {planar.area(clipped)}')
        mesh.save(dest/'meshes'/f'{part}.dae')
    region_list=list(regions.values())+[curb_footprint]
    max_overlap=max(planar.area(planar.intersection(a,b))
                    for i,a in enumerate(region_list) for b in region_list[i+1:])
    missing=planar.area(frame)-sum(planar.area(g) for g in region_list)
    if max_overlap>1e-7 or abs(missing)>1e-6:
        raise RuntimeError(f'Invalid ground partition: overlap={max_overlap}, missing={missing}')
    audit={'max_pairwise_overlap_m2':max_overlap,'coverage_error_m2':missing,
           'region_areas_m2':{k:planar.area(g) for k,g in regions.items()},
           'kerb_footprint_area_m2':planar.area(curb_footprint),
           'visual_surface_z_m':0.0,'overlay_layers':0,'depth_bias':0}
    (dest/'geometry_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    planar.close()
    obsolete=dest/'materials/scripts/track.material'
    if obsolete.exists(): obsolete.unlink()
    kerb=Mesh(); kerb.solid_ring(R-LINE,R,0.05); kerb.save(dest/'meshes/kerb.dae')
    sdf=ET.Element('sdf',{'version':'1.6'}); model=tag(sdf,'model',name=name); tag(model,'static','true')
    link=tag(model,'link',name='track')
    collision=tag(link,'collision',name='flat_drivable_surface'); tag(collision,'pose','0 0 -0.1 0 0 0'); box_geom(collision,'188 104 0.2')
    ode=tag(tag(tag(collision,'surface'),'friction'),'ode'); tag(ode,'mu',0.9); tag(ode,'mu2',0.9)
    for part,mat in [('surround',(0.37,0.39,0.36,1)),('track',rgba),('infield',(0.15,0.32,0.12,1)),('markings',(1,1,1,1)),('kerb',(0.9,0.9,0.9,1))]:
        visual=tag(link,'visual',name=part); mesh_geom(visual,f'model://{name}/meshes/{part}.dae'); material(visual,mat)
        if part!='kerb': tag(visual,'cast_shadows','false')
    kc=tag(link,'collision',name='inner_kerb'); mesh_geom(kc,f'model://{name}/meshes/kerb.dae')
    write_xml(dest/'model.sdf',sdf)
    config=ET.Element('model'); tag(config,'name',name); tag(config,'version','1.1'); tag(config,'sdf','model.sdf',version='1.6')
    author=tag(config,'author'); tag(author,'name','remote5g_ws'); tag(config,'description',f'Full-scale 8-lane WA 400m standard oval, {color}; 130m sprint apron; robot simulation approximation.')
    write_xml(dest/'model.config',config)
    worldroot=ET.Element('sdf',{'version':'1.6'}); world=tag(worldroot,'world',name=f'remote5g_track_{color}')
    tag(world,'gravity','0 0 -9.81')
    physics=tag(world,'physics',name='ode',type='ode'); tag(physics,'max_step_size','0.001'); tag(physics,'real_time_update_rate','1000')
    scene=tag(world,'scene'); tag(scene,'ambient','0.6 0.6 0.6 1'); tag(scene,'background','0.65 0.80 0.95 1'); tag(scene,'shadows','true')
    light=tag(world,'light',name='sun',type='directional'); tag(light,'cast_shadows','true'); tag(light,'diffuse','0.8 0.8 0.8 1'); tag(light,'specular','0.1 0.1 0.1 1'); tag(light,'direction','-0.3 0.2 -1')
    include=tag(world,'include'); tag(include,'uri',f'model://{name}')
    camera=tag(tag(world,'gui'),'camera',name='overview'); tag(camera,'pose','0 -105 105 0 0.7853981634 1.5707963268'); tag(camera,'view_controller','orbit')
    write_xml(ROOT/f'simulation/worlds/athletics_400m_{color}.world',worldroot)


def path_at(s,radius):
    bend=math.pi*radius
    if s<L: return -H+s,-radius,0.0
    s-=L
    if s<bend:
        a=-math.pi/2+s/radius
        return H+radius*math.cos(a),radius*math.sin(a),a+math.pi/2
    s-=bend
    if s<L: return H-s,radius,math.pi
    s-=L; a=math.pi/2+s/radius
    return -H+radius*math.cos(a),radius*math.sin(a),a+math.pi/2


def main():
    for color,rgba in COLORS.items(): make_model(color,rgba)
    paths=ROOT/'simulation/maps'; paths.mkdir(parents=True,exist_ok=True)
    for lane in range(1,N+1):
        radius=R+(lane-0.5)*W
        length=2*L+2*math.pi*radius
        count=math.ceil(length/0.25)
        with (paths/f'lane_{lane}_centerline.csv').open('w') as f:
            writer=csv.writer(f); writer.writerow(['s_m','x_m','y_m','z_m','yaw_rad'])
            for i in range(count+1):
                s=length*i/count; x,y,yaw=path_at(s,radius)
                writer.writerow([f'{s:.9f}',f'{x:.9f}',f'{y:.9f}','0',f'{math.atan2(math.sin(yaw),math.cos(yaw)):.9f}'])
    metadata={'model_revision':'1.1','units':'metres','sdf_version':'1.6','inner_radius_m':R,'straight_length_m':L,'lanes':N,'lane_width_m':W,'line_width_m':LINE,'outer_radius_m':OUTER,'oval_bounds_m':[2*(H+OUTER),2*OUTER],'lane_1_measurement_offset_m':0.30,'lane_1_measurement_length_m':2*L+2*math.pi*(R+0.30),'lane_1_centerline_length_m':2*L+2*math.pi*(R+W/2),'arc_segments_per_semicircle':SEGMENTS,'max_outer_arc_chord_error_m':OUTER*(1-math.cos(math.pi/(2*SEGMENTS))),'kerb_width_m':LINE,'kerb_height_m':0.05,'surface_collision_z_m':0,'visual_surface_z_m':0,'visual_partition':'non-overlapping triangulated regions','depth_bias':0,'friction_mu':0.9,'sprint_apron_x_m':[-70.805,59.195],'colors_rgba':COLORS,'source':{'file':'World-Athletics-Track-and-Field-Facilities-Manual-2019-Edition.pdf','section':'2.2.1.1','printed_pages':[29,30,31],'pdf_pages':[27,28,29]},'scope':'Standard oval and sprint apron only. No full athletics event markings or robot competition props. Colors and friction are simulation choices.'}
    (ROOT/'config/athletics_track.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('Generated red/blue SDF models, worlds, lane paths and dimensions.')

if __name__=='__main__': main()
