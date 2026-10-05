#!/usr/bin/env python3
"""Generate a synthetic teaching building from metric objects, never image pixels.

One manifest owns the geometry used by BOTH OccupancyGrid and Gazebo SDF.
All architectural dimensions are design assumptions for training, not a survey.
"""
import argparse
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


PARAMS = {
    'building_width': 48.0, 'building_depth': 40.0,
    'wall_thickness': 0.16, 'wall_height': 2.6,
    'door_width': 1.20, 'main_corridor_width': 3.0,
    'resolution': 0.05, 'margin': 0.5,
    'footprint_padding': 0.06,
}
FOOTPRINT = [[.35,.20],[.20,.35],[-.20,.35],[-.35,.20],
             [-.35,-.20],[-.20,-.35],[.20,-.35],[.35,-.20]]


def polygon(obj):
    c, s = math.cos(obj['yaw']), math.sin(obj['yaw'])
    return [(obj['x'] + dx*c - dy*s, obj['y'] + dx*s + dy*c)
            for dx,dy in [(-obj['sx']/2,-obj['sy']/2),(obj['sx']/2,-obj['sy']/2),
                          (obj['sx']/2,obj['sy']/2),(-obj['sx']/2,obj['sy']/2)]]


def scene(variant):
    obstacles, rooms, doors = [], [], []
    def box(name, kind, x, y, sx, sy, sz=2.6, yaw=0.0):
        assert min(sx,sy,sz) > 0
        obstacles.append(dict(id=name,kind=kind,x=x,y=y,z=sz/2,
                              sx=sx,sy=sy,sz=sz,yaw=yaw))

    def wall(name, a, b, gaps=()):
        """Split a wall at explicit openings (distance from endpoint A, width)."""
        dx,dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx,dy)
        ux,uy = dx/length,dy/length
        cursor = 0.0
        ranges = []
        for d,width in sorted(gaps):
            start,end = d-width/2,d+width/2
            assert 0 <= start < end <= length, (name,d,width,length)
            assert start >= cursor, name
            if start > cursor: ranges.append((cursor,start))
            cursor=end
            doors.append(dict(id=name+'_door_'+str(len(doors)),
                              center=[a[0]+ux*d,a[1]+uy*d],width=width,
                              yaw=math.atan2(dy,dx)))
        if cursor < length: ranges.append((cursor,length))
        for i,(start,end) in enumerate(ranges):
            # End-point semantics are wall centreline. Door net width is the
            # exact gap between split box end faces.
            m=(start+end)/2
            box(name+'_'+str(i),'wall',a[0]+ux*m,a[1]+uy*m,
                end-start,PARAMS['wall_thickness'],yaw=math.atan2(dy,dx))

    def room(name, bounds, label=None, locked=False):
        rooms.append(dict(id=name,bounds=bounds,label=label or name,locked=locked))

    wall('outer_south',(-24,-20),(24,-20))
    wall('outer_north',(-24,20),(24,20))
    wall('outer_west',(-24,-20),(-24,20))
    wall('outer_east',(24,-20),(24,20))

    # Four repeated classrooms on each side of a three-metre spine.
    ys=[-12,-4,4,12,20]
    centers=[-8,0,8,16]
    wall('west_spine',(-18.2,-12),(-18.2,20),[(y+12,1.2) for y in centers])
    wall('east_spine',(-15.2,-12),(-15.2,20),[(y+12,1.2) for y in centers])
    wall('classroom_hall_boundary',(-10,-12),(-10,20),[(4,2.4),(22,2.4)])
    for i,y in enumerate(ys[:-1]):
        wall('west_partition_'+str(i),(-24,y),(-18.2,y))
        wall('east_partition_'+str(i),(-15.2,y),(-10,y),
             [(2.6,1.2)] if i == 0 else [])
        room('W'+str(101+i),[-24,y,-18.2,ys[i+1]])
        room('W'+str(105+i),[-15.2,y,-10,ys[i+1]])
        # Small desk banks leave clear entry/turning space.
        for side in ('w','e'):
            x=-22.0 if side=='w' else -12.3
            box('desk_'+side+str(i),'desk',x,y+2.0,2.0,0.8,.75)

    # Two lobby connections around the classroom boundary close a large loop.
    wall('north_services',(-10,12),(8,12),[(3,1.2),(9,1.2),(15,1.2)])
    for i,x in enumerate([-10,-4,2,8]):
        wall('north_divider_'+str(i),(x,12),(x,20))
    for i,(x1,x2) in enumerate([(-10,-4),(-4,2),(2,8)]):
        room('N'+str(201+i),[x1,12,x2,20],['Office','Computer lab','Seminar'][i])
        box('north_desk_'+str(i),'desk',(x1+x2)/2,18,2.6,.8,.75)

    # Southern seminar rooms, main lobby and closed stairs/elevator cores.
    wall('south_seminars',(-10,-12),(8,-12),[(3,1.2),(9,1.2),(15,1.2)])
    for i,x in enumerate([-10,-4,2,8]):
        wall('south_divider_'+str(i),(x,-20),(x,-12))
    for i,(x1,x2) in enumerate([(-10,-4),(-4,2),(2,8)]):
        room('S'+str(301+i),[x1,-20,x2,-12],['Seminar','Lab','Project room'][i])
        box('seminar_table_'+str(i),'desk',(x1+x2)/2,-18,2.0,.9,.75)
    wall('stair_boundary',(-18.2,-20),(-18.2,-12))
    wall('stair_elevator_divider',(-20,-20),(-20,-12))
    room('STAIR',[-24,-20,-20,-12],'Stairs / closed',True)
    room('LIFT',[-20,-20,-18.2,-12],'Lift / closed',True)
    room('SERVICE',[-15.2,-20,-10,-12],'Service room')

    # Chamfered auditorium: the ring around it provides multiple routes.
    auditorium=[(8,5),(11,2),(20,2),(20,16),(11,16),(8,13)]
    gaps={1:[(6,1.8)],3:[(3,1.8)],5:[(4,1.8)]}
    for i,a in enumerate(auditorium):
        wall('auditorium_'+str(i),a,auditorium[(i+1)%len(auditorium)],gaps.get(i,()))
    room('AUD',[8,2,20,16],'Auditorium')
    for row,y in enumerate([4.7,7.5,10.3,13.1]):
        for col,x in enumerate([13.1,17.1]):
            box('aud_seats_'+str(row)+'_'+str(col),'seating',x,y,2.5,1.0,.95)
    box('aud_stage','stage',9.7,9,1.1,4.0,.45)

    # Smaller lecture theatre and surrounding corridor/lobby.
    wall('lecture_north',(8,-2),(19,-2),[(5.5,1.8)])
    wall('lecture_south',(8,-12),(19,-12),[(5.5,1.8)])
    wall('lecture_west',(8,-12),(8,-2),[(5,1.2)])
    wall('lecture_east',(19,-12),(19,-2))
    room('LEC',[8,-12,19,-2],'Lecture hall')
    for row,y in enumerate([-9.8,-7.4,-5.0]):
        for col,x in enumerate([10.8,16.2]):
            box('lecture_seats_'+str(row)+'_'+str(col),'seating',x,y,2.5,.9,.95)

    # Pillars aid localization in otherwise large open areas.
    for i,(x,y) in enumerate([(-6,-6),(-6,3),(0,-6),(0,3),(5,8),(22,7),(22,13)]):
        box('pillar_'+str(i),'pillar',x,y,.5,.5)
    box('reception','desk',21,-16,3,1.0,.9)
    box('lobby_bench','bench',19,-18.5,2,.5,.55)
    box('hall_bench','bench',-8.5,0,.6,2.4,.55)

    # Fixed, deterministic variants. These obstacles are included in the map
    # so each variant can be assessed without hidden-map assumptions.
    seed=None
    if variant!='baseline':
        seed=int(variant.split('_')[-1])
        rng=random.Random(seed)
        for i,(x,y) in enumerate([(-2,-1),(3,-9),(22,-5),(-16.7,5)]):
            box('cart_'+str(i),'cart',x+rng.uniform(-.25,.25),
                y+rng.uniform(-.3,.3),.8,.6,.85,yaw=rng.choice([0,math.pi/6]))
        if seed>=100:
            # Partial corridor restriction, still leaving >=1.6 m bypass.
            box('heldout_partition','cart',22.8,0,1.3,1.0,1.0)

    def task(name,start,goal,expected='reachable',ability=''):
        return dict(id=name,start=start,goal=goal,expected=expected,ability=ability)
    spawn=[13.5,-16.0,0.0]
    # Continuous smoke route: enter lecture hall, cross a 1.20 m side door,
    # return through the hallway and a 1.20 m project-room doorway.
    smoke=[task('enter_lecture',spawn,[13.5,-7,0],ability='seat aisle and 1.8m entry'),
           task('exit_120cm_door',[13.5,-7,0],[5.5,-7,0],ability='1.2m side doorway'),
           task('enter_project_room',[5.5,-7,0],[5,-15,0],ability='turn and 1.2m classroom doorway')]
    tasks=[
        task('T01_narrow_door',[5,-10,0],[5,-15,0],ability='1.20m doorway'),
        task('T02_classroom_turn',[-12.5,-8,0],[-16.7,-2,math.pi/2],ability='classroom exit and right-angle turns'),
        task('T03_repeated_rooms',[-16.7,-8,math.pi/2],[-21,16,math.pi],ability='repeated classroom localization'),
        task('T04_pillar_hall',[-3,-9,0],[4,6,math.pi/2],ability='pillar avoidance'),
        task('T05_ring_north',[5,9,0],[22,11,math.pi/2],ability='auditorium ring and alternate routes'),
        task('T06_auditorium',[17,18,0],[15.1,10.5,-math.pi/2],ability='seating aisle and auditorium entry'),
        task('T07_deadend_office',[-7,16,0],[-3,9,-math.pi/2],ability='office exit'),
        task('T08_long_return',[22,11,-math.pi/2],[13.5,-16,0],ability='long multi-turn circulation'),
        task('T09_lobby_loop',spawn,[-16.7,-16,math.pi],ability='south classroom block detour'),
        task('T10_unreachable',spawn,[-22,-16,0],'unreachable','closed stairs rejection'),
    ]
    return dict(scene_id='teaching_training_v1',variant=variant,seed=seed,
                bounds=[-24,-20,24,20],parameters=PARAMS,spawn=spawn,
                obstacles=obstacles,rooms=rooms,doors=doors,footprint=FOOTPRINT,
                smoke_tasks=smoke,tasks=tasks,
                assumptions=['All building dimensions inferred for a synthetic training environment.',
                             'Reference photo influences functional layout only; no pixel thresholding.',
                             'Seats grouped into blocks; stairs/elevators are closed single-floor zones.',
                             'Existing ideal_kinematic_sim is a navigation software abstraction; contacts are not simulated.'])


def sub(parent,tag,text=None,**attrs):
    e=ET.SubElement(parent,tag,attrs)
    if text is not None: e.text=str(text)
    return e


def write_sdf(manifest,path):
    root=ET.Element('sdf',version='1.9')
    world=sub(root,'world',name=manifest['scene_id']+'_'+manifest['variant'])
    physics=sub(world,'physics',name='default',type='ignored')
    sub(physics,'max_step_size','0.01');sub(physics,'real_time_factor','1.0')
    sub(world,'gravity','0 0 -9.81')
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),
                          ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        plugin=sub(world,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
        if name=='Sensors':sub(plugin,'render_engine','ogre2')
    scene_node=sub(world,'scene');sub(scene_node,'ambient','.7 .7 .7 1');sub(scene_node,'background','.85 .88 .9 1')
    light=sub(world,'light',name='sun',type='directional');sub(light,'pose','0 0 15 0 0 0')
    sub(light,'diffuse','.8 .8 .8 1');sub(light,'direction','-.3 -.2 -1')
    # One static model/link owns all pieces. Each object has exact pose, shape,
    # collision and visual, shared with the occupancy map's polygon footprint.
    model=sub(world,'model',name='teaching_building');sub(model,'static','true');link=sub(model,'link',name='structure')
    objects=[dict(id='floor',kind='floor',x=0,y=0,z=-.05,sx=48.0,sy=40.0,sz=.1,yaw=0)]+manifest['obstacles']
    colors={'wall':'.7 .74 .79 1','seating':'.18 .32 .5 1','desk':'.62 .39 .2 1',
            'stage':'.48 .28 .18 1','pillar':'.4 .43 .46 1','bench':'.21 .46 .36 1',
            'cart':'.75 .22 .13 1','floor':'.84 .83 .79 1'}
    for o in objects:
        for kind in ['collision','visual']:
            child=sub(link,kind,name=o['id']+'_'+kind)
            sub(child,'pose',f"{o['x']} {o['y']} {o['z']} 0 0 {o['yaw']}")
            box_node=sub(sub(child,'geometry'),'box');sub(box_node,'size',f"{o['sx']} {o['sy']} {o['sz']}")
            if kind=='visual':
                material=sub(child,'material');sub(material,'ambient',colors[o['kind']]);sub(material,'diffuse',colors[o['kind']])
    ET.indent(root,space='  ')
    path.write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8',newline='\n')


def write_map(manifest,folder):
    r=PARAMS['resolution'];margin=PARAMS['margin']
    minx,miny,maxx,maxy=manifest['bounds'];minx-=margin;miny-=margin;maxx+=margin;maxy+=margin
    width=round((maxx-minx)/r);height=round((maxy-miny)/r)
    image=Image.new('L',(width,height),0);draw=ImageDraw.Draw(image)
    def pix(x,y):return ((x-minx)/r,(maxy-y)/r)
    # Conservatively cover metric box outlines on a 0.05 m raster.
    draw.rectangle([pix(-24,20),pix(24,-20)],fill=255)
    for o in manifest['obstacles']:draw.polygon([pix(x,y) for x,y in polygon(o)],fill=0)
    image.save(folder/'teaching.png')
    array=np.array(image)
    (folder/'teaching.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+array.tobytes())
    (folder/'teaching.yaml').write_text(
        f'image: teaching.pgm\nmode: trinary\nresolution: {r}\norigin: [{minx}, {miny}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',encoding='utf-8',newline='\n')
    manifest['map']=dict(resolution=r,origin=[minx,miny,0],width=width,height=height)

    # Clean labelled plan drawn solely from semantic geometry.
    scale=30.0
    def sp(x,y):return ((x-minx)*scale,(maxy-y)*scale)
    preview=Image.new('RGB',(round((maxx-minx)*scale),round((maxy-miny)*scale)), '#edf1f5')
    d=ImageDraw.Draw(preview)
    d.rectangle([sp(-24,20),sp(24,-20)],fill='#ffffff')
    room_colors=['#eef5ff','#f2f8f4','#fff6e6']
    for i,room in enumerate(manifest['rooms']):
        x1,y1,x2,y2=room['bounds']
        d.rectangle([sp(x1,y2),sp(x2,y1)],fill='#e4e7eb' if room['locked'] else room_colors[i%3])
    colors={'wall':'#243447','seating':'#3f6e9c','desk':'#b17b48','stage':'#93654b',
            'pillar':'#44566a','bench':'#438468','cart':'#d7503a'}
    for o in manifest['obstacles']:d.polygon([sp(x,y) for x,y in polygon(o)],fill=colors[o['kind']])
    font=None
    for filename in ['C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(filename).exists():font=ImageFont.truetype(filename,17);break
    font=font or ImageFont.load_default()
    for room in manifest['rooms']:
        x1,y1,x2,y2=room['bounds'];px,py=sp((x1+x2)/2,(y1+y2)/2)
        label=room['id']+'\n'+room['label'] if room['id']!=room['label'] else room['id']
        d.multiline_text((px,py),label,fill='#20354a',font=font,anchor='mm',align='center')
    for x,y,text in [(-16.7,10,'3 m corridor'),(-2,6,'Pillar hall'),(22,9,'Ring'),(13.5,-16,'Main lobby')]:
        d.text(sp(x,y),text,font=font,fill='#456179',anchor='mm')
    for door in manifest['doors']:
        x,y=door['center'];px,py=sp(x,y)
        d.ellipse((px-3,py-3,px+3,py+3),fill='#2f946b')
    x,y,_=manifest['spawn'];fp=[sp(x+vx,y+vy) for vx,vy in FOOTPRINT]
    d.polygon(fp,fill='#ef882f',outline='#a74b00')
    d.text(sp(14.8,-16.8),'QianLi start',font=font,fill='#ba5800')
    d.text((26,16),'QianLi Teaching Training v1 | 48 x 40 m | synthetic metric model',font=font,fill='#20354a')
    # Scale bar and axis indicator.
    a,b=sp(-23,-19),sp(-18,-19);d.line([a,b],fill='#324b60',width=3)
    d.text((a[0],a[1]-24),'5 m',font=font,fill='#324b60')
    d.text(sp(21,19),'Y / north up',font=font,fill='#324b60',anchor='mm')
    preview.save(folder/'layout.png')


def validate(manifest,folder):
    import cv2
    def overlap(a,b):
        for p in (a,b):
            for u,v in zip(p,p[1:]+p[:1]):
                nx,ny=v[1]-u[1],u[0]-v[0]
                pa=[x*nx+y*ny for x,y in a]
                pb=[x*nx+y*ny for x,y in b]
                if min(max(pa),max(pb))-max(min(pa),min(pb)) <= 1e-8:
                    return False
        return True
    door_checks=[]
    for door in manifest['doors']:
        opening=polygon(dict(x=door['center'][0],y=door['center'][1],
                             sx=door['width']-.002,sy=PARAMS['wall_thickness']-.002,
                             yaw=door['yaw']))
        blocked=[o['id'] for o in manifest['obstacles'] if overlap(opening,polygon(o))]
        assert not blocked, (door['id'],blocked)
        door_checks.append(dict(id=door['id'],net_width=door['width'],passed=True))
    grid=np.array(Image.open(folder/'teaching.png'))
    r=manifest['map']['resolution'];minx,miny,_=manifest['map']['origin']
    # Rotation-safe conservative padded body radius ~0.463m, plus raster error.
    radius=math.hypot(.35,.2)+PARAMS['footprint_padding']/math.cos(math.pi/8)+r
    distance=cv2.distanceTransform((grid==255).astype(np.uint8),cv2.DIST_L2,5)*r
    safe=(distance>=radius).astype(np.uint8)
    count,labels=cv2.connectedComponents(safe,8)
    def component(p):
        col=math.floor((p[0]-minx)/r);row=grid.shape[0]-1-math.floor((p[1]-miny)/r)
        return int(labels[row,col]),float(distance[row,col])
    checks=[]
    for t in manifest['tasks']+manifest['smoke_tasks']:
        a,da=component(t['start']);b,db=component(t['goal'])
        valid=(a!=0 and a==b) if t['expected']=='reachable' else (a!=0 and a!=b)
        checks.append(dict(id=t['id'],passed=valid,start_component=a,goal_component=b,
                           start_clearance=round(da,3),goal_clearance=round(db,3)))
    sdf=ET.parse(folder/'teaching.sdf')
    coll=sdf.findall('.//collision')
    assert len(coll)==len(manifest['obstacles'])+1
    for o in manifest['obstacles']:
        el=sdf.find(".//collision[@name='"+o['id']+"_collision']")
        assert el is not None
        assert [float(v) for v in el.find('geometry/box/size').text.split()]==[o['sx'],o['sy'],o['sz']]
        assert [float(v) for v in el.find('pose').text.split()]==[o['x'],o['y'],o['z'],0,0,o['yaw']]
    report=dict(result='PASS' if all(x['passed'] for x in checks) else 'FAIL',
                variant=manifest['variant'],geometry_objects=len(manifest['obstacles']),
                sdf_collision_count=len(coll),safe_rotation_radius=round(radius,4),
                map_sdf_common_source=True,door_checks=door_checks,connectivity_checks=checks)
    (folder/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8',newline='\n')
    assert report['result']=='PASS',report
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'generated')
    parser.add_argument('--variants',nargs='+',default=['baseline','train_000','train_001','test_101'])
    args=parser.parse_args()
    for variant in args.variants:
        folder=args.output/variant;folder.mkdir(parents=True,exist_ok=True)
        manifest=scene(variant);write_sdf(manifest,folder/'teaching.sdf');write_map(manifest,folder)
        (folder/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8',newline='\n')
        report=validate(manifest,folder)
        print(variant,report['result'],report['geometry_objects'],'objects')


if __name__=='__main__': main()
