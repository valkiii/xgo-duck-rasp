import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from design import *
from shapely.geometry import Polygon, box as sbox2, Point
from shapely.ops import unary_union
import trimesh
def extrude(poly,z0,z1):
    m=trimesh.creation.extrude_polygon(poly,height=z1-z0); m.apply_translation([0,0,z0]); return m
def union(ms):
    return trimesh.boolean.union(ms,engine='manifold')
def diff(a,bs):
    return trimesh.boolean.difference([a]+bs,engine='manifold')
def cyl(x,y,z0,z1,r,sections=48):
    m=trimesh.creation.cylinder(radius=r,height=z1-z0,sections=sections); m.apply_translation([x,y,(z0+z1)/2]); return m
# ------------- tray ----------------
TRAY_Z0,TRAY_Z1=238.0,241.0          # beam bottom/top
PCB_BOT=ZU+PCB_B                     # 245.0
POST_H=PCB_BOT-TRAY_Z1               # 4.0
TAB_Z=220.3                          # top of support tabs (existing M2 holes)
PILLARS=[(21.0,25.0),(67.1,0.0)]      # existing support-plate holes (M2) re-used as tray anchors
POSTS=[tuple(S2W([X,Y,0])[0][:2]) for X,Y in HOLES]
def build_tray():
    rear=sbox2(15.8,-10.5,24.0,28.5)
    front=sbox2(65.0,-10.5,81.0,19.8)
    parts=[extrude(rear,TRAY_Z0,TRAY_Z1),extrude(front,TRAY_Z0,TRAY_Z1)]
    for (x,y) in POSTS: parts.append(cyl(x,y,TRAY_Z1-0.5,PCB_BOT,2.75))
    tray=union(parts)
    cuts=[]
    for (x,y) in PILLARS: cuts.append(cyl(x,y,TRAY_Z0-1,TRAY_Z1+1,1.25))
    for (x,y) in PILLARS: cuts.append(cyl(x,y,TRAY_Z1-1.5,TRAY_Z1+1,2.1))
    for (x,y) in POSTS: cuts.append(cyl(x,y,TRAY_Z1-0.5,PCB_BOT+0.01,1.1))
    return diff(tray,cuts)
# ------------- camera ---------------
LENS=(0.0,237.1)            # y,z of eye axis
CAM_FRONT=89.3              # membrane rear face = PCB lens-side face
CAM_T=1.0
CAM_BACK=CAM_FRONT-CAM_T    # 88.3
BOSS_L=1.2
PLATE_T=1.6
PLATE_FRONT=CAM_BACK-BOSS_L # 85.0
PLATE_BACK=PLATE_FRONT-PLATE_T  # 83.4
# camera board upside-down (connector on the top edge): board z 227.6..251.6
CAM_Z0=LENS[1]-9.5; CAM_Z1=CAM_Z0+24.0
CAM_HOLES=[(sy*10.5,zz) for sy in (-1,1) for zz in (CAM_Z0+2.0, CAM_Z0+2.0+12.5)]
COLUMNS=[(-16.5,LENS[1]),(16.5,LENS[1])]   # shell columns for the plate ears
WALL_REAR=92.8
def xbox(x0,x1,y0,y1,z0,z1):
    m=trimesh.creation.box(extents=[x1-x0,y1-y0,z1-z0]); m.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]); return m
def xcyl(y,z,x0,x1,r,sections=40):
    m=trimesh.creation.cylinder(radius=r,height=x1-x0,sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0])); m.apply_translation([(x0+x1)/2,y,z]); return m
def build_camera_mount():
    plate=xbox(PLATE_BACK,PLATE_FRONT,-19.5,19.5,227.0,247.0)
    parts=[plate]+[xcyl(y,z,PLATE_FRONT-0.3,CAM_BACK,2.3) for (y,z) in CAM_HOLES]
    m=union(parts)
    cuts=[xcyl(y,z,PLATE_BACK-1,CAM_BACK+0.01,0.85) for (y,z) in CAM_HOLES]        # Ø1.7 M2 self-tap pilots (through the plate down into the bosses, blind 6.5)
    # pilots are blind: from the boss tip down 6.0 mm
    cuts=[xcyl(y,z,CAM_BACK-6.0,CAM_BACK+0.01,0.85) for (y,z) in CAM_HOLES]
    cuts+= [xcyl(y,LENS[1],PLATE_BACK-1,PLATE_FRONT+1,1.25) for y in (-16.5,16.5)]  # M2 clearance for the ear screws (shell columns)
    # ribbon notch is not needed: ribbon leaves above the plate (board top edge z 251.6 > plate top 247)
    return diff(m,cuts)
def camera_dummy():
    pcb=xbox(CAM_BACK,CAM_FRONT,-12.5,12.5,CAM_Z0,CAM_Z1)
    lens=xbox(CAM_FRONT,CAM_FRONT+5.5,-4.25,4.25,LENS[1]-4.25,LENS[1]+4.25)
    conn=xbox(CAM_BACK-2.5,CAM_BACK,-8.0,8.0,CAM_Z1-3.0,CAM_Z1)
    return union([pcb,lens,conn])
def pi_dummy():
    parts=[xbox(0,0,0,0,0,0)] if False else []
    ms=[]
    def sb(b): 
        return sbox(*b)
    ms.append(sb((0,65,0,30,PCB_B,PCB_T)))
    ms.append(sb((0,65,0,30,PCB_T,TOP)))
    ms.append(sb((0,65,0,30,0,UB)))
    ms.append(sb(HDR)); 
    return ms
if __name__=='__main__':
    tray=build_tray(); print('tray watertight',tray.is_watertight,'vol cm3',tray.volume/1000,'bounds',tray.bounds.round(1).tolist())
    tray.export(OUT2+'tray_world.stl')
    cm=build_camera_mount(); print('camera mount watertight',cm.is_watertight,cm.volume/1000,cm.bounds.round(1).tolist())
    cm.export(OUT2+'camera_mount_world.stl')
    cd=camera_dummy(); cd.export(OUT2+'camera_dummy_world.stl')

def build_spacer():
    h=TRAY_Z0-TAB_Z
    return diff(cyl(0,0,0,h,3.1),[cyl(0,0,-1,h+1,1.3)])
