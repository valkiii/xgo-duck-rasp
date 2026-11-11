import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
from viz import *
import json
OUT2='/Users/vc/Research/xgo-duck/cad/head_v2/'
REG=json.load(open('/Users/vc/Research/xgo-duck/cad/pi_placement/registration_init.json'))
def T_of(name): return np.array(REG[name]['T'])
def load_part_world(name):
    me=trimesh.load(STRUCT+name); me.apply_transform(T_of(name)); return me
def head_servos_world():
    m,d=load(); set_pose(m,d,'INIT'); wm=world_meshes(m,d)
    out={}
    for b in ['jaw_soft','yaw_roll_motion']:
        for n,me in wm[b]:
            for p in me.split(only_watertight=False):
                if p.is_volume and 15.5<p.volume/1000<16.5 and abs(p.extents.max()-34)<1:
                    out[f'{b}_{p.bounds[0][0]:.0f}']=p
    return out
def rotx_about(angle,y0,z0):
    c,s=np.cos(angle),np.sin(angle); T=np.eye(4); T[1,1]=c;T[1,2]=-s;T[2,1]=s;T[2,2]=c
    T[1,3]=y0-(c*y0-s*z0); T[2,3]=z0-(s*y0+c*z0); return T
