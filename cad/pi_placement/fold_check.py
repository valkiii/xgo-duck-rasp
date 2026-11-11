import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
from scipy.spatial import cKDTree
m,d=load()
def body_T(pose,b,q=None):
    set_pose(m,d,pose,q); T=np.eye(4); bid=m.body(b).id; T[:3,:3]=d.xmat[bid].reshape(3,3); T[:3,3]=d.xpos[bid]*1000; return T
Ti=body_T('INIT','jaw_soft')
new=trimesh.load(OUT+'head_shell_world_pi.stl'); stack=trimesh.load(OUT+'stack_envelope_head_world.stl')
old=trimesh.load(STRUCT+'head_shell.stl'); res=json.load(open(OUT+'registration_init.json')) if False else None
import json
res=json.load(open(OUT+'registration_init.json')); old.apply_transform(np.array(res['head_shell.stl']['T']))
pn=np.vstack([new.sample(30000),stack.sample(20000)]); po=old.sample(30000)
def others(pose):
    set_pose(m,d,pose); wm=world_meshes(m,d); pts=[]
    for b,l in wm.items():
        if b in('jaw_soft','yaw_roll_motion','neck_pitch'): continue   # head group
        for n,me in l: pts.append(me.sample(20000))
    return np.vstack(pts)
for pose in ['STAND','SIT','FOLD','INIT']:
    Tp=body_T(pose,'jaw_soft'); H=Tp@np.linalg.inv(Ti)
    O=cKDTree(others(pose))
    def tf(p): return (H[:3,:3]@p.T).T+H[:3,3]
    dn=O.query(tf(pn))[0].min(); do=O.query(tf(po))[0].min()
    print(pose,'min gap head(with hump+stack) to torso/legs/neck-servo: %.1f mm ; original shell: %.1f mm'%(dn,do))
