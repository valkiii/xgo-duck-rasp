import numpy as np, trimesh, mujoco, os
R='/Users/vc/Research/xgo-duck/cad/ref/'
XML=R+'xgoduck_rl/src/mjlab_microduck/robot/xgoduck/scene_walk.xml'
ASSETS=R+'xgoduck_rl/src/mjlab_microduck/robot/xgoduck/assets/'
STRUCT=R+'xgoduck_hardware/structure/'
OUT='/Users/vc/Research/xgo-duck/cad/pi_placement/'

def load():
    m=mujoco.MjModel.from_xml_path(XML); d=mujoco.MjData(m); return m,d

def set_pose(m,d,key='INIT',q=None):
    mujoco.mj_resetDataKeyframe(m,d,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,key))
    if q:
        for k,v in q.items():
            j=m.joint(k); d.qpos[j.qposadr[0]]=v
    mujoco.mj_forward(m,d)

_cache={}
def mesh_of(m,gi):
    mid=m.geom_dataid[gi]
    name=m.mesh(mid).name
    if name not in _cache:
        _cache[name]=trimesh.load(ASSETS+name+'.stl')  # metres
    return _cache[name]

def world_meshes(m,d,scale=1000.0):
    """returns dict bodyname-> list of (geomname, trimesh in world frame, mm)"""
    out={}
    for gi in range(m.ngeom):
        if m.geom_type[gi]!=mujoco.mjtGeom.mjGEOM_MESH: continue
        if m.geom_group[gi]!=2: continue
        me=mesh_of(m,gi).copy()
        # mesh vertices in mujoco are re-centred; use model mesh pos/quat
        mid=m.geom_dataid[gi]
        # MuJoCo recenters meshes to inertial frame: mesh_pos/mesh_quat give transform from original to centered
        mp=m.mesh_pos[mid]; mq=m.mesh_quat[mid]
        T0=np.eye(4); Rm=np.zeros(9); mujoco.mju_quat2Mat(Rm,mq); T0[:3,:3]=Rm.reshape(3,3); T0[:3,3]=mp
        # original v -> centered v' = R^T (v - p)
        Tinv=np.linalg.inv(T0)
        me.apply_transform(Tinv)
        T=np.eye(4); T[:3,:3]=d.geom_xmat[gi].reshape(3,3); T[:3,3]=d.geom_xpos[gi]
        me.apply_transform(T); me.apply_scale(scale)
        b=m.body(m.geom_bodyid[gi]).name
        out.setdefault(b,[]).append((m.geom(gi).name or m.mesh(mid).name,me))
    return out
