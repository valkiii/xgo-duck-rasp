import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
import scipy.ndimage as ndi
rng=np.random.default_rng(1)
GX=(-120,160); GY=(-110,110); GZ=(-5,290)
def grid_shape(px=1.0):
    return (int((GX[1]-GX[0])/px)+1,int((GY[1]-GY[0])/px)+1,int((GZ[1]-GZ[0])/px)+1)
def voxelize_points(pts,px=1.0):
    ix=np.round((pts[:,0]-GX[0])/px).astype(int); iy=np.round((pts[:,1]-GY[0])/px).astype(int); iz=np.round((pts[:,2]-GZ[0])/px).astype(int)
    s=grid_shape(px); ok=(ix>=0)&(ix<s[0])&(iy>=0)&(iy<s[1])&(iz>=0)&(iz<s[2])
    return ix[ok],iy[ok],iz[ok]
def link_geoms(m, joints_filter):
    """geoms (visual meshes group 2) whose body is a descendant (incl.) of bodies of given joints"""
    pass
def sample_geoms(m,gidx,n):
    out={}
    for gi in gidx:
        me=mesh_of(m,gi).copy()
        mid=m.geom_dataid[gi]; mp=m.mesh_pos[mid]; mq=m.mesh_quat[mid]
        T0=np.eye(4); Rm=np.zeros(9); mujoco.mju_quat2Mat(Rm,mq); T0[:3,:3]=Rm.reshape(3,3); T0[:3,3]=mp
        me.apply_transform(np.linalg.inv(T0))
        pts=me.sample(n)+0  # local geom frame, metres
        out[gi]=pts
    return out
def sweep(m,d,gidx,qranges,nsamp=3000,ncfg=3000,key='STAND',extra_cfgs=None,px=1.0,seed=0):
    """qranges: dict joint name -> (lo,hi) for random sampling; others stay at key pose. returns bool grid"""
    rng=np.random.default_rng(seed)
    set_pose(m,d,key)
    base=d.qpos.copy()
    pts=sample_geoms(m,gidx,nsamp)
    grid=np.zeros(grid_shape(px),bool)
    names=list(qranges)
    adrs=[m.joint(n).qposadr[0] for n in names]
    lo=np.array([qranges[n][0] for n in names]); hi=np.array([qranges[n][1] for n in names])
    cfgs=[lo+(hi-lo)*rng.random(len(names)) for _ in range(ncfg)]
    # corners (3 levels per joint, capped)
    import itertools
    lv=[np.array([lo[i],(lo[i]+hi[i])/2,hi[i]]) for i in range(len(names))]
    cor=list(itertools.product(*lv))
    if len(cor)>1500: cor=[cor[i] for i in rng.choice(len(cor),1500,replace=False)]
    cfgs+= [np.array(c) for c in cor]
    if extra_cfgs: cfgs+=extra_cfgs
    for q in cfgs:
        d.qpos[:]=base
        for a,v in zip(adrs,q): d.qpos[a]=v
        mujoco.mj_kinematics(m,d)
        for gi,p in pts.items():
            W=(d.geom_xmat[gi].reshape(3,3)@p.T).T+d.geom_xpos[gi]
            ix,iy,iz=voxelize_points(W*1000,px); grid[ix,iy,iz]=True
    return grid
def finish(grid,dil=1):
    g=ndi.binary_dilation(grid,iterations=dil)
    return ndi.binary_fill_holes(g)
