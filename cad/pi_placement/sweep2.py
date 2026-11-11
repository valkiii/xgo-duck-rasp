import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
import itertools
def sweep_filtered(m,d,gidx_all,gidx_check,qranges,trunk_core,nsamp=2000,ncfg=4000,seed=0,max_hits=3):
    rng=np.random.default_rng(seed)
    set_pose(m,d,'STAND'); base=d.qpos.copy()
    pts=sample_geoms(m,gidx_all,nsamp)
    names=list(qranges); adrs=[m.joint(n).qposadr[0] for n in names]
    lo=np.array([qranges[n][0] for n in names]); hi=np.array([qranges[n][1] for n in names])
    cfgs=[lo+(hi-lo)*rng.random(len(names)) for _ in range(ncfg)]
    lv=[np.array([lo[i],(lo[i]+hi[i])/2,hi[i]]) for i in range(len(names))]
    cor=list(itertools.product(*lv)); cfgs+=[np.array(c) for c in cor]
    grid=np.zeros(grid_shape(),bool); nacc=0
    for q in cfgs:
        d.qpos[:]=base
        for a,v in zip(adrs,q): d.qpos[a]=v
        mujoco.mj_kinematics(m,d)
        W={}
        hits=0
        for gi,p in pts.items():
            Wg=((d.geom_xmat[gi].reshape(3,3)@p.T).T+d.geom_xpos[gi])*1000
            W[gi]=Wg
            if gi in gidx_check:
                ix,iy,iz=voxelize_points(Wg); hits+=trunk_core[ix,iy,iz].sum()
        if hits>max_hits: continue
        nacc+=1
        for gi,Wg in W.items():
            ix,iy,iz=voxelize_points(Wg); grid[ix,iy,iz]=True
    return grid,nacc,len(cfgs)
