import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from scipy.signal import fftconvolve
S=np.load(OUT+'sweeps_stand.npz'); trunk=S['trunk']
m,d=load()
def gl(names):
    ids={m.body(n).id for n in names}
    return [g for g in range(m.ngeom) if m.geom_group[g]==2 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_bodyid[g] in ids]
hg=gl(['neck','neck_pitch','yaw_roll_motion','jaw_soft'])
P=sample_geoms(m,hg,2500)
box=(66,31,29); clr=1.0
shp=tuple(int(np.ceil(p+2*clr)) for p in box)
rng=np.random.default_rng(0)
ja={n:m.joint(n).qposadr[0] for n in ['neck_pitch','head_pitch','head_yaw','head_roll']}
def headgrid(thb,thf,dh,hy,hr,n=2500):
    set_pose(m,d,'STAND'); base=d.qpos.copy()
    g=np.zeros(grid_shape(),bool)
    cfg=[]
    for _ in range(n):
        th=rng.uniform(thf,thb); cfg.append((th,th+rng.uniform(-dh,dh),rng.uniform(-hy,hy),rng.uniform(-hr,hr)))
    for th in (thf,thb):
        for dd in (-dh,dh):
            for yy in (-hy,hy):
                for rr in (-hr,hr): cfg.append((th,th+dd,yy,rr))
    for c in cfg:
        d.qpos[:]=base
        for k,v in zip(['neck_pitch','head_pitch','head_yaw','head_roll'],c): d.qpos[ja[k]]=v
        mujoco.mj_kinematics(m,d)
        for gi,p in P.items():
            W=((d.geom_xmat[gi].reshape(3,3)@p.T).T+d.geom_xpos[gi])*1000
            ix,iy,iz=voxelize_points(W); g[ix,iy,iz]=True
    return ndi.binary_fill_holes(ndi.binary_dilation(g,iterations=1))
def fit(obst):
    x0,x1,y0,y1,z0,z1=(-60,34,-30,30,140,215)
    ix0=int(x0-clr-GX[0]); ix1=int(x1+clr-GX[0]); iy0=int(y0-clr-GY[0]); iy1=int(y1+clr-GY[0]); iz0=int(z0-clr-GZ[0]); iz1=int(z1+clr-GZ[0])
    sub=obst[ix0:ix1+1,iy0:iy1+1,iz0:iz1+1]
    cor=fftconvolve(sub.astype(np.float32),np.ones(shp,np.float32),mode='valid'); free=cor<0.5
    ii,jj,kk=np.where(free); X0=ii+ix0+GX[0]+clr; Y0=jj+iy0+GY[0]+clr; Z0=kk+iz0+GZ[0]+clr
    sel=(Z0<=162)&(abs(Y0+15.5)<=10)
    return sel.sum(), (X0[sel].min(),X0[sel].max(),Z0[sel].min()) if sel.sum() else None
if __name__=='__main__':
    for thb in [0,5,10,15,20,30]:
        for tag,dh,hy,hr in [('head tracks neck (dh=0), yaw0 roll0',0,0,0),('dh+-25 yaw+-60 roll+-25',25,60,25)]:
            g=headgrid(np.radians(thb),np.radians(-30),np.radians(dh),np.radians(hy),np.radians(hr))
            print(thb,tag,fit(trunk|g),flush=True)
