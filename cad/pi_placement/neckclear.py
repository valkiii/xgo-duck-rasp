import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from torso_search import search, box_mask
from scipy.signal import fftconvolve
S=np.load(OUT+'sweeps_stand.npz')
m,d=load()
def gl(names):
    ids={m.body(n).id for n in names}
    return [g for g in range(m.ngeom) if m.geom_group[g]==2 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_bodyid[g] in ids]
hg=gl(['neck','neck_pitch','yaw_roll_motion','jaw_soft'])
trunk=S['trunk']
xs=np.arange(GX[0],GX[1]+1.0); ys=np.arange(GY[0],GY[1]+1.0); zs=np.arange(GZ[0],GZ[1]+1.0)
res={}
for th in range(-30,65,5):
    for dh in [0]:   # head pitch relative = th + dh (keeps head level-ish)
        set_pose(m,d,'STAND')
        d.qpos[m.joint('neck_pitch').qposadr[0]]=np.radians(th)
        d.qpos[m.joint('head_pitch').qposadr[0]]=np.radians(th+dh)
        mujoco.mj_kinematics(m,d)
        g=np.zeros(grid_shape(),bool)
        P=sample_geoms(m,hg,60000)
        for gi,p in P.items():
            W=((d.geom_xmat[gi].reshape(3,3)@p.T).T+d.geom_xpos[gi])*1000
            ix,iy,iz=voxelize_points(W); g[ix,iy,iz]=True
        g=ndi.binary_fill_holes(ndi.binary_dilation(g,iterations=1))
        obst=trunk|g|S['legs_full_mjcf']
        mk=box_mask(66,31,29)
        free=fftconvolve(obst.astype(np.float32),mk[::-1,::-1,::-1].astype(np.float32),mode='valid')<0.5
        # torso-top zone: x -50..30, |y|<=35, z>=140
        ii,jj,kk=np.where(free)
        X=ii+GX[0]; Y=jj+GY[0]; Z=kk+GZ[0]
        sel=(X>=-52)&(X+mk.shape[0]<=34)&(Z>=140)&(Z<=170)&(abs(Y+16)<30)
        # attached to trunk top: box bottom within 3 mm of trunk surface
        print(th, 'free boxes in torso-top zone:',sel.sum(), (X[sel].min(),X[sel].max(),Z[sel].min(),Z[sel].max()) if sel.sum() else '')
