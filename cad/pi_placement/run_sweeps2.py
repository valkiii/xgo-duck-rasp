import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep2 import *
S=dict(np.load(OUT+'sweeps_stand.npz'))
m,d=load(); set_pose(m,d,'STAND')
trunk_core=ndi.binary_erosion(S['trunk'],iterations=2)
def gl(names):
    ids={m.body(n).id for n in names}
    return [g for g in range(m.ngeom) if m.geom_group[g]==2 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_bodyid[g] in ids]
lim={n:m.jnt_range[m.joint(n).id].copy() for n in ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle','right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle']}
home={n:d.qpos[m.joint(n).qposadr[0]] for n in lim}
legL=['yaw2roll','hip_l','upper_leg_left','leg','ankle_left']; legR=['bearing_roll','hip_l_2','upper_leg_right','leg_2','ankle_right']
chkL=gl(['upper_leg_left','leg','ankle_left']); chkR=gl(['upper_leg_right','leg_2','ankle_right'])
out={}
for tag,ph in [('legs_full_mjcf',None),('legs_pitch60',np.radians(60)),('legs_pitch40',np.radians(40))]:
    g=np.zeros(grid_shape(),bool)
    for side,bodies,chk in [('left',legL,chkL),('right',legR,chkR)]:
        r={}
        for j in ['hip_yaw','hip_roll','hip_pitch','knee','ankle']:
            n=f'{side}_{j}'; lo,hi=lim[n]
            if j=='hip_pitch' and ph is not None: lo=max(lo,home[n]-ph); hi=min(hi,home[n]+ph)
            r[n]=(lo,hi)
        gg,na,nt=sweep_filtered(m,d,gl(bodies),set(chk),r,trunk_core,ncfg=2500)
        print(tag,side,'accepted',na,'of',nt,flush=True)
        g|=gg
    out[tag]=ndi.binary_dilation(g,iterations=1)
    print(tag,out[tag].sum()/1000,flush=True)
S.update(out)
np.savez_compressed(OUT+'sweeps_stand.npz',**S)
