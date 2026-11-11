import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
m,d=load(); set_pose(m,d,'STAND')
def geoms_of_bodies(names):
    ids=set()
    for n in names:
        ids.add(m.body(n).id)
    return [g for g in range(m.ngeom) if (m.geom_group[g]==2 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_bodyid[g] in ids)]
def J(n): return m.joint(n)
lim={n:m.jnt_range[m.joint(n).id].copy() for n in ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle','right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle','neck_pitch','head_pitch','head_yaw','head_roll']}
print(lim)
stand=np.array([0,0,0])
set_pose(m,d,'STAND'); home={n:d.qpos[m.joint(n).qposadr[0]] for n in lim}
print(home)
out={}
legL=['yaw2roll','hip_l','upper_leg_left','leg','ankle_left']
legR=['bearing_roll','hip_l_2','upper_leg_right','leg_2','ankle_right']
headb=['neck','neck_pitch','yaw_roll_motion','jaw_soft']
def leg_ranges(side,pitch_halfrange):
    r={}
    for j in ['hip_yaw','hip_roll','hip_pitch','knee','ankle']:
        n=f'{side}_{j}'; lo,hi=lim[n]
        if j=='hip_pitch' and pitch_halfrange is not None:
            lo=max(lo,home[n]-pitch_halfrange); hi=min(hi,home[n]+pitch_halfrange)
        r[n]=(lo,hi)
    return r
for tag,ph in [('legs_full',None),('legs_pitch60',np.radians(60))]:
    g=np.zeros(grid_shape(),bool)
    for side,bodies in [('left',legL),('right',legR)]:
        r=leg_ranges(side,ph)
        g|=sweep(m,d,geoms_of_bodies(bodies),r,nsamp=2500,ncfg=1500)
    out[tag]=finish(g); print(tag,out[tag].sum()/1000,'cm3',flush=True)
# head: neck range variants
def headr(neck_lo,neck_hi,full_head=True):
    r={'neck_pitch':(neck_lo,neck_hi)}
    if full_head:
        for n in ['head_pitch','head_yaw','head_roll']: r[n]=tuple(lim[n])
    return r
for tag,(lo,hi,fh) in {'head_full':(lim['neck_pitch'][0],lim['neck_pitch'][1],True),
                       'head_neck_back60':(home['neck_pitch']-0.0,np.radians(60),True),
                       'head_nominal':(home['neck_pitch']-np.radians(15),home['neck_pitch']+np.radians(15),False)}.items():
    r=headr(lo,hi,fh)
    if not fh: r.update({'head_pitch':(home['head_pitch']-np.radians(20),home['head_pitch']+np.radians(20)),'head_yaw':(-0.5,0.5)})
    g=sweep(m,d,geoms_of_bodies(headb),r,nsamp=3000,ncfg=2500)
    out[tag]=finish(g); print(tag,out[tag].sum()/1000,'cm3',flush=True)
# trunk solid
g=np.zeros(grid_shape(),bool)
gi=[g_ for g_ in range(m.ngeom) if m.geom_group[g_]==2 and m.body(m.geom_bodyid[g_]).name=='trunk_base'][0]
pts=sample_geoms(m,[gi],400000)[gi]
set_pose(m,d,'STAND')
W=(d.geom_xmat[gi].reshape(3,3)@pts.T).T+d.geom_xpos[gi]
ix,iy,iz=voxelize_points(W*1000); g[ix,iy,iz]=True
out['trunk']=finish(g,1); print('trunk',out['trunk'].sum()/1000)
np.savez_compressed(OUT+'sweeps_stand.npz',**out)
