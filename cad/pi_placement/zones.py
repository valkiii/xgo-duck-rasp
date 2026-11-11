import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from scipy.signal import fftconvolve
import itertools
S=np.load(OUT+'sweeps_stand.npz')
trunk=S['trunk']
ZONES={ # (name): constraint function on box [x0,x1,y0,y1,z0,z1] (mm)
 'top of torso (behind neck)': lambda b: b[4]>=140 and b[0]>=-55 and b[1]<=34,
 'under torso': lambda b: b[5]<=74 and b[4]>=0 and b[0]>=-50 and b[1]<=52 and abs(b[2]+b[3])<12,
 'front of chest': lambda b: b[0]>=47 and b[4]>=40 and b[5]<=160 and abs(b[2]+b[3])<12,
}
def zone_search(obst,dims,zone,clr=1.0):
    best=[]
    for perm in sorted(set(itertools.permutations(dims))):
        shp=tuple(int(np.ceil(p+2*clr)) for p in perm)
        mk=np.ones(shp,np.float32)
        cor=fftconvolve(obst.astype(np.float32),mk[::-1,::-1,::-1],mode='valid')
        free=cor<0.5
        idx=np.argwhere(free)
        # box core extents (excluding clearance)
        for i,j,k in idx:
            b=[i+GX[0]+clr,i+GX[0]+clr+perm[0],j+GY[0]+clr,j+GY[0]+clr+perm[1],k+GZ[0]+clr,k+GZ[0]+clr+perm[2]]
            pass
    return None
def zone_free(obst,perm,zone,clr=1.0):
    shp=tuple(int(np.ceil(p+2*clr)) for p in perm)
    mk=np.ones(shp,np.float32)
    cor=fftconvolve(obst.astype(np.float32),mk[::-1,::-1,::-1],mode='valid')
    free=cor<0.5
    ii,jj,kk=np.where(free)
    x0=ii+GX[0]+clr; y0=jj+GY[0]+clr; z0=kk+GZ[0]+clr
    b=np.c_[x0,x0+perm[0],y0,y0+perm[1],z0,z0+perm[2]]
    sel=np.array([zone(r) for r in b]) if len(b)<2e6 else None
    return b[sel] if sel is not None and sel.any() else np.zeros((0,6))
if __name__=='__main__':
    dims=(66,31,29)
    for lg in ['legs_full_mjcf','legs_pitch60','legs_p60_r12_y12','legs_gait_p60_r6_y6','legs_sagittal_p60']:
        for hv in ['none','head_nominal','head_neck_back60']:
            obst=trunk|S[lg]|(S[hv] if hv!='none' else False)
            for zn,zf in ZONES.items():
                tot=0; ex=None
                for perm in sorted(set(itertools.permutations(dims))):
                    # restrict zone region first by cropping via bbox search to cut cost: cheap prefilter by sampling region
                    b=zone_free(obst,perm,zf)
                    tot+=len(b)
                    if len(b) and ex is None: ex=(perm,b[0].round(0).tolist())
                print(lg.ljust(22),hv.ljust(18),zn.ljust(27),tot,ex,flush=True)
