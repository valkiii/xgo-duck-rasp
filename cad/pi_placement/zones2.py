import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from scipy.signal import fftconvolve
import itertools, json
S=np.load(OUT+'sweeps_stand.npz')
trunk=S['trunk']
# zone: bounds for the box core min/max corner: (xmin,xmax_end, ymin,ymax_end, zmin,zmax_end) in mm, and centre-y tolerance
ZONES={
 'top of torso (behind neck)': (-60,34, -30,30, 140,215),
 'under torso': (-50,52, -30,30, 0,74),
 'front of chest': (47,140, -30,30, 40,160),
}
def run(obst,perm,zone,clr=1.0):
    x0,x1,y0,y1,z0,z1=zone
    shp=tuple(int(np.ceil(p+2*clr)) for p in perm)
    # crop: indices for core region
    ix0=int(x0-clr-GX[0]); ix1=int(x1+clr-GX[0]); iy0=int(y0-clr-GY[0]); iy1=int(y1+clr-GY[0]); iz0=int(z0-clr-GZ[0]); iz1=int(z1+clr-GZ[0])
    sub=obst[ix0:ix1+1,iy0:iy1+1,iz0:iz1+1]
    if any(a<b for a,b in zip(sub.shape,shp)): return np.zeros((0,6))
    cor=fftconvolve(sub.astype(np.float32),np.ones(shp,np.float32),mode='valid')
    free=cor<0.5
    ii,jj,kk=np.where(free)
    X0=ii+ix0+GX[0]+clr; Y0=jj+iy0+GY[0]+clr; Z0=kk+iz0+GZ[0]+clr
    return np.c_[X0,X0+perm[0],Y0,Y0+perm[1],Z0,Z0+perm[2]]
if __name__=='__main__':
    dims=(66,31,29); rows=[]
    for lg in ['legs_full_mjcf','legs_pitch60','legs_p60_r12_y12','legs_gait_p60_r6_y6','legs_sagittal_p60']:
        for hv in ['none','head_nominal']:
            obst=trunk|S[lg]|(S[hv] if hv!='none' else False)
            for zn,z in ZONES.items():
                allb=[]
                for perm in sorted(set(itertools.permutations(dims))):
                    b=run(obst,perm,z)
                    # require box roughly centred in y (centre within +-8)
                    if len(b): 
                        b=b[abs((b[:,2]+b[:,3])/2)<=8]
                    for r in b: allb.append((perm,*r))
                n=len(allb)
                ex=None
                if n:
                    a=np.array([r[1:] for r in allb]); ex=a[np.argmin(a[:,4])].round(0).tolist()
                print(lg.ljust(22),hv.ljust(14),zn.ljust(27),n,ex,flush=True)
                rows.append((lg,hv,zn,n,ex))
    json.dump(rows,open(OUT+'zones2.json','w'),default=float)
