import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from scipy.signal import fftconvolve
import itertools, json
S=np.load(OUT+'sweeps_stand.npz')
trunk=S['trunk']; trunk_d=ndi.binary_dilation(trunk,iterations=3)&~trunk
ZONES={
 'top of torso (behind neck)': (-60,34, -30,30, 140,215),
 'under torso': (-50,52, -30,30, 0,74),
 'front of chest': (47,140, -30,30, 40,160),
}
def run(obst,perm,zone,clr=1.0,min_contact=300):
    x0,x1,y0,y1,z0,z1=zone
    shp=tuple(int(np.ceil(p+2*clr)) for p in perm)
    ix0=int(x0-clr-GX[0]); ix1=int(x1+clr-GX[0]); iy0=int(y0-clr-GY[0]); iy1=int(y1+clr-GY[0]); iz0=int(z0-clr-GZ[0]); iz1=int(z1+clr-GZ[0])
    sub=obst[ix0:ix1+1,iy0:iy1+1,iz0:iz1+1]
    if any(a<b for a,b in zip(sub.shape,shp)): return np.zeros((0,6)),None
    cor=fftconvolve(sub.astype(np.float32),np.ones(shp,np.float32),mode='valid')
    free=cor<0.5
    tsub=trunk_d[ix0:ix1+1,iy0:iy1+1,iz0:iz1+1]
    # contact voxels within the dilated box (shell of 3mm) : use box mask bigger by 3
    shp2=tuple(s+6 for s in shp)
    sub2=np.pad(tsub,3)
    ct=fftconvolve(sub2.astype(np.float32),np.ones(shp2,np.float32),mode='valid')
    ct=ct[:free.shape[0],:free.shape[1],:free.shape[2]]
    ok=free&(ct>=min_contact)
    ii,jj,kk=np.where(ok)
    X0=ii+ix0+GX[0]+clr; Y0=jj+iy0+GY[0]+clr; Z0=kk+iz0+GZ[0]+clr
    return np.c_[X0,X0+perm[0],Y0,Y0+perm[1],Z0,Z0+perm[2]],ct[ok]
if __name__=='__main__':
    dims=(66,31,29); rows=[]
    for lg in ['legs_full_mjcf','legs_pitch60','legs_p60_r12_y12','legs_gait_p60_r6_y6','legs_sagittal_p60']:
        for hv in ['none','head_nominal']:
            obst=trunk|S[lg]|(S[hv] if hv!='none' else False)
            for zn,z in ZONES.items():
                best=None;n=0
                for perm in sorted(set(itertools.permutations(dims))):
                    b,c=run(obst,perm,z)
                    if len(b)==0: continue
                    msk=abs((b[:,2]+b[:,3])/2)<=8
                    b=b[msk];c=c[msk]; n+=len(b)
                    if len(b):
                        i=np.argmax(c)
                        if best is None or c[i]>best[0]: best=(float(c[i]),perm,b[i].round(0).tolist())
                print(lg.ljust(22),hv.ljust(14),zn.ljust(27),n,best,flush=True)
                rows.append((lg,hv,zn,n,best))
    json.dump(rows,open(OUT+'zones3.json','w'),default=float)
