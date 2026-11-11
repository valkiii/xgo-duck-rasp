import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from piraise2 import *
def pose_report(yaw,boxes=H):
    R=rotz(np.radians(yaw)); mk,mo=mask(R,boxes=boxes)
    ok=feasible(blocked_relaxed,mk); ex=excess_map(mk)
    kk=np.where(ok.any(2),ok.argmax(2),10**6)
    Z=np.where(kk<10**6,zs[np.minimum(kk,len(zs)-1)],np.inf); tot=Z+ex
    return R,mk,mo,ok,ex,kk,Z,tot
if __name__=='__main__':
    for yaw in [180,190,170,0,270,90]:
        R,mk,mo,ok,ex,kk,Z,tot=pose_report(yaw)
        i,j=np.unravel_index(np.argmin(tot),tot.shape)
        cen=np.array([xs[i],ys[j],Z[i,j]])-np.array(mo)   # stack-frame C point world position (z=0 bottom of underside parts, mask includes clr)
        print('yaw',yaw,'raise %.1f'%tot[i,j],'stack C at world',cen.round(1),'-> underside-parts bottom z (incl 0.8 clr gap) = ',round(Z[i,j]-mo[2]+0.0,1))
