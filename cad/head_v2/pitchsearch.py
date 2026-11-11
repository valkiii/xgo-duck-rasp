import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from piraise2 import *
def roty(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def search(boxes,yaws,pitches):
    best=[]
    for yaw in yaws:
        for p in pitches:
            R=rotz(np.radians(yaw))@roty(np.radians(p)); mk,mo=mask(R,boxes=boxes)
            ok=feasible(blocked_relaxed,mk)
            if not ok.any(): continue
            ex=excess_map(mk)
            kk=np.where(ok.any(2),ok.argmax(2),10**6)
            Z=np.where(kk<10**6,zs[np.minimum(kk,len(zs)-1)],np.inf); tot=Z+ex
            i,j=np.unravel_index(np.argmin(tot),tot.shape)
            best.append((float(tot[i,j]),yaw,p))
    best.sort(); return best
if __name__=='__main__':
    for nm,yaws in [('yaw~180',[170,180,190]),('yaw~0',[-10,0,10])]:
        b=search(H,yaws,range(-12,13,3))
        print(nm,[(round(r[0],1),r[1],r[2]) for r in b[:8]],flush=True)
