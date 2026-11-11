import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_raise2 import *
def roty(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rotx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
res=[]
for pitch in [-10,-5,0,5,10]:
  for roll in [-6,0,6]:
    for yaw in list(range(150,230,10))+list(range(-30,30,10)):
        R=rotz(np.radians(yaw))@roty(np.radians(pitch))@rotx(np.radians(roll))
        mk,mo=stack_mask(R)
        ok=feasible(blocked,None,mk,mo)
        if not ok.any(): continue
        ex=excess_map(mk,mo)
        kk=np.where(ok.any(2), ok.argmax(2), 10**6)
        Z=np.where(kk<10**6, zs[np.minimum(kk,len(zs)-1)] , np.inf)
        tot=Z+ex; i,j=np.unravel_index(np.argmin(tot),tot.shape)
        res.append((tot[i,j],pitch,roll,yaw,i,j))
res.sort()
for r in res[:12]: print(r)
