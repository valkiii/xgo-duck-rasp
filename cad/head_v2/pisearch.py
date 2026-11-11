import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from pifit import *
from common2 import OUT2
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
shell=D['shell']
others=D['head']|D['eyeshell']|D['support']|D['Srear']|D['Sfront']|D['jaw']|D['yawg']
nz=len(zs); has=shell.any(2); first=np.where(has,shell.argmax(2),nz)
K=np.arange(nz)[None,None,:]
cavity=(K<first[:,:,None])&has[:,:,None]
ceil_z=np.where(has,zs[np.minimum(first,nz-1)],np.nan)
blocked=others|~cavity; blocked[:,:,zs<212]=True
free=~blocked; lab,n=ndi.label(free); seed=(np.argmin(abs(xs-80)),np.argmin(abs(ys-0)),np.argmin(abs(zs-245)))
comp=lab==lab[seed]; blocked=~comp
np.savez_compressed(OUT2+'blocked2.npz',blocked=blocked,ceil_z=ceil_z,others=others)
print('interior free cm3',comp.sum()*0.5/1000)
res=[]
for yaw in range(0,360,10):
    R=rotz(np.radians(yaw)); mk,mo=mask(R)
    ok=feasible(blocked,mk)
    if ok.any(): res.append((yaw,int(ok.sum())))
print('hard-feasible yaws:',res)
