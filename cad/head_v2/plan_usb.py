import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from planner import *
import numpy as np
occ=np.load(OUT2+'plan_occ_v0.npy'); occ1,zs1=to1mm(occ)
for name,start,goal,clear in [('usb',(27.0,27.5,247.5),(27.5,-21.5,224.0),3.2),('wires',(59.0,-12.5,260.0),(27.5,-18.0,224.0),3.6)]:
    try:
        p=plan(occ1,zs1,start,goal,clear)
        L=np.sum(np.linalg.norm(np.diff(p,axis=0),axis=1))
        sp=smooth(p); r=curvature_radius(sp)
        print(name,'raw nodes',len(p),'length %.0f mm'%L,'smoothed len %.0f'%np.sum(np.linalg.norm(np.diff(sp,axis=0),axis=1)),'min radius %.1f (5th pct %.1f)'%(r[5:-5].min(),np.percentile(r,5)))
        np.save(OUT2+f'path_{name}_v0.npy',sp); np.save(OUT2+f'path_{name}_raw_v0.npy',p)
    except Exception as e: print(name,'FAILED',e)
