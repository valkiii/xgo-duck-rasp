import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from pathtools import *
from design import *
from scipy import ndimage as ndi
from scipy.interpolate import splprep, splev
D=np.load(OUT2+'grid2.npz'); XS,YS,ZS=D['xs'],D['ys'],D['zs']
_edt={}
def edt_for(name):
    if name not in _edt:
        occ=np.load(OUT2+name)
        _edt[name]=ndi.distance_transform_edt(~occ,sampling=(1.0,1.0,0.5))
    return _edt[name]
def spline_path(ctrl,n=300,s=0,k=3):
    tck,u=splprep(np.asarray(ctrl).T,s=s,k=k); uu=np.linspace(0,1,n); x,y,z=splev(uu,tck); return np.c_[x,y,z]
def radius(P):
    d1=np.gradient(P,axis=0); d2=np.gradient(d1,axis=0)
    cr=np.linalg.norm(np.cross(d1,d2),axis=1); n=np.linalg.norm(d1,axis=1)**3
    return 1.0/np.maximum(cr/np.maximum(n,1e-9),1e-9)
def clearance(P,occ_name):
    E=edt_for(occ_name)
    i=np.clip(np.round(P[:,0]-XS[0]).astype(int),0,E.shape[0]-1); j=np.clip(np.round(P[:,1]-YS[0]).astype(int),0,E.shape[1]-1); k=np.clip(np.round((P[:,2]-ZS[0])/0.5).astype(int),0,E.shape[2]-1)
    return E[i,j,k]
def report(name,P,r,occ_name):
    c=clearance(P,occ_name); rad=radius(P)
    bad=np.where(c<r)[0]
    print(f'{name}: length {path_length(P):.0f} mm, min clearance-to-surface {c.min():.2f} (tube r {r}), min bend radius {rad[3:-3].min():.1f}, 5th pct {np.percentile(rad[3:-3],5):.1f}, violations {len(bad)}')
    if len(bad): print('   first violations at',[tuple(np.round(P[b],1)) for b in bad[:4]],'clr',[round(float(c[b]),2) for b in bad[:4]])
def table(P,occ_name,every=15):
    c=clearance(P,occ_name); r=radius(P)
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]
    for i in range(0,len(P),every): print(f'  s={s[i]:6.1f} p=({P[i,0]:6.1f},{P[i,1]:6.1f},{P[i,2]:6.1f}) clr={c[i]:4.1f} R={min(r[i],99):5.1f}')
