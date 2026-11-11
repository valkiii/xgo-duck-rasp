import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
import numpy as np, heapq
from scipy import ndimage as ndi
from scipy.interpolate import splprep, splev
from design import OUT2
D=np.load(OUT2+'grid2.npz'); XS,YS,ZS=D['xs'],D['ys'],D['zs']
def to1mm(occ):
    # downsample z by 2 (any)
    nz=occ.shape[2]//2*2
    o=occ[:,:,:nz].reshape(occ.shape[0],occ.shape[1],nz//2,2).any(3)
    return o, ZS[:nz:2]+0.25
def plan(occ1,zs1,start,goal,clear,extra_cost=None,max_nodes=3_000_000):
    free=ndi.distance_transform_edt(~occ1)>=clear
    def idx(p): return (int(round(p[0]-XS[0])),int(round(p[1]-YS[0])),int(np.argmin(abs(zs1-p[2]))))
    s=idx(start); g=idx(goal)
    if not free[s] or not free[g]: raise RuntimeError(f'start/goal blocked {free[s]} {free[g]}')
    nb=[(dx,dy,dz,np.sqrt(dx*dx+dy*dy+(dz*1.0)**2)) for dx in (-1,0,1) for dy in (-1,0,1) for dz in (-1,0,1) if (dx,dy,dz)!=(0,0,0)]
    shape=free.shape
    gscore={s:0.0}; came={}; pq=[(0,s)]
    while pq:
        f,u=heapq.heappop(pq)
        if u==g: break
        gu=gscore[u]
        for dx,dy,dz,c in nb:
            v=(u[0]+dx,u[1]+dy,u[2]+dz)
            if not(0<=v[0]<shape[0] and 0<=v[1]<shape[1] and 0<=v[2]<shape[2]) or not free[v]: continue
            ng=gu+c
            if ng<gscore.get(v,1e18):
                gscore[v]=ng; came[v]=u
                h=np.sqrt((v[0]-g[0])**2+(v[1]-g[1])**2+(v[2]-g[2])**2)
                heapq.heappush(pq,(ng+h,v))
    if g not in came and g!=s: raise RuntimeError('no path')
    path=[g]
    while path[-1]!=s: path.append(came[path[-1]])
    path=path[::-1]
    return np.array([[XS[0]+p[0],YS[0]+p[1],zs1[p[2]]] for p in path])
def smooth(path,sm=60.0,n=400):
    # decimate then spline
    pts=path[::3]; 
    if np.linalg.norm(pts[-1]-path[-1])>0.1: pts=np.vstack([pts,path[-1]])
    tck,u=splprep(pts.T,s=sm,k=3)
    uu=np.linspace(0,1,n); x,y,z=splev(uu,tck); return np.c_[x,y,z]
def curvature_radius(P):
    d1=np.gradient(P,axis=0); d2=np.gradient(d1,axis=0)
    cr=np.linalg.norm(np.cross(d1,d2),axis=1); n=np.linalg.norm(d1,axis=1)**3
    k=cr/np.maximum(n,1e-9); return 1.0/np.maximum(k,1e-9)
