import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from headmodel import *
import trimesh.ray.ray_triangle as rt
def column_intervals(mesh, xs, ys, z0, z1):
    """z-ray casting per (x,y) column; returns dict (i,j)-> list of (za,zb) solid intervals"""
    X,Y=np.meshgrid(xs,ys,indexing='ij')
    org=np.c_[X.ravel(),Y.ravel(),np.full(X.size,z0)]
    dirs=np.tile([0,0,1.0],(X.size,1))
    # only rays within xy bounds
    b=mesh.bounds
    sel=(org[:,0]>=b[0,0])&(org[:,0]<=b[1,0])&(org[:,1]>=b[0,1])&(org[:,1]<=b[1,1])
    idx=np.where(sel)[0]
    loc,ir,_=mesh.ray.intersects_location(org[idx],dirs[idx],multiple_hits=True)
    out={}
    for k in np.unique(ir):
        zs=np.sort(loc[ir==k][:,2])
        # dedupe near-equal
        zz=[zs[0]]
        for z in zs[1:]:
            if z-zz[-1]>1e-4: zz.append(z)
        if len(zz)%2: zz=zz[:-1] if len(zz)>1 else []
        iv=[(zz[i],zz[i+1]) for i in range(0,len(zz),2)]
        out[idx[k]]=iv
    return out
def build_grid(meshes, x0=-25,x1=110,y0=-50,y1=50,z0=195,z1=275, px=1.0, pz=0.5):
    xs=np.arange(x0,x1+1e-6,px); ys=np.arange(y0,y1+1e-6,px); zs=np.arange(z0,z1+1e-6,pz)
    G={}
    for name,me in meshes.items():
        g=np.zeros((len(xs),len(ys),len(zs)),bool)
        ci=column_intervals(me,xs,ys,z0-5,z1)
        for k,ivs in ci.items():
            i,j=divmod(k,len(ys))
            for a,b in ivs:
                g[i,j,(zs>=a)&(zs<=b)]=True
        G[name]=g
    return xs,ys,zs,G
if __name__=='__main__':
    parts,servos,m,d=head_parts()
    meshes=dict(parts)
    meshes.pop('head_pitch.stl'); meshes.pop('eye.stl',None)
    for i,(b,s) in enumerate(servos): meshes[f'servo{i}_{b}']=s
    xs,ys,zs,G=build_grid(meshes)
    np.savez_compressed(OUT+'head_grid.npz',xs=xs,ys=ys,zs=zs,**{k.replace('.','_'):v for k,v in G.items()})
    for k,v in G.items(): print(k, v.sum()*1.0*1.0*0.5/1000,'cm3')
