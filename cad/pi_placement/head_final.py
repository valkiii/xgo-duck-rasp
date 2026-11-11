import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_raise2 import *
import scipy.ndimage as ndi
YAW=180; CX,CY=52,8
R=rotz(np.radians(YAW)); mk,mo=stack_mask(R)
ok=feasible(blocked,None,mk,mo); ex=excess_map(mk,mo)
i=int(round(CX+mo[0]-xs[0])); j=int(round(CY+mo[1]-ys[0]))
kk=ok[i,j].argmax(); assert ok[i,j,kk]
Zc=zs[kk]-mo[2]
centre=np.array([CX,CY,Zc])
print('stack centre (world, INIT frame)',centre,'excess',ex[i,j]+zs[kk])
def stack_world_mesh():
    from stackfit import BOXES
    ms=[]
    for (x0,x1,y0,y1,z0,z1) in BOXES:
        b=trimesh.creation.box(extents=[x1-x0,y1-y0,z1-z0]); b.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]); ms.append(b)
    return ms
def to_world(me):
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=centre-R@np.array([L/2,W/2,0.0]); me=me.copy(); me.apply_transform(T); return me,T
if __name__=='__main__':
    from stackfit import L,W
    # required lift map per column
    ms=[to_world(b)[0] for b in stack_world_mesh()]
    ms.append(to_world(trimesh.creation.box(extents=[65+2,30+2,1.5]).apply_translation([32.5,15,-0.75]) or trimesh.creation.box())[0]) if False else None
    top=np.full((len(xs),len(ys)),-np.inf)
    # rasterise stack top per column via z-ray on each box
    for me in ms:
        b=me.bounds
        for a in range(len(xs)):
            if xs[a]<b[0,0]-1.5 or xs[a]>b[1,0]+1.5: continue
            for c in range(len(ys)):
                if ys[c]<b[0,1]-1.5 or ys[c]>b[1,1]+1.5: continue
                # box rotated by 180 so axis aligned: inside test with margin 1.0
                if b[0,0]-1<=xs[a]<=b[1,0]+1 and b[0,1]-1<=ys[c]<=b[1,1]+1: top[a,c]=max(top[a,c],b[1,2]+1.0)
    need=np.where(np.isfinite(top),np.maximum(top-np.where(np.isnan(ceil_z),-1e9,ceil_z),0),0)
    print('max need',need.max(),'at',np.unravel_index(need.argmax(),need.shape))
    np.savez_compressed(OUT+'head_need.npz',need=need,top=top,centre=centre)
