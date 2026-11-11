import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from design import *
from scipy import ndimage as ndi
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
B=np.load(OUT2+'blocked2.npz'); ceil_z=B['ceil_z']; others=B['others']
CLR=0.8
def voxelize_boxes(boxes_world,clr=CLR):
    g=np.zeros(D['head'].shape,bool)
    for lo,hi in boxes_world:
        ix=(xs>=lo[0]-clr)&(xs<=hi[0]+clr); iy=(ys>=lo[1]-clr)&(ys<=hi[1]+clr); iz=(zs>=lo[2]-clr)&(zs<=hi[2]+clr)
        g[np.ix_(ix,iy,iz)]=True
    return g
def wbox(b):
    a=S2W([b[0],b[2],b[4]])[0]; c=S2W([b[1],b[3],b[5]])[0]; return np.minimum(a,c),np.maximum(a,c)
def pi_envelope_boxes():
    bx=[(0,65,0,30,0,UB),(0,65,0,30,PCB_B,PCB_T),(0,65,0,30,PCB_T,TOP),HDR_REST,JUMP,USB_PLUG,SD_KEEP]
    return [wbox(b) for b in bx]
def need_from_grid(g):
    top=np.where(g.any(2),zs[len(zs)-1-np.argmax(g[:,:,::-1],axis=2)],-np.inf)
    cz=np.where(np.isnan(ceil_z),np.inf,ceil_z)
    return np.where(np.isfinite(top),np.maximum(top-cz,0),0), top
if __name__=='__main__':
    g=voxelize_boxes(pi_envelope_boxes())
    print('overlap with swept obstacles (voxels):',int((g&others).sum()))
    need,top=need_from_grid(g)
    print('max need',need.max(),'at',[ (xs[i],ys[j]) for i,j in [np.unravel_index(need.argmax(),need.shape)]])
    np.savez_compressed(OUT2+'need_pi.npz',need=need,top=top)
