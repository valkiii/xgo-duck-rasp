import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from stackfit import stack_mask, rotz, UPS
from scipy.signal import fftconvolve
S=np.load(OUT+'sweeps_stand.npz')
xs=np.arange(GX[0],GX[1]+1.0); ys=np.arange(GY[0],GY[1]+1.0); zs=np.arange(GZ[0],GZ[1]+1.0)
def box_mask(a,b,c,clr=1.0):
    return np.ones((int(np.ceil(a+2*clr)),int(np.ceil(b+2*clr)),int(np.ceil(c+2*clr))),bool)
def search(obst, trunk, dims, label, attach_min=150, attach_d=3):
    res=[]
    for perm in sorted(set(__import__('itertools').permutations(dims))):
        mk=box_mask(*perm)
        cor=fftconvolve(obst.astype(np.float32),mk[::-1,::-1,::-1].astype(np.float32),mode='valid')
        free=cor<0.5
        # attachment: box dilated by attach_d overlaps trunk (trunk not in obst for this? obst includes trunk) -> compute using trunk
        mk2=np.ones((mk.shape[0]+2*attach_d,mk.shape[1]+2*attach_d,mk.shape[2]+2*attach_d),bool)
        tc=fftconvolve(trunk.astype(np.float32),mk2[::-1,::-1,::-1].astype(np.float32),mode='valid')
        # align: free index (i,j,k) corresponds to mk origin at i; mk2 origin at i-attach_d
        att=np.zeros_like(free)
        sh=(free.shape[0]-2*attach_d,free.shape[1]-2*attach_d,free.shape[2]-2*attach_d)
        # tc has size free.shape-2*attach_d ... valid conv: grid-mk2+1 = grid-mk+1-2d
        att[attach_d:attach_d+tc.shape[0],attach_d:attach_d+tc.shape[1],attach_d:attach_d+tc.shape[2]]=tc>attach_min
        ok=free&att
        res.append((perm,free.sum(),ok.sum(),np.argwhere(ok)))
        print(label,perm,'free',free.sum(),'free&attached',ok.sum(),flush=True)
    return res
if __name__=='__main__':
    trunk=S['trunk']; legs=S['legs_full_mjcf']
    out={}
    for hv in ['head_nominal','head_neck_back60','head_full']:
        obst=trunk|legs|S[hv]
        out[hv]=search(obst,trunk,(66,31,29),hv)
    import pickle; pickle.dump({k:[(p,f,o,a) for p,f,o,a in v] for k,v in out.items()},open(OUT+'torso_search.pkl','wb'))
