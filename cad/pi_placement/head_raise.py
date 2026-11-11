import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from stackfit import *
from scipy import ndimage as ndi
import pickle
D=np.load(OUT+'head_grid.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
B=np.load(OUT+'head_blocked.npz'); ceil_z=B['ceil_z']; others=B['others']
has=~np.isnan(ceil_z)
# footprint must be inside the shell columns; to avoid leaking out at the rim, only allow columns whose dome ceiling is >= 240 (not the rim skirt)
def blocked_for(delta, min_ceil=0.0):
    K=zs[None,None,:]
    free=(K<(ceil_z[:,:,None]+delta))&has[:,:,None]&(ceil_z[:,:,None]>=min_ceil)&~others&(K>=212)
    lab,n=ndi.label(free)
    seed=(np.argmin(abs(xs-80)),np.argmin(abs(ys-0)),np.argmin(abs(zs-245)))
    return ~(lab==lab[seed])
if __name__=='__main__':
    out={}
    for delta in [0,3,6,9,12,15]:
        bl=blocked_for(delta)
        for uname,R0 in UPS.items():
            for yaw in range(0,360,15):
                R=rotz(np.radians(yaw))@R0
                mk,mo=stack_mask(R)
                ok=feasible(bl,None,mk,mo)
                if ok.sum(): out[(delta,uname,yaw)]=np.argwhere(ok)
        print(delta,[ (k[1],k[2],len(v)) for k,v in out.items() if k[0]==delta][:8],flush=True)
    pickle.dump(out,open(OUT+'head_raise.pkl','wb'))
