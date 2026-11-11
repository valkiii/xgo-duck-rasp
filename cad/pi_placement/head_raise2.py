import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from stackfit import *
from scipy import ndimage as ndi
import pickle
D=np.load(OUT+'head_grid.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
B=np.load(OUT+'head_blocked.npz'); ceil_z=B['ceil_z']; others=B['others']
has=~np.isnan(ceil_z)
MIN_CEIL=236.0  # columns whose existing dome is lower than this are 'rim' columns: new bump cannot go there without widening head
rim=~has | (ceil_z<MIN_CEIL)
rim_d=ndi.binary_dilation(rim,iterations=2)   # 2 mm wall
K=zs[None,None,:]
blocked=others|rim_d[:,:,None]|(K<212)
# interior connectivity
free=~blocked
lab,n=ndi.label(free)
seed=(np.argmin(abs(xs-80)),np.argmin(abs(ys-0)),np.argmin(abs(zs-245)))
# above-ceiling cells are 'free' in this relaxed model, label connectivity incl. above dome; fine
comp=lab==lab[seed]
blocked=~comp
cz=np.where(has,ceil_z,np.inf)
def excess_map(mk, mo, pz=0.5):
    nx,ny,nz=mk.shape
    colany=mk.any(2)
    ctop=np.where(colany, nz-1-np.argmax(mk[:,:,::-1],axis=2), -1)*pz   # relative top (mm above mask z origin)
    NX,NY=len(xs),len(ys)
    out=np.full((NX-nx+1,NY-ny+1),-np.inf)
    for a,b in zip(*np.where(colany)):
        sub=cz[a:a+NX-nx+1,b:b+NY-ny+1]
        out=np.maximum(out,ctop[a,b]-sub)
    return out
if __name__=='__main__':
    res=[]
    for uname,R0 in UPS.items():
        for yaw in range(0,360,10):
            R=rotz(np.radians(yaw))@R0
            mk,mo=stack_mask(R)
            ok=feasible(blocked,None,mk,mo)
            if not ok.any(): continue
            ex=excess_map(mk,mo)
            # lowest feasible k per (i,j)
            kk=np.where(ok.any(2), ok.argmax(2), 10**6)
            Z=np.where(kk<10**6, zs[np.minimum(kk,len(zs)-1)] , np.inf)
            tot=Z+ex
            # pick min raise = max(tot,0)
            i,j=np.unravel_index(np.argmin(tot),tot.shape)
            res.append((tot[i,j],uname,yaw,i,j,kk[i,j],mo))
            print(uname,yaw,'min raise %.1f'%tot[i,j],'at grid',i,j,'k',kk[i,j],flush=True)
    res.sort(key=lambda r:r[0])
    pickle.dump(res,open(OUT+'head_raise2.pkl','wb'))
    print('best',res[:5])
