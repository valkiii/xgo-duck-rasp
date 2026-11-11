import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from pifit import *
import pifit
from common2 import OUT2
from scipy import ndimage as ndi
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
B=np.load(OUT2+'blocked2.npz'); ceil_z=B['ceil_z']; others=B['others']
has=~np.isnan(ceil_z)
MIN_CEIL=236.0
rim=~has|(ceil_z<MIN_CEIL); rim_d=ndi.binary_dilation(rim,iterations=2)
K=zs[None,None,:]
blk=others|rim_d[:,:,None]|(K<212)
free=~blk; lab,n=ndi.label(free); seed=(np.argmin(abs(xs-80)),np.argmin(abs(ys-0)),np.argmin(abs(zs-245)))
blocked_relaxed=~(lab==lab[seed])
cz=np.where(has,ceil_z,np.inf)
def excess_map(mk,pz=0.5):
    nx,ny,nzz=mk.shape; colany=mk.any(2)
    ctop=np.where(colany,nzz-1-np.argmax(mk[:,:,::-1],axis=2),-1)*pz
    NX,NY=len(xs),len(ys); out=np.full((NX-nx+1,NY-ny+1),-np.inf)
    for a,b in zip(*np.where(colany)):
        out=np.maximum(out,ctop[a,b]-cz[a:a+NX-nx+1,b:b+NY-ny+1])
    return out
def best_for(boxes,yaws=range(0,360,10),label=''):
    best=[]
    for yaw in yaws:
        R=rotz(np.radians(yaw)); mk,mo=mask(R,boxes=boxes)
        ok=feasible(blocked_relaxed,mk)
        if not ok.any(): continue
        ex=excess_map(mk)
        kk=np.where(ok.any(2),ok.argmax(2),10**6)
        Z=np.where(kk<10**6,zs[np.minimum(kk,len(zs)-1)],np.inf); tot=Z+ex
        i,j=np.unravel_index(np.argmin(tot),tot.shape)
        best.append((float(tot[i,j]),yaw,int(i),int(j),int(kk[i,j]),mo))
    best.sort(key=lambda r:r[0]); return best
if __name__=='__main__':
    full=dict(BOXES)
    variants={'all':full,'no usb plug/sd keepout':{k:v for k,v in full.items() if k not in('usb_plug','sd_card')},
     'no jumper zone (header only)':{k:v for k,v in full.items() if k not in('jumper_zone','usb_plug','sd_card')},
     'pcb+top only':{k:v for k,v in full.items() if k in('tray','posts+pcb','top_parts')}}
    for nm,bx in variants.items():
        b=best_for(bx)
        print(nm,'->',[(round(r[0],1),r[1]) for r in b[:6]],flush=True)
