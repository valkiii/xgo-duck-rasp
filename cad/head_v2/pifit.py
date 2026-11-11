import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2'); sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
import numpy as np
from scipy.signal import fftconvolve
from scipy import ndimage as ndi
# stack frame: X 0..65 (SD end ->CSI end), Y 0 (USB edge) .. 30 (header edge), Z 0 = tray bottom
TRAY_T=2.0; STANDOFF=4.0
PCB_B=TRAY_T+STANDOFF; PCB_T=PCB_B+1.4
BOXES={
 'tray':(-2,67,-2,32,0,TRAY_T),
 'posts+pcb':(0,65,0,30,TRAY_T,PCB_T),
 'top_parts':(0,65,0,30,PCB_T,PCB_T+3.3),
 'header_rest':(29.5,58.5,22.5,29.4,PCB_T,PCB_T+10.0),
 'jumper_zone':(6.0,29.5,22.5,29.4,PCB_T,PCB_T+21.0),
 'usb_plug':(35.4,47.4,-11,0,PCB_B-2,PCB_B+7),
 'sd_card':(-3,0,9,23,PCB_B-2,PCB_T+1),
}
C=np.array([32.5,15.0,0.0])
def rotz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def mask(R,clr=0.8,px=1.0,pz=0.5,boxes=None):
    boxes=boxes or BOXES
    allc=np.array([[b[0],b[2],b[4]] for b in boxes.values()]+[[b[1],b[3],b[5]] for b in boxes.values()])
    wc=(R@(np.array([[x,y,z] for x in (allc[:,0].min(),allc[:,0].max()) for y in (allc[:,1].min(),allc[:,1].max()) for z in (allc[:,2].min(),allc[:,2].max())])-C).T).T
    lo=np.floor(wc.min(0)-clr-1); hi=np.ceil(wc.max(0)+clr+1)
    xs=np.arange(lo[0],hi[0]+1e-6,px); ys=np.arange(lo[1],hi[1]+1e-6,px); zs=np.arange(lo[2],hi[2]+1e-6,pz)
    X,Y,Z=np.meshgrid(xs,ys,zs,indexing='ij'); P=np.c_[X.ravel(),Y.ravel(),Z.ravel()]
    Q=(R.T@P.T).T+C
    ins=np.zeros(len(P),bool)
    for (x0,x1,y0,y1,z0,z1) in boxes.values():
        ins|=((Q[:,0]>=x0-clr)&(Q[:,0]<=x1+clr)&(Q[:,1]>=y0-clr)&(Q[:,1]<=y1+clr)&(Q[:,2]>=z0-clr)&(Q[:,2]<=z1+clr))
    return ins.reshape(len(xs),len(ys),len(zs)),(xs[0],ys[0],zs[0])
def feasible(blocked,mk):
    return fftconvolve(blocked.astype(np.float32),mk[::-1,::-1,::-1].astype(np.float32),mode='valid')<0.5
