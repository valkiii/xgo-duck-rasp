import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
from scipy.signal import fftconvolve
# ---- stack model (stack frame: X long 65, Y 30 (Y=0 = ports edge, Y=30 = header edge), Z up from LiPo bottom)
L,W=65.0,30.0
BOXES=[ # (x0,x1,y0,y1,z0,z1)
 (0,65,0,30,0,17.5),            # LiPo + PiSugar + Pi PCB
 (0,65,0,30,17.5,20.5),         # components above PCB
 (7,58,22.5,29.5,20.5,26.0),    # 2x20 header strip + pins (8.5 mm)
 (7,21,22.5,29.5,26.0,28.5),    # right-angle jumpers at one end (11 mm above PCB)
]
TRAY=1.5   # printed tray under LiPo
def stack_mask(R, clr=1.0, px=1.0, pz=0.5, tray=TRAY):
    """R: 3x3 maps stack-frame vector -> world vector.  Returns bool mask (nx,ny,nz) of voxels (pitch px,px,pz) the dilated stack occupies,
    and the offset (in voxels) of the stack-frame origin... we return mask and 'origin_world_min' = world coord of mask[0,0,0] centre relative to the stack centre point."""
    c=np.array([L/2,W/2,0.0])
    corners=np.array([[x,y,z] for x in (0,L) for y in (0,W) for z in (-tray,28.5)])
    wc=(R@(corners-c).T).T
    lo=np.floor(wc.min(0)-clr-1); hi=np.ceil(wc.max(0)+clr+1)
    xs=np.arange(lo[0],hi[0]+1e-6,px); ys=np.arange(lo[1],hi[1]+1e-6,px); zs=np.arange(lo[2],hi[2]+1e-6,pz)
    X,Y,Z=np.meshgrid(xs,ys,zs,indexing='ij')
    P=np.c_[X.ravel(),Y.ravel(),Z.ravel()]
    Q=(R.T@P.T).T+c
    inside=np.zeros(len(P),bool)
    bx=[(0,65,0,30,-tray,17.5)]+BOXES[1:]
    for (x0,x1,y0,y1,z0,z1) in bx:
        inside|=((Q[:,0]>=x0-clr)&(Q[:,0]<=x1+clr)&(Q[:,1]>=y0-clr)&(Q[:,1]<=y1+clr)&(Q[:,2]>=z0-clr)&(Q[:,2]<=z1+clr))
    return inside.reshape(len(xs),len(ys),len(zs)), (xs[0],ys[0],zs[0])
def rotz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
UPS={ # stack axis that points world +z -> rotation base
 'flat(Z up)': np.eye(3),
 'edge(+Y up)': np.array([[1,0,0],[0,0,-1],[0,1,0]],float),   # stack Y -> world z ; stack Z -> world -y
 'edge(-Y up)': np.array([[1,0,0],[0,0,1],[0,-1,0]],float),
}
def feasible(occ, grid_origin, mask, mask_origin):
    """occ bool grid (pitch 1,1,0.5 at grid_origin).  returns bool array over *mask-origin positions*: True if no overlap. index (i,j,k): mask[0,0,0] at grid cell (i,j,k)."""
    cor=fftconvolve(occ.astype(np.float32), mask[::-1,::-1,::-1].astype(np.float32), mode='valid')
    return cor<0.5
