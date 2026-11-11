import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
from scipy.signal import fftconvolve
S=np.load(OUT+'sweeps_stand.npz'); trunk=S['trunk']
m,d=load()
def gl(names):
    ids={m.body(n).id for n in names}
    return [g for g in range(m.ngeom) if m.geom_group[g]==2 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_bodyid[g] in ids]
hg=gl(['neck','neck_pitch','yaw_roll_motion','jaw_soft'])
box=(66,31,29); clr=1.0
shp=tuple(int(np.ceil(p+2*clr)) for p in box)
out={}
for thb in [5,10,15,20,25]:
    for tag,hp,hy,hr in [('level head only',0,0,0),('+-25 head pitch, yaw +-60, roll +-25',25,60,25)]:
        r={'neck_pitch':(np.radians(-30),np.radians(thb))}
        # head pitch sampled relative -> sample absolute in [neck-hp, neck+hp] approx: use range [-30-hp, thb+hp]
        r['head_pitch']=(np.radians(-30-hp),np.radians(thb+hp)); r['head_yaw']=(np.radians(-hy),np.radians(hy)); r['head_roll']=(np.radians(-hr),np.radians(hr))
        g=sweep(m,d,hg,r,nsamp=2500,ncfg=2500)
        g=ndi.binary_fill_holes(ndi.binary_dilation(g,iterations=1))
        obst=trunk|g
        # zone top
        x0,x1,y0,y1,z0,z1=(-60,34,-30,30,140,215)
        ix0=int(x0-clr-GX[0]); ix1=int(x1+clr-GX[0]); iy0=int(y0-clr-GY[0]); iy1=int(y1+clr-GY[0]); iz0=int(z0-clr-GZ[0]); iz1=int(z1+clr-GZ[0])
        sub=obst[ix0:ix1+1,iy0:iy1+1,iz0:iz1+1]
        cor=fftconvolve(sub.astype(np.float32),np.ones(shp,np.float32),mode='valid'); free=cor<0.5
        ii,jj,kk=np.where(free); X0=ii+ix0+GX[0]+clr; Y0=jj+iy0+GY[0]+clr; Z0=kk+iz0+GZ[0]+clr
        # require resting on torso: z0 <= 160
        sel=(Z0<=162)&(abs(Y0+15.5)<=8)
        print(thb,tag,'free resting boxes',sel.sum(), (X0[sel].min(),X0[sel].max(),Z0[sel].min()) if sel.sum() else '',flush=True)
