import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2'); sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common2 import *
from occ import build_grid
import piraise as P
from piraise2 import H
from scipy import ndimage as ndi
D=np.load(OUT2+'grid2.npz')
serv=head_servos_world(); hy=load_part_world('head_yaw.STL')
base=D['head']|D['eyeshell']|D['support']|D['Srear']|D['Sfront']|D['jaw']
def yaw_group(lim):
    meshes={}
    for a in np.arange(-lim,lim+0.1,5 if lim>5 else max(lim,1)):
        T=rotx_about(np.radians(a),0,222.6)
        meshes[f'S{a}']=serv['yaw_roll_motion_31'].copy().apply_transform(T); meshes[f'h{a}']=hy.copy().apply_transform(T)
    xs,ys,zs,G=build_grid(meshes)
    g=np.zeros_like(D['head'])
    for v in G.values(): g|=v
    return g
for lim in [0,10,15,25]:
    yg=yaw_group(lim) if lim<25 else D['yawg']
    others=base|yg
    has=P.has; K=P.zs[None,None,:]
    blk=others|P.rim_d[:,:,None]|(K<212)
    free=~blk; lab,n=ndi.label(free); seed=(np.argmin(abs(P.xs-80)),np.argmin(abs(P.ys-0)),np.argmin(abs(P.zs-245)))
    P.blocked_relaxed=~(lab==lab[seed])
    for nm,yaws in [('180',[170,180,190]),('0',[-10,0,10])]:
        b=P.best_for(H,yaws=yaws)
        print('roll +-%d'%lim,'yaw~',nm,[(round(r[0],1),r[1]) for r in b[:3]],flush=True)
