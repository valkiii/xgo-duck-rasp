import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2'); sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common2 import *
from occ import build_grid
import piraise as P
from piraise2 import H
from scipy import ndimage as ndi
from pifit import *
D=np.load(OUT2+'grid2.npz')
serv=head_servos_world(); hy=load_part_world('head_yaw.STL')
base=D['head']|D['eyeshell']|D['support']|D['Srear']|D['Sfront']|D['jaw']
def yaw_group(lim):
    meshes={}
    for a in np.arange(-lim,lim+0.1,5):
        T=rotx_about(np.radians(a),0,222.6)
        meshes[f'S{a}']=serv['yaw_roll_motion_31'].copy().apply_transform(T); meshes[f'h{a}']=hy.copy().apply_transform(T)
    xs,ys,zs,G=build_grid(meshes); g=np.zeros_like(D['head'])
    for v in G.values(): g|=v
    return g
def blocked_for(yg):
    others=base|yg; K=P.zs[None,None,:]
    blk=others|P.rim_d[:,:,None]|(K<212)
    free=~blk; lab,n=ndi.label(free); seed=(np.argmin(abs(P.xs-80)),np.argmin(abs(P.ys-0)),np.argmin(abs(P.zs-245)))
    return ~(lab==lab[seed])
def raise_at(boxes,yaw,cx,cy,blocked):
    R=rotz(np.radians(yaw)); mk,mo=mask(R,boxes=boxes)
    P.blocked_relaxed=blocked
    ok=feasible(blocked,mk); ex=P.excess_map(mk)
    i=int(round(cx+mo[0]-P.xs[0])); j=int(round(cy+mo[1]-P.ys[0]))
    if not ok[i,j].any(): return None
    k=ok[i,j].argmax(); return P.zs[k]+ex[i,j], P.zs[k]-mo[2]
if __name__=='__main__':
    for lim in [0,10,15,25]:
        yg=yaw_group(lim) if lim<25 else D['yawg']; bl=blocked_for(yg)
        for (cx,cy) in [(41,-2),(41,0),(43,4),(43,8),(41,-4)]:
            r=raise_at(H,180,cx,cy,bl); print('roll+-%d'%lim,(cx,cy),r)
