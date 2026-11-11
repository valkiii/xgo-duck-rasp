import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_raise2 import *
D2=np.load(OUT+'head_grid.npz'); shell=D2['head_shell_stl']
def wall_dist_y(x,z,sign):
    """distance from y=ycoord... returns the y of the shell inner surface at (x,z) searching outward from y=0 in direction sign"""
    i=np.argmin(abs(xs-x)); k=np.argmin(abs(zs-z))
    col=shell[i,:,k]
    rng=range(len(ys)//2, len(ys)) if sign>0 else range(len(ys)//2,-1,-1)
    for j in rng:
        if col[j]: return ys[j]
    return None
res=[]
for yaw in [170,180,190,0,10,-10]:
    R=rotz(np.radians(yaw)); mk,mo=stack_mask(R)
    ok=feasible(blocked,None,mk,mo); ex=excess_map(mk,mo)
    kk=np.where(ok.any(2), ok.argmax(2), 10**6)
    Z=np.where(kk<10**6, zs[np.minimum(kk,len(zs)-1)] , np.inf)
    tot=Z+ex
    port_dir=R@np.array([0,-1,0])   # outward normal of ports edge in world
    for (i,j),v in np.ndenumerate(tot):
        if v>14: continue
        k=kk[i,j]
        centre=np.array([xs[i],ys[j],zs[k]])-np.array(mo)
        # port edge midpoint at stack-frame (32.5,0,12) 
        pe=centre+R@(np.array([32.5,0,12])-np.array([32.5,15,0]))
        # wall in direction of port_dir (only y-ish handled)
        if abs(port_dir[1])>0.9:
            wy=wall_dist_y(pe[0],pe[2],np.sign(port_dir[1]))
            gap=None if wy is None else (wy-pe[1])*np.sign(port_dir[1])
        else: gap=None
        res.append((yaw,round(v,1),round(centre[0],1),round(centre[1],1),round(centre[2],1),gap))
import collections
# best raise per gap bucket
res=[r for r in res if r[5] is not None]
res.sort(key=lambda r:(r[1]+0.5*max(r[5],0)))
for r in res[:15]: print(r)
print('--- gap<=6')
for r in sorted([r for r in res if r[5]<=6],key=lambda r:r[1])[:10]: print(r)
