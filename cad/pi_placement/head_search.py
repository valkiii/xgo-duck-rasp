import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from stackfit import *
import pickle
D=np.load(OUT+'head_grid.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
shell=D['head_shell_stl']
others=np.zeros_like(shell)
for n in ['head_stl','jaw_stl','eye_shell_stl','head_servo_support_stl','head_yaw_STL','servo0_jaw_soft','servo1_jaw_soft','servo3_yaw_roll_motion']:
    others|=D[n]
nz=len(zs)
has=shell.any(2)
first=np.where(has, shell.argmax(2), nz)       # first shell cell index per column
K=np.arange(nz)[None,None,:]
cavity=(K<first[:,:,None])&has[:,:,None]
ceil_z=np.where(has, zs[np.minimum(first,nz-1)], np.nan)   # inner surface of dome (z of first shell cell)
blocked_hard=others|~cavity
np.savez_compressed(OUT+'head_blocked.npz',blocked=blocked_hard,ceil_z=ceil_z,others=others,cavity=cavity)
def run(blocked, label, clr=1.0, yaws=range(0,360,15)):
    results=[]
    for uname,R0 in UPS.items():
        for yaw in yaws:
            R=rotz(np.radians(yaw))@R0
            mk,mo=stack_mask(R,clr=clr)
            ok=feasible(blocked,None,mk,mo)
            n=ok.sum()
            if n: results.append((uname,yaw,n,ok,mo))
            print(label,uname,yaw,n,flush=True)
    return results
if __name__=='__main__':
    res=run(blocked_hard,'hard')
    pickle.dump([(u,y,n,np.argwhere(ok)) for u,y,n,ok,mo in res],open(OUT+'head_hard_results.pkl','wb'))
