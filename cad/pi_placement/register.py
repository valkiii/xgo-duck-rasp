import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
import itertools, json
from scipy.spatial import cKDTree
def rot24():
    out=[]
    for perm in itertools.permutations(range(3)):
        for s in itertools.product([1,-1],repeat=3):
            M=np.zeros((3,3))
            for i,p in enumerate(perm): M[i,p]=s[i]
            if np.linalg.det(M)>0: out.append(M)
    return out
def register(src, tgt, nsamp=6000):
    """find rigid T s.t. T@src ~ tgt (both trimesh, same units mm)"""
    best=None
    pt=tgt.sample(nsamp); tree=cKDTree(pt)
    for M in rot24():
        ext=(np.abs(M)@src.extents)
        if np.abs(ext-tgt.extents).max()>2.0: continue
        T=np.eye(4); T[:3,:3]=M
        c=M@src.bounds.mean(0)  # bounds center
        T[:3,3]=tgt.bounds.mean(0)-c
        s=src.sample(nsamp)
        # ICP
        for it in range(30):
            sp=(T[:3,:3]@s.T).T+T[:3,3]
            dist,idx=tree.query(sp)
            A=sp; B=pt[idx]
            ca=A.mean(0); cb=B.mean(0)
            H=(A-ca).T@(B-cb); U,S,Vt=np.linalg.svd(H); Rr=Vt.T@U.T
            if np.linalg.det(Rr)<0: Vt[-1]*=-1; Rr=Vt.T@U.T
            dT=np.eye(4); dT[:3,:3]=Rr; dT[:3,3]=cb-Rr@ca
            T=dT@T
        sp=(T[:3,:3]@s.T).T+T[:3,3]
        dist,_=tree.query(sp); err=np.sqrt((dist**2).mean())
        if best is None or err<best[0]: best=(err,T)
    return best
if __name__=='__main__':
    m,d=load(); set_pose(m,d,'INIT')
    wm=world_meshes(m,d)
    comps=[]
    for b in ['jaw_soft','yaw_roll_motion','neck_pitch','neck']:
        for n,me in wm[b]:
            for p in me.split(only_watertight=False): comps.append((b,p))
    names=['head.stl','head_shell.stl','jaw.stl','eye_shell.stl','head_servo_support.stl','head_yaw.STL','head_pitch.stl','eye.stl']
    res={}
    for nm in names:
        src=trimesh.load(STRUCT+nm)
        best=None
        for b,p in comps:
            if np.abs(p.extents-src.extents).max()>3 and not (np.abs(np.sort(p.extents)-np.sort(src.extents)).max()<3): continue
            r=register(src,p)
            if r and (best is None or r[0]<best[0]): best=(r[0],r[1],b,p.bounds.round(1).tolist())
        print(nm, None if best is None else (round(best[0],3),best[2],best[3]))
        if best: res[nm]={'T':best[1].tolist(),'rms':best[0],'body':best[2]}
    json.dump(res,open(OUT+'registration_init.json','w'),indent=1)
