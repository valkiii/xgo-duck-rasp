import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2'); sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common2 import *
from occ import build_grid
def roty(a,x0,z0):
    c,s=np.cos(a),np.sin(a); R=np.array([[c,0,s],[0,1,0],[-s,0,c]]); T=np.eye(4); T[:3,:3]=R; p=np.array([x0,0,z0]); T[:3,3]=p-R@p; return T
if __name__=='__main__':
    serv=head_servos_world()
    P={n:load_part_world(n) for n in ['head.stl','head_shell.stl','jaw.stl','eye_shell.stl','head_servo_support.stl','head_yaw.STL']}
    meshes={'head':P['head.stl'],'shell':P['head_shell.stl'],'eyeshell':P['eye_shell.stl'],'support':P['head_servo_support.stl'],'Srear':serv['jaw_soft_-6'],'Sfront':serv['jaw_soft_31']}
    for th in range(0,31,5): meshes[f'jaw{th}']=P['jaw.stl'].copy().apply_transform(roty(np.radians(th),41,225.4))
    for a in range(-25,26,5):
        T=rotx_about(np.radians(a),0,222.6)
        meshes[f'Syaw{a}']=serv['yaw_roll_motion_31'].copy().apply_transform(T)
        meshes[f'hyaw{a}']=P['head_yaw.STL'].copy().apply_transform(T)
    xs,ys,zs,G=build_grid(meshes)
    jaw=np.zeros_like(G['head']); 
    for k in G:
        if k.startswith('jaw'): jaw|=G[k]
    yawg=np.zeros_like(G['head'])
    for k in G:
        if k.startswith('Syaw') or k.startswith('hyaw'): yawg|=G[k]
    np.savez_compressed(OUT2+'grid2.npz',xs=xs,ys=ys,zs=zs,head=G['head'],shell=G['shell'],eyeshell=G['eyeshell'],support=G['support'],Srear=G['Srear'],Sfront=G['Sfront'],jaw=jaw,yawg=yawg,jaw0=G['jaw0'],Syaw0=G['Syaw0'])
    print({k:int(v.sum()) for k,v in [('jaw',jaw),('yawg',yawg)]})
