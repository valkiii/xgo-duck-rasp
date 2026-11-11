import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from design import *
from solids import *
from envelope import voxelize_boxes, pi_envelope_boxes
from occ import build_grid
from scipy import ndimage as ndi
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
def mesh_grid(m):
    _,_,_,G=build_grid({'m':m}); return G['m']
def blocked_for_planning(shell_world,extra_meshes=(),clr_pi=0.5):
    sh=mesh_grid(shell_world)
    nz=len(zs); has=sh.any(2); first=np.where(has,sh.argmax(2),nz)
    K=np.arange(nz)[None,None,:]
    cavity=(K<first[:,:,None])&has[:,:,None]
    occ=D['head']|D['eyeshell']|D['support']|D['Srear']|D['Sfront']|D['jaw']|D['yawg']
    occ|=~cavity
    occ[:,:,zs<211]=True
    for m in extra_meshes: occ|=mesh_grid(m)
    occ|=voxelize_boxes(pi_envelope_boxes(),clr=clr_pi)
    return occ
if __name__=='__main__':
    shell=trimesh.load(OUT2+'shell_v0_world.stl')
    extras=[trimesh.load(OUT2+'tray_world.stl'),trimesh.load(OUT2+'camera_mount_world.stl'),trimesh.load(OUT2+'camera_dummy_world.stl')]
    occ=blocked_for_planning(shell,extras)
    np.save(OUT2+'plan_occ_v0.npy',occ)
    print('occ voxels',occ.sum())
