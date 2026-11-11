import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from design import *
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
from occ import build_grid
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
def lift_map(need,grow=13,sigma=3.0,fade_x=(86.0,90.0)):
    base=ndi.maximum_filter(need,size=grow)
    Dm=ndi.gaussian_filter(base,sigma=sigma); Dm=np.maximum(Dm,need)
    Dm=np.where(Dm<0.15,0,Dm)
    # never touch the eye / face-plate zone
    fx=np.clip((fade_x[1]-xs)/(fade_x[1]-fade_x[0]),0,1)[:,None]
    return Dm*np.maximum(fx,0) if False else Dm
def warp_shell(shell_world,Dm):
    v=shell_world.vertices.copy()
    wx=np.clip((v[:,0]-84.0)/(89.0-84.0),0,1)
    zl=234.0+(253.0-234.0)*wx; zh=246.0+(257.0-246.0)*wx
    ramp=np.clip((v[:,2]-zl)/(zh-zl),0,1); ramp=ramp*ramp*(3-2*ramp)
    lift=map_coordinates(Dm,[v[:,0]-xs[0],v[:,1]-ys[0]],order=1,mode='nearest')
    v2=v.copy(); v2[:,2]+=lift*ramp
    return trimesh.Trimesh(v2,shell_world.faces.copy(),process=False)
def ceil_of(shell_mesh):
    xs_,ys_,zs_,G=build_grid({'s':shell_mesh})
    sh=G['s']; nz=len(zs_); has=sh.any(2); first=np.where(has,sh.argmax(2),nz)
    return np.where(has,zs_[np.minimum(first,nz-1)],np.nan)
if __name__=='__main__':
    N=np.load(OUT2+'need_pi.npz'); Dm=lift_map(N['need'])
    sh=load_part_world('head_shell.stl'); new=warp_shell(sh,Dm)
    print('max lift',Dm.max(),'bounds',new.bounds.round(1).tolist(),'watertight',new.is_watertight)
    new.export(OUT2+'shell_v0_world.stl'); np.save(OUT2+'Dm_v0.npy',Dm)
