import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_final import *
from occ import column_intervals
import scipy.ndimage as ndi, json
res=json.load(open(OUT+'registration_init.json'))
Ts=np.array(res['head_shell.stl']['T']); Tinv=np.linalg.inv(Ts)
shell_part=trimesh.load(STRUCT+'head_shell.stl')
sw=shell_part.copy(); sw.apply_transform(Ts)
N=np.load(OUT+'head_need.npz'); need=N['need']
base=ndi.maximum_filter(need,size=13)
D=ndi.gaussian_filter(base,sigma=3.0)
D=np.maximum(D,need)
D=np.where(D<0.15,0,D)
print('max lift',D.max())
def lift_at(x,y):
    # bilinear sample of D
    fx=(x-xs[0]); fy=(y-ys[0])
    from scipy.ndimage import map_coordinates
    return map_coordinates(D,[fx,fy],order=1,mode='nearest')
v=sw.vertices.copy()
wx=np.clip((v[:,0]-84.0)/(89.0-84.0),0,1)   # 0 behind x=84, 1 in front of x=89 (eye / face plate zone)
zl=234.0+(253.0-234.0)*wx; zh=246.0+(257.0-246.0)*wx
ramp=np.clip((v[:,2]-zl)/(zh-zl),0,1); ramp=ramp*ramp*(3-2*ramp)
# vertices of the shell skirt/front wall: only lift where inside the hump area
dz=lift_at(v[:,0],v[:,1])*ramp
v2=v.copy(); v2[:,2]+=dz
new=trimesh.Trimesh(v2,sw.faces.copy(),process=False)
print('watertight',new.is_watertight,'volume cm3 old/new',sw.volume/1000,new.volume/1000)
print('bounds old',sw.bounds.round(1).tolist(),'new',new.bounds.round(1).tolist())
new.export(OUT+'head_shell_world_raised.stl')
# back to the printable frame
np_=new.copy(); np_.apply_transform(Tinv); np_.export(OUT+'head_shell_raised_nocuts.stl')
np.savez_compressed(OUT+'head_lift.npz',D=D)
