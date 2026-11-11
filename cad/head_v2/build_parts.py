import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from solids import *
from shellwarp import *
from route2 import CUT_FLANGE,CUT_GATE
import json
def Tinv(name): return np.linalg.inv(T_of(name))
def to_part(m,name):
    m=m.copy(); m.apply_transform(Tinv(name)); return m
def wbox(x0,x1,y0,y1,z0,z1): return xbox(x0,x1,y0,y1,z0,z1)
LY,LZ=LENS
# ---------------- shell ----------------
N=np.load(OUT2+'need_all.npz'); Dm=lift_map(N['need'])
sh=load_part_world('head_shell.stl'); new=warp_shell(sh,Dm)
cols=[xcyl(y,z,PLATE_FRONT,93.4,2.8) for (y,z) in COLUMNS]
new=union([new]+cols)
cuts=[xcyl(LY,LZ,88.5,96.6,6.5)]                                  # lens bore through the membrane (Ø13)
cuts+=[xcyl(y,z,CAM_FRONT,CAM_FRONT+1.2,2.2) for (y,z) in CAM_HOLES]   # screw-head pockets
cuts+=[xcyl(y,z,PLATE_FRONT-0.1,PLATE_FRONT+6.0,0.85) for (y,z) in COLUMNS]          # M2 pilots Ø1.7 x 6
new=diff(new,cuts)
print('shell watertight',new.is_watertight,'vol',new.volume/1000,'bounds',new.bounds.round(1).tolist())
new.export(OUT2+'head_shell_world_v2.stl'); to_part(new,'head_shell.stl').export(OUT2+'head_shell_v2.stl')
np.save(OUT2+'Dm_final.npy',Dm)
# ---------------- eye shell ----------------
es=load_part_world('eye_shell.stl')
es2=diff(es,[xcyl(LY,LZ,91.0,96.3,6.3)])
print('eye_shell watertight',es2.is_watertight)
to_part(es2,'eye_shell.stl').export(OUT2+'eye_shell_v2.stl'); es2.export(OUT2+'eye_shell_world_v2.stl')
# ---------------- eye (cap only + lens hole + countersink) ----------------
eye=trimesh.load(STRUCT+'eye.stl'); eye.apply_transform(T_of('eye_shell.stl'))        # world
capcut=xbox(80,96.2,-30,30,200,280)
e2=diff(eye,[capcut])
# countersink profile (r, x)  in world x
prof=np.array([[0,95.5],[3.75,95.5],[3.75,97.6],[6.2,99.2],[0,99.2]])
cs=trimesh.creation.revolve(prof)   # revolve around z of (r,z) -> axis z; map to x axis
cs.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))   # z -> x  (z axis now along +x)
cs.apply_translation([0,LY,LZ])
# after rotation about y by +90deg: z->x, x->-z ; profile z coordinate becomes x coordinate. fine
e2=diff(e2,[cs])
print('eye_cam watertight',e2.is_watertight,e2.volume,'bounds',e2.bounds.round(1).tolist())
to_part(e2,'eye_shell.stl').export(OUT2+'eye_cam.stl'); e2.export(OUT2+'eye_cam_world.stl')
# ---------------- support plate ----------------
sup=load_part_world('head_servo_support.stl')
gate_s=xbox(14.5,22.5,14.2,19.8,217.5,221.0)           # tab slot
fl=xbox(*CUT_FLANGE[:2],*CUT_FLANGE[2:4],*CUT_FLANGE[4:])
sup2=diff(sup,[gate_s,fl])
print('support watertight',sup2.is_watertight,sup.volume-sup2.volume)
to_part(sup2,'head_servo_support.stl').export(OUT2+'head_servo_support_v2.stl'); sup2.export(OUT2+'support_world_v2.stl')
# ---------------- head.stl ----------------
hd=load_part_world('head.stl')
gate_h=xbox(14.5,22.5,14.2,19.8,203.0,219.0)
hd2=diff(hd,[gate_h]); print('head watertight',hd2.is_watertight,hd.volume-hd2.volume)
to_part(hd2,'head.stl').export(OUT2+'head_v2.stl'); hd2.export(OUT2+'head_world_v2.stl')
# ---------------- tray, spacer, camera mount ----------------
tray=build_tray(); print('tray',tray.is_watertight,tray.volume/1000)
tray.export(OUT2+'tray_world.stl'); tray.export(OUT2+'pi_tray.stl')          # modelled printing orientation: beams flat on the bed, posts up
sp=build_spacer(); sp.export(OUT2+'tray_spacer.stl'); print('spacer',sp.is_watertight,sp.bounds.round(1).tolist())
cm=build_camera_mount(); cm.export(OUT2+'camera_mount_world.stl')
cp=cm.copy(); cp.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]))   # x -> z? check below
print('cm bounds before',cm.bounds.round(1).tolist())
cp.apply_translation(-cp.bounds[0]); cp.export(OUT2+'camera_mount.stl'); print('camera_mount print bounds',cp.bounds.round(1).tolist(),cp.is_watertight)
