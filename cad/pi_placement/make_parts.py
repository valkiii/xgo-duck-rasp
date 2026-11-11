import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_final import *
import json
res=json.load(open(OUT+'registration_init.json'))
def tf(name): return np.array(res[name]['T'])
new=trimesh.load(OUT+'head_shell_world_raised.stl')
# port window: stack ports edge at world y = cy + 15 ; stack X in [3,62] -> world x = cx+32.5-X
cy_edge=CY+15.0
win_x0=CX+32.5-62; win_x1=CX+32.5-3
win=trimesh.creation.box(extents=[win_x1-win_x0, 40.0, 16.0]); win.apply_translation([(win_x0+win_x1)/2, cy_edge-1.0+20.0, Zc+13.5])
# camera: eye centre
ey_c=np.array([0,0,99.09+138.0])
shell_cut=trimesh.boolean.difference([new,win],engine='manifold')
print('shell cut watertight',shell_cut.is_watertight,'vol',shell_cut.volume/1000)
shell_cut.export(OUT+'head_shell_world_pi.stl')
p=shell_cut.copy(); p.apply_transform(np.linalg.inv(tf('head_shell.stl'))); p.export(OUT+'head_shell_pi.stl')
print('exported head_shell_pi.stl (printable frame)', p.bounds.round(1).tolist())
# eye with lens hole
eye=trimesh.load(STRUCT+'eye.stl'); print('eye',eye.bounds.round(1).tolist())
Te=tf('eye_shell.stl')
# eye axis in part frame is y_c; centre x_c=0,z_c=-138
cyl=trimesh.creation.cylinder(radius=4.6,height=30,sections=64)
# cylinder axis z -> y
Rm=np.array([[1,0,0],[0,0,-1],[0,1,0]],float); T=np.eye(4); T[:3,:3]=Rm; T[:3,3]=[0,105,-138]
cyl.apply_transform(T)
eye2=trimesh.boolean.difference([eye,cyl],engine='manifold')
print('eye_cam watertight',eye2.is_watertight,eye.volume,eye2.volume)
eye2.export(OUT+'eye_cam.stl')
# diagnostics: in world frame where are things
w=win.copy(); print('port window world bounds',w.bounds.round(1).tolist())
# in the shell printable frame
w2=win.copy(); w2.apply_transform(np.linalg.inv(tf('head_shell.stl'))); print('port window in head_shell.stl frame',w2.bounds.round(1).tolist())
e=eye.copy(); e.apply_transform(Te); print('eye in world',e.bounds.round(1).tolist())
json.dump({'stack_centre_world':centre.tolist(),'yaw_deg':YAW,'port_window_world_bounds':w.bounds.tolist(),'port_window_shell_frame_bounds':w2.bounds.tolist()},open(OUT+'head_pose.json','w'),indent=1)
