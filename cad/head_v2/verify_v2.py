import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from solids import *
from pathtools import tube
from plan_grid import mesh_grid
from ribbon import ribbon_path
import trimesh
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
W=lambda f: trimesh.load(OUT2+f)
serv=head_servos_world()
static={'shell_v2':W('head_shell_world_v2.stl'),'head_v2':W('head_world_v2.stl'),'support_v2':W('support_world_v2.stl'),'eye_shell_v2':W('eye_shell_world_v2.stl'),'eye_cam':W('eye_cam_world.stl'),
        'S_front':serv['jaw_soft_31'],'S_rear':serv['jaw_soft_-6']}
def piparts():
    ms={}
    ms['pi_pcb']=sbox(*PCB); ms['pi_top']=sbox(0,65,0,30,PCB_T,TOP); ms['pi_under']=sbox(0,65,0,30,0,UB)
    ms['pi_header+jumper_zone']=sbox(*JUMP); ms['pi_header_rest']=sbox(*HDR_REST); ms['usb_plug']=sbox(*USB_PLUG); ms['sd_card']=sbox(*SD_KEEP)
    return ms
usb=np.load(OUT2+'path_usb.npy'); wires=np.load(OUT2+'path_wires.npy')
items=piparts()
items['tray']=W('tray_world.stl')
items['tray_spacers']=trimesh.util.concatenate([build_spacer().copy().apply_translation([x,y,TAB_Z+0.1]) for (x,y) in PILLARS])
items['camera_mount']=W('camera_mount_world.stl'); items['camera_pcb+lens']=W('camera_dummy_world.stl')
items['usb_cable']=tube(usb[np.linalg.norm(usb-usb[0],axis=1)>2.5],1.75)
items['wire_bundle']=tube(wires[np.linalg.norm(wires-wires[0],axis=1)>2.5],1.9)
rp=ribbon_path(); items['ribbon_corridor']=tube(rp[8:],0.2)
res={}
print('--- static overlaps (mm3) ---')
for iname,im in items.items():
    row={}
    for sname,sm in static.items():
        try:
            if not im.is_volume: 
                # tubes are open: use sampled-point containment instead
                pts=im.sample(4000); row[sname]=int(sm.contains(pts).sum())
            else:
                x=trimesh.boolean.intersection([im,sm],engine='manifold'); row[sname]=round(float(x.volume),2) if not x.is_empty else 0.0
        except Exception as e: row[sname]='err'
    res[iname]=row
    bad={k:v for k,v in row.items() if v not in (0,0.0)}
    print(iname.ljust(24),'OK' if not bad else bad)
print('--- swept obstacles (yaw/roll group over +-25 deg, jaw 0..30 deg): voxel overlap counts, 1mm/0.5mm voxels ---')
for iname,im in items.items():
    try:
        if im.is_volume: g=mesh_grid(im)
        else:
            g=np.zeros_like(D['jaw'])
            for p in im.sample(6000):
                i=int(round(p[0]-xs[0])); j=int(round(p[1]-ys[0])); k=int(round((p[2]-zs[0])/0.5))
                if 0<=i<g.shape[0] and 0<=j<g.shape[1] and 0<=k<g.shape[2]: g[i,j,k]=True
        print(iname.ljust(24),'jaw sweep:',int((g&D['jaw']).sum()),' roll sweep (S_yaw,head_yaw):',int((g&D['yawg']).sum()))
    except Exception as e: print(iname,'err',e)
