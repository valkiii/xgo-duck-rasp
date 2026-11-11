import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from solids import *
from pathtools import tube
from ribbon import ribbon_path
from grid2 import roty
import matplotlib.patches as mp
W=lambda f: trimesh.load(OUT2+f)
serv=head_servos_world()
shell=W('head_shell_world_v2.stl'); old=load_part_world('head_shell.stl')
head=W('head_world_v2.stl'); sup=W('support_world_v2.stl'); es=W('eye_shell_world_v2.stl'); eyec=W('eye_cam_world.stl')
jaw=load_part_world('jaw.stl'); hyaw=load_part_world('head_yaw.STL')
tray=W('tray_world.stl'); cm=W('camera_mount_world.stl'); cam=W('camera_dummy_world.stl')
spac=trimesh.util.concatenate([build_spacer().copy().apply_translation([x,y,TAB_Z+0.1]) for (x,y) in PILLARS])
pi=trimesh.util.concatenate([sbox(*PCB),sbox(0,65,0,30,PCB_T,TOP),sbox(0,65,0,30,0,UB)])
hdr=trimesh.util.concatenate([sbox(*HDR)]+[sbox(HDR_X0+2.54*n,HDR_X0+2.54*n+2.4,22.8,27.4,PCB_T+8.5,PCB_T+8.5+12.0) for n in range(9)])  # 7-9 dupont housings (reserved 21 mm)
plug=sbox(*USB_PLUG)
usb=np.load(OUT2+'path_usb.npy'); wires=np.load(OUT2+'path_wires.npy'); rp=ribbon_path()
usbm=tube(usb[np.linalg.norm(usb-usb[0],axis=1)>0.5],1.75); wirm=tube(wires,1.9); ribm=tube(rp,0.4)
COL={'shell':'#2060c0','old':'#999999','head':'#8888ff','sup':'#aa6633','es':'#ff9900','eye':'#ff5500','servo':'#cc0000','jaw':'#22aa22','tray':'#00aaaa','pi':'#006600','hdr':'#33aa33','plug':'#aa00aa','usb':'#000000','wires':'#dd00dd','rib':'#ee8800','cam':'#cc00cc','cm':'#00cccc','yaw':'#ff66ff'}
def items(jawm=None):
    return [('old',old,COL['old']),('shell',shell,COL['shell']),('head',head,COL['head']),('sup',sup,COL['sup']),('es',es,COL['es']),('eye',eyec,COL['eye']),
        ('Sf',serv['jaw_soft_31'],COL['servo']),('Sr',serv['jaw_soft_-6'],COL['servo']),('Sy',serv['yaw_roll_motion_31'],COL['yaw']),('hy',hyaw,COL['yaw']),
        ('jaw',jawm if jawm is not None else jaw,COL['jaw']),('tray',tray,COL['tray']),('spac',spac,COL['tray']),('pi',pi,COL['pi']),('hdr',hdr,COL['hdr']),('plug',plug,COL['plug']),
        ('usb',usbm,COL['usb']),('w',wirm,COL['wires']),('rib',ribm,COL['rib']),('cam',cam,COL['cam']),('cm',cm,COL['cm'])]
ex=np.array([1,0,0]);ey=np.array([0,1,0]);ez=np.array([0,0,1])
def sect(ax,it,o,n,uv,xl,yl,lw=1.0):
    for nm,m,c in it:
        try: s=m.section(plane_origin=o,plane_normal=n)
        except Exception: s=None
        if s is None: continue
        for e in s.entities:
            p=s.vertices[e.points]; ax.plot(p@uv[0],p@uv[1],color=c,lw=lw*(1.6 if nm in('usb','w','rib') else 1))
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect('equal'); ax.grid(alpha=.25)
it=items()
# ---- XZ sections
fig,axs=plt.subplots(2,3,figsize=(30,15))
for ax,y in zip(axs.ravel(),[-8,-14,4,0,18,27.5]):
    sect(ax,it,[0,y,0],ey,(ex,ez),(-22,108),(205,275)); ax.set_title(f'XZ section y={y}  (blue=new shell, grey=old shell)')
fig.suptitle('Head v2: side sections. Pi Zero 2W (green), header+Dupont zone (light green), tray (teal), USB plug (purple) + cable (black), wire bundle (magenta), CSI ribbon (orange), camera (pink) + mount (cyan)',fontsize=13)
plt.tight_layout(); plt.savefig('b1_sections_xz.png',dpi=45); plt.close()
# ---- YZ sections
fig,axs=plt.subplots(2,3,figsize=(30,15))
for ax,x in zip(axs.ravel(),[18,27.5,52,64,80,86.5]):
    sect(ax,it,[x,0,0],ex,(ey,ez),(-46,46),(205,275)); ax.set_title(f'YZ section x={x}')
plt.tight_layout(); plt.savefig('b2_sections_yz.png',dpi=45); plt.close()
# ---- plan sections
fig,axs=plt.subplots(2,3,figsize=(33,15))
for ax,z in zip(axs.ravel(),[222,236,239.5,246.8,250.5,256]):
    sect(ax,it,[0,0,z],ez,(ex,ey),(-22,108),(-46,46)); ax.set_title(f'plan z={z}')
plt.tight_layout(); plt.savefig('b3_sections_plan.png',dpi=42); plt.close()
# ---- jaw sweep
fig,axs=plt.subplots(2,3,figsize=(30,14))
cols=['#22aa22','#88cc22','#ccaa00','#ee6600']
for ax,(y,xl) in zip(axs[0],[(-38,(25,60)),(0,(-22,108)),(38,(25,60))]):
    for th,cc in zip([0,10,20,30],cols):
        jm=jaw.copy(); jm.apply_transform(roty(np.radians(th),41,225.4))
        sect(ax,[('jaw',jm,cc)],[0,y,0],ey,(ex,ez),xl,(200,275),lw=1.4)
    sect(ax,[i for i in it if i[0]!='jaw'],[0,y,0],ey,(ex,ez),xl,(200,275),lw=0.8); ax.set_title(f'jaw opening 0/10/20/30 deg (green->orange), XZ y={y}')
# jaw sweep voxels vs everything: bar chart of min clearance
from scipy import ndimage as ndi
D=np.load(OUT2+'grid2.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
from plan_grid import mesh_grid
new_parts={'Pi (all keep-outs)':None}
objs={'tray':tray,'spacers':spac,'camera mount':cm,'camera pcb+lens':cam,'USB cable':usbm,'wire bundle':wirm,'ribbon':ribm,'Pi envelope':trimesh.util.concatenate([pi,hdr,plug])}
jg=D['jaw']; edt=ndi.distance_transform_edt(~jg,sampling=(1,1,0.5))
names=[];vals=[]
for n,m in objs.items():
    if m.is_volume: g=mesh_grid(m)
    else:
        g=np.zeros_like(jg)
        for p in m.sample(8000):
            i=int(round(p[0]-xs[0])); j=int(round(p[1]-ys[0])); k=int(round((p[2]-zs[0])/0.5))
            if 0<=i<g.shape[0] and 0<=j<g.shape[1] and 0<=k<g.shape[2]: g[i,j,k]=True
    d=edt[g].min() if g.any() else np.nan; names.append(n); vals.append(d)
ax=axs[1][0]; ax.barh(names,vals,color='#22aa22'); ax.set_xlabel('min distance to the union of the jaw sweep 0-30 deg [mm]'); ax.set_title('jaw sweep clearance (new parts)')
for i,v in enumerate(vals): ax.text(v+0.3,i,f'{v:.1f}',va='center')
# head roll sweep clearance
yg=D['yawg']; edt2=ndi.distance_transform_edt(~yg,sampling=(1,1,0.5)); vals2=[]
for n,m in objs.items():
    if m.is_volume: g=mesh_grid(m)
    else:
        g=np.zeros_like(jg)
        for p in m.sample(8000):
            i=int(round(p[0]-xs[0])); j=int(round(p[1]-ys[0])); k=int(round((p[2]-zs[0])/0.5))
            if 0<=i<g.shape[0] and 0<=j<g.shape[1] and 0<=k<g.shape[2]: g[i,j,k]=True
    vals2.append(edt2[g].min())
ax=axs[1][1]; ax.barh(names,vals2,color='#cc44cc'); ax.set_xlabel('min distance to the yaw-servo/bracket swept by head roll +-25 deg [mm]'); ax.set_title('roll sweep clearance (new parts)')
for i,v in enumerate(vals2): ax.text(v+0.3,i,f'{v:.1f}',va='center')
axs[1][2].axis('off')
plt.tight_layout(); plt.savefig('b4_jaw_roll_sweep_check.png',dpi=45); plt.close()
print('jaw clearances',dict(zip(names,np.round(vals,1))),'roll',dict(zip(names,np.round(vals2,1))))
