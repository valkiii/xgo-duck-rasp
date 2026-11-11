import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_final import *
from viz import *
from headmodel import head_parts
parts,servos,m_,d_=head_parts()
new=trimesh.load(OUT+'head_shell_world_raised.stl')
ms_=[to_world(b)[0] for b in stack_world_mesh()]
stack=trimesh.util.concatenate(ms_)
tray=trimesh.creation.box(extents=[67,32,1.5]); tray.apply_translation([32.5,15,-0.75-1.0+0.0]); tray,_=to_world(tray)
stack.export(OUT+'stack_envelope_head_world.stl')
fig,axs=plt.subplots(2,3,figsize=(24,13))
ex_=np.array([1,0,0]);ey_=np.array([0,1,0]);ez_=np.array([0,0,1])
others=[(n,me,'0.45') for n,me in parts.items() if n not in('head_shell.stl','head_pitch.stl','eye.stl')]+[(f's{i}',s,'#d62728') for i,(b,s) in enumerate(servos)]
old=parts['head_shell.stl']
def sect(ax,plane_o,normal,uv,xl,yl):
    section2d(ax,others,plane_o,normal,uv,lw=.8)
    section2d(ax,[('old',old,'#888888')],plane_o,normal,uv,lw=1.2)
    section2d(ax,[('new',new,'#1f77b4')],plane_o,normal,uv,lw=1.6)
    section2d(ax,[('stack',stack,'#2ca02c')],plane_o,normal,uv,lw=1.6)
    section2d(ax,[('tray',tray,'#ff7f0e')],plane_o,normal,uv,lw=1.2)
    ax.set_xlim(*xl); ax.set_ylim(*yl)
for ax,y in zip(axs[0],[0,8,20]):
    sect(ax,[0,y,0],ey_,(ex_,ez_),(-25,110),(200,280)); ax.set_title(f'XZ section y={y}  (grey=old shell, blue=raised shell, green=stack, red=servos)')
for ax,x in zip(axs[1],[30,52,75]):
    sect(ax,[x,0,0],ex_,(ey_,ez_),(-50,50),(200,280)); ax.set_title(f'YZ section x={x}')
plt.tight_layout(); plt.savefig('10_head_sections_stack.png',dpi=55)
