import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from common2 import *
shell=load_part_world('head_shell.stl'); es=load_part_world('eye_shell.stl'); head=load_part_world('head.stl'); jaw=load_part_world('jaw.stl')
eye=trimesh.load(STRUCT+'eye.stl'); eye.apply_transform(T_of('eye_shell.stl'))
fig,axs=plt.subplots(2,3,figsize=(24,14))
ex=np.array([1,0,0]);ey=np.array([0,1,0]);ez=np.array([0,0,1])
items=[('shell',shell,'#888888'),('es',es,'#ff8800'),('eye',eye,'#0066ff'),('head',head,'#6666ff'),('jaw',jaw,'#33aa33')]
for ax,y in zip(axs[0],[0,-8,10]):
    section2d(ax,items,[0,y,0],ey,(ex,ez),lw=1.2); ax.set_xlim(60,110); ax.set_ylim(205,270); ax.set_title(f'XZ y={y}')
for ax,x in zip(axs[1],[89,92.5,96]):
    section2d(ax,items,[x,0,0],ex,(ey,ez),lw=1.2); ax.set_xlim(-48,48); ax.set_ylim(205,270); ax.set_title(f'YZ x={x}')
plt.tight_layout(); plt.savefig('a3_eye_detail.png',dpi=50)
print('shell front wall: ',[ (round(b,1)) for b in shell.bounds.flatten()])
