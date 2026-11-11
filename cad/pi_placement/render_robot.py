import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *; from viz import *
m,d=load()
def body_T(pose,b):
    set_pose(m,d,pose); T=np.eye(4); bid=m.body(b).id; T[:3,:3]=d.xmat[bid].reshape(3,3); T[:3,3]=d.xpos[bid]*1000; return T
Ti=body_T('INIT','jaw_soft'); Ts=body_T('STAND','jaw_soft'); Hmap=Ts@np.linalg.inv(Ti)
set_pose(m,d,'STAND'); wm=world_meshes(m,d)
robot=[(n,me,(.6,.6,.65),0.55) for b,l in wm.items() for n,me in l if n!='head_roll']
stack=trimesh.load(OUT+'stack_envelope_head_world.stl'); stack.apply_transform(Hmap)
shell=trimesh.load(OUT+'head_shell_world_pi.stl'); shell.apply_transform(Hmap)
# original head_roll mesh replaced by new raised shell + others (keep original head_roll but ghost)
hr=[me for b,l in wm.items() for n,me in l if n=='head_roll'][0]
# B: stack box on the torso top
def box(x0,x1,y0,y1,z0,z1):
    b=trimesh.creation.box(extents=[x1-x0,y1-y0,z1-z0]); b.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]); return b
stB=box(-52,13,-15,15,158,186.5)
stC=box(-89.5,-36,-18.3,18.3,77,143)
fig=plt.figure(figsize=(24,8))
views=[(15,-55),(15,-125),(5,-90)]
cfg=[('A: head option (raised shell, stack)',[('hr',hr,(.8,.8,.9),0.1),('shell',shell,(.25,.45,.9),0.35),('stack',stack,(.1,.65,.2),0.95)]),
     ('B: top of torso (neck 20 deg)',[('hr',hr,(.8,.8,.9),0.5),('stack',stB,(.1,.65,.2),0.95)]),
     ('C: back stacked (53.5 deep)',[('hr',hr,(.8,.8,.9),0.5),('stack',stC,(.1,.45,.9),0.9)])]
for i,(t,extra) in enumerate(cfg):
    ax=fig.add_subplot(1,3,i+1,projection='3d')
    draw3d(ax,robot+extra,15,-120+i*0)
    ax.set_xlim(-110,110); ax.set_ylim(-110,110); ax.set_zlim(0,280); ax.set_box_aspect((220,220,280)); ax.set_title(t)
plt.tight_layout(); plt.savefig('15_options_on_robot.png',dpi=55)
