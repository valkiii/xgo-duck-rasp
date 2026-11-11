import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from common2 import *
serv=head_servos_world()
parts={n:load_part_world(n) for n in ['head.stl','head_shell.stl','jaw.stl','eye_shell.stl','head_servo_support.stl','head_yaw.STL']}
Syaw=serv['yaw_roll_motion_31']; Sfront=serv['jaw_soft_31']; Srear=serv['jaw_soft_-6']
zs=[214,219,224,229,234,238]
fig,axs=plt.subplots(2,3,figsize=(27,16))
ex=np.array([1,0,0]);ey=np.array([0,1,0]);ez=np.array([0,0,1])
for ax,z in zip(axs.ravel(),zs):
    section2d(ax,[('h',parts['head.stl'],'#6666ff'),('sup',parts['head_servo_support.stl'],'#aa6633'),('jaw',parts['jaw.stl'],'#33aa33'),('eye',parts['eye_shell.stl'],'#ff9900'),('shell',parts['head_shell.stl'],'#999999'),
        ('Sf',Sfront,'#dd0000'),('Sr',Srear,'#dd0000')],[0,0,z],ez,(ex,ey),lw=1)
    for ang,ls in [(-25,':'),(0,'-'),(25,'--')]:
        T=rotx_about(np.radians(ang),0,222.6)
        for nm,me in [('Syaw',Syaw),('hyaw',parts['head_yaw.STL'])]:
            r=me.copy(); r.apply_transform(T); 
            s=r.section(plane_origin=[0,0,z],plane_normal=ez)
            if s is None: continue
            for e in s.entities:
                p=s.vertices[e.points]; ax.plot(p[:,0],p[:,1],'m',ls=ls,lw=1.3)
    ax.set_xlim(-25,108); ax.set_ylim(-48,48); ax.set_aspect('equal'); ax.set_title(f'plan z={z} (magenta: yaw servo+head_yaw part at roll -25/0/+25)'); ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig('a1_plan_slices.png',dpi=45)
