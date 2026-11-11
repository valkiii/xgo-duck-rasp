import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
m,d=load()
def com(add=None):
    M=m.body_mass.sum(); P=(m.body_mass[:,None]*d.xipos).sum(0)
    if add:
        for mass,p in add: M+=mass; P=P+mass*np.array(p)
    return M,P/M
set_pose(m,d,'STAND')
M0,c0=com()
print('total mass g',M0*1000,'COM STAND (mm)',(c0*1000).round(1))
hm={b:m.body_mass[m.body(b).id]*1000 for b in ['neck','neck_pitch','yaw_roll_motion','jaw_soft','trunk_base']}
print(hm, 'head group (yaw_roll_motion+jaw_soft+neck_pitch)',hm['yaw_roll_motion']+hm['jaw_soft']+hm['neck_pitch'])
# head COM and moment about neck-pitch axis
nb=m.body('neck'); axis_pt=d.xpos[nb.id].copy()   # neck joint origin
print('neck joint axis point (mm)',(axis_pt*1000).round(1))
hb=[m.body(b).id for b in ['neck_pitch','yaw_roll_motion','jaw_soft']]
mh=sum(m.body_mass[i] for i in hb); ch=sum(m.body_mass[i]*d.xipos[i] for i in hb)/mh
print('head group COM',(ch*1000).round(1),'moment about neck axis (x-lever mm)',((ch-axis_pt)[0]*1000).round(1),'torque N.mm', (mh*9.81*(ch-axis_pt)[0]*1000*1000).round(0))
feet=[d.site_xpos[m.site(n).id]*1000 for n in ['left_foot','right_foot']]
print('foot sites',[f.round(1) for f in feet])
# foot sole x extent
set_pose(m,d,'STAND')
# helper: INIT->STAND mapping of a head-local point
def head_point_stand(p_init):
    set_pose(m,d,'INIT'); bid=m.body('jaw_soft').id
    Rm=d.xmat[bid].reshape(3,3); t=d.xpos[bid]
    loc=Rm.T@(np.array(p_init)/1000-t)
    set_pose(m,d,'STAND'); return (d.xmat[bid].reshape(3,3)@loc+d.xpos[bid])
stack_head=head_point_stand([52,8,240.5+14])   # stack COM approx 14 mm above LiPo bottom
print('head stack COM at STAND (mm)',(stack_head*1000).round(1))
opts={
 'A head (55 g)':[(0.055,stack_head)],
 'B torso top (55 g) @(-20,0,172)':[(0.055,np.array([-20,0,172])/1000)],
 'C back stacked (stack+URT+plastic 80 g) @(-65,0,110)':[(0.080,np.array([-65,0,110])/1000)],
 'ref: back side-by-side v1 (30 g plastic+~60 g elec) @(-52,0,109)':[(0.090,np.array([-52,0,109])/1000)],
}
for k,a in opts.items():
    M,c=com(a); print(k.ljust(66),'M %.0f g'%(M*1000),'COM',(c*1000).round(1),'shift',((c-c0)*1000).round(1))
