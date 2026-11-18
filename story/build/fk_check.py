"""Forward kinematics sanity check of poses on the MJCF tree (degrees, controller sign convention)."""
import json, sys
import numpy as np
M = json.load(open('assets/duck_model.json'))
IDMAP = {'left_hip_yaw':10,'left_hip_roll':11,'left_hip_pitch':12,'left_knee':13,'left_ankle':14,
         'right_hip_yaw':20,'right_hip_roll':21,'right_hip_pitch':22,'right_knee':23,'right_ankle':24,
         'neck_pitch':30,'head_pitch':31,'head_yaw':32,'head_roll':33}
def qm(q):
    w,x,y,z=q; n=np.linalg.norm(q); w,x,y,z=np.array(q)/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def rot(axis,a):
    axis=np.array(axis)/np.linalg.norm(axis); K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.eye(3)+np.sin(a)*K+(1-np.cos(a))*K@K
FEET={'ankle_left':[0.022982865,-0.0053683133,-0.033328407],'ankle_right':[0.022982865,0.0053683132,-0.033328408],'jaw_soft':[0,-0.01234994,-0.07399628]}
def fk(b,R,p,deg,out):
    R=R@qm(b['quat']); p=p+(R@np.array(b['pos'])) if False else p
    return out
def run(deg):
    out={}
    def rec(b,R0,p0):
        p=p0+R0@np.array(b['pos']); R=R0@qm(b['quat'])
        if 'joint' in b:
            R=R@rot(b['joint']['axis'],np.deg2rad(deg.get(IDMAP[b['joint']['name']],0)))
        if b['name'] in FEET: out[b['name']]=p+R@np.array(FEET[b['name']])
        for c in b['children']: rec(c,R,p)
    rec(M['root'],np.eye(3),np.zeros(3))
    return out
poses={
 'zero':{},
 'home':dict(zip([10,11,12,13,14,20,21,22,23,24,30,31],[0,-5,-23,0,23,0,5,23,0,-23,20,20])),
 'sit_measured':dict(zip([10,11,12,13,14,20,21,22,23,24,30,31,32,33],[-0.3,3.1,70.6,69.5,-17.1,-0.2,-0.8,-62.2,-69.2,8.3,100.3,62.7,-6.3,-2.4])),
 'sit_negated':dict(zip([10,11,12,13,14,20,21,22,23,24,30,31,32,33],[0.3,-3.1,-70.6,-69.5,17.1,0.2,0.8,62.2,69.2,-8.3,-100.3,-62.7,6.3,2.4])),
}
for k,d in poses.items():
    o=run(d); print(k, {n:np.round(v*1000).astype(int).tolist() for n,v in o.items()})
print('--- single joints (left foot x fwd, z up, mm)')
for jid in (12,13,14,11,10,30,31):
    for a in (30,-30):
        o=run({jid:a, jid+10 if jid<20 else jid:0}); print(jid,a,np.round(o['ankle_left']*1000).astype(int).tolist(), 'jaw',np.round(o['jaw_soft']*1000).astype(int).tolist())
