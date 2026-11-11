import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from register import *
def head_parts(pose='INIT'):
    """printable STLs transformed into world (mm) frame of given pose, plus transform used. Returns dict name->mesh"""
    m,d=load(); set_pose(m,d,'INIT')
    res=json.load(open(OUT+'registration_init.json'))
    parts={}
    for nm,r in res.items():
        me=trimesh.load(STRUCT+nm); me.apply_transform(np.array(r['T'])); parts[nm]=me
    # head servos from MJCF components (not printable)
    wm=world_meshes(m,d)
    k=0
    servos=[]
    for b in ['jaw_soft','yaw_roll_motion','neck_pitch','neck']:
        for n,me in wm[b]:
            for p in me.split(only_watertight=False):
                if p.is_volume and abs(p.volume/1000-16.0)<1 : servos.append((b,p))
                if p.is_volume and 15.5<p.volume/1000<16.5: pass
    return parts,servos,m,d
if __name__=='__main__':
    parts,servos,m,d=head_parts()
    for b,s in servos: print(b,s.bounds.round(1).tolist(),round(s.volume/1000,2))
    for n in ['head_camera','tof','head_imu','mouth_tip']:
        print(n, (d.site_xpos[m.site(n).id]*1000).round(1))
