import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_final import *
from headmodel import head_parts
parts,servos,m_,d_=head_parts()
new=trimesh.load(OUT+'head_shell_world_pi.stl')
from stackfit import L,W,BOXES,TRAY
def stack_boxes(clr):
    out=[]
    bx=[(0,65,0,30,-TRAY,17.5)]+BOXES[1:]
    for (x0,x1,y0,y1,z0,z1) in bx:
        b=trimesh.creation.box(extents=[x1-x0+2*clr,y1-y0+2*clr,z1-z0+2*clr]); b.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]); out.append(to_world(b)[0])
    return out
for clr in [0.0,1.0]:
    sb=stack_boxes(clr)
    tot={}
    for name,me in [('head_shell_pi',new)]+[(k,v) for k,v in parts.items() if k not in('head_shell.stl','head_pitch.stl')]+[(f'servo{i}',s) for i,(b,s) in enumerate(servos)]:
        v=0
        for b in sb:
            try:
                x=trimesh.boolean.intersection([b,me],engine='manifold'); v+=x.volume if not x.is_empty else 0
            except Exception as e: v+=0
        tot[name]=round(v,2)
    print('clearance',clr,'overlap volume mm3 per part:',tot)
