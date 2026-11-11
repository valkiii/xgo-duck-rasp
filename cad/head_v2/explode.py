import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from previews import *
import matplotlib.pyplot as plt
def off(m,dz=0,dx=0,dy=0):
    m=m.copy(); m.apply_translation([dx,dy,dz]); return m
c=lambda h: tuple(int(h[i:i+2],16)/255 for i in (1,3,5))
fig=plt.figure(figsize=(26,12))
for k,(el,az) in enumerate([(22,-58),(30,-125)]):
    ax=fig.add_subplot(1,2,k+1,projection='3d')
    base=[('head',off(head),c(COL['head']),0.8),('sup',off(sup,8),c(COL['sup']),0.9),('servo',off(serv['jaw_soft_31'],8),c(COL['servo']),0.9),('servo2',off(serv['jaw_soft_-6'],8),c(COL['servo']),0.9),('servo3',off(serv['yaw_roll_motion_31'],8),c(COL['servo']),0.9),
          ('jaw',off(jaw,-20),c(COL['jaw']),0.8),
          ('spac',off(spac,18),c(COL['tray']),1),('tray',off(tray,26),c(COL['tray']),1),
          ('pi',off(pi,36),c(COL['pi']),1),('hdr',off(hdr,36),c(COL['hdr']),1),('plug',off(plug,36),c(COL['plug']),1),
          ('usb',off(usbm,36),(0,0,0),1),('w',off(wirm,36),c(COL['wires']),1),('rib',off(ribm,36),c(COL['rib']),1),
          ('shell',off(shell,70),c(COL['shell']),0.28),('es',off(es,50,12),c(COL['es']),0.9),('eye',off(eyec,50,24),c(COL['eye']),1),
          ('cam',off(cam,36,-28),c(COL['cam']),1),('cm',off(cm,36,-40),c(COL['cm']),1)]
    draw3d(ax,base,el,az)
    ax.set_xlim(-25,125); ax.set_ylim(-60,60); ax.set_zlim(180,360); ax.set_box_aspect((150,120,180))
    ax.set_title('exploded view (bottom to top: head, support plate+servos, spacers, tray, Pi+cables, shell; camera parts pulled out front)')
plt.tight_layout(); plt.savefig('b5_exploded.png',dpi=55)
