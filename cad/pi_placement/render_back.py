import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *; from viz import *
import matplotlib.patches as mp
m,d=load(); set_pose(m,d,'STAND'); wm=world_meshes(m,d)
ms=[(n,me,'0.3') for b,l in wm.items() for n,me in l]
fig,ax=plt.subplots(figsize=(13,9))
section2d(ax,ms,[0,0,0],np.array([0,1,0]),(np.array([1,0,0]),np.array([0,0,1])),lw=.8)
x0=-36  # boss tips
layers=[('carrier plate 1.5',1.5,'#999999'),('standoffs 8 (URT PCB 1.6 + parts, 11.5 total)',8+11.5,'#e6a800'),('gap 1 + tray 1.5',2.5,'#bbbbbb'),('Pi Zero 2W + PiSugar 3 stack 28.5 (header+jumpers)',28.5,'#2ca02c'),('roof/cover 1.5',1.5,'#999999')]
x=x0; 
for name,t,c in layers:
    ax.add_patch(mp.Rectangle((x-t,77),t,66,fc=c,ec='k',alpha=.75)); ax.text(x-t/2,146,f'{t:g}',ha='center',fontsize=8); x-=t
ax.annotate('',xy=(x,70),xytext=(x0,70),arrowprops=dict(arrowstyle='<->',color='b')); ax.text((x+x0)/2,64,f'depth {x0-x:.1f} mm behind boss tips (x={x:.1f})',ha='center',color='b')
ax.axvline(-47,color='r',ls='--'); ax.text(-46,50,'today: back of 7-11 mm\nboard slab x=-47',color='r',fontsize=8)
ax.axvline(x,color='b',ls='--'); ax.text(x-1,100,'lying-on-back\nfloor plane',rotation=90,color='b',ha='right')
ax.axvline(-107,color='g',ls=':'); ax.text(-106,30,'farthest leg reach\n(MJCF sweep) x=-107',color='g',fontsize=8)
ax.set_xlim(-125,60); ax.set_ylim(20,170); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_title('Option C: URT-2 + Pi stack on the back, side section y=0 (STAND)')
plt.savefig('14_back_stacked_side.png',dpi=60)
print('depth',x0-x)
