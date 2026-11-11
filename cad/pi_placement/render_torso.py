import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from sweep import *
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, matplotlib.patches as mp
from matplotlib.colors import ListedColormap
sys.path.insert(0,'.')
from neckenv2 import headgrid
S=np.load(OUT+'sweeps_stand.npz')
xs=np.arange(GX[0],GX[1]+1); ys=np.arange(GY[0],GY[1]+1); zs=np.arange(GZ[0],GZ[1]+1)
trunk=S['trunk']
legs=S['legs_full_mjcf']; legs60=S['legs_p60_r12_y12']
g_nom=headgrid(np.radians(20),np.radians(-30),np.radians(10),np.radians(30),np.radians(10),n=800)  # neck -30..20, head pitch rel +-10, yaw +-30, roll +-10
g_full=S['head_full']
fig,axs=plt.subplots(1,3,figsize=(27,10))
def layer(ax,img,color,alpha,extent):
    cm=ListedColormap([(0,0,0,0),color]); ax.imshow(img,origin='lower',extent=extent,cmap=cm,alpha=alpha,vmin=0,vmax=1,interpolation='nearest')
ext_xz=[xs[0]-.5,xs[-1]+.5,zs[0]-.5,zs[-1]+.5]
ax=axs[0]
layer(ax,legs[:,ys>0,:].any(1).T,(0.9,0.3,0.3,1),0.25,ext_xz)       # left leg sweep, full MJCF limits
layer(ax,legs60[:,ys>0,:].any(1).T,(0.8,0.1,0.1,1),0.35,ext_xz)
layer(ax,g_full.any(1).T,(1,0.6,0,1),0.15,ext_xz)
layer(ax,g_nom.any(1).T,(1,0.6,0,1),0.45,ext_xz)
layer(ax,trunk.any(1).T,(0.1,0.1,0.1,1),0.9,ext_xz)
def rect(ax,x0,x1,z0,z1,c,l,fill=True,hatch=None):
    ax.add_patch(mp.Rectangle((x0,z0),x1-x0,z1-z0,fc=c if fill else 'none',ec=c,alpha=0.8 if fill else 1,lw=2,hatch=hatch)); ax.text(x0,z1+2,l,color=c,fontsize=10)
rect(ax,-52,13,158,187,'#00aa00','B: top of torso (neck<=20deg, head tracks neck)')
rect(ax,-86,-36,77,143,'#0044ff','C: stacked URT+Pi on back (53 mm)',fill=False)
rect(ax,-49,17,40,69,'#cc00cc','under torso (only if roll/yaw frozen)',fill=False,hatch='//')
rect(ax,52,118,127,156,'#cc00cc','chest (needs roll/yaw ~frozen, sticks out 70)',fill=False,hatch='//')
ax.set_xlim(-120,150); ax.set_ylim(0,290); ax.set_aspect('equal'); ax.set_title('side view (x fwd): trunk=black, left-leg sweep (red; dark=hip pitch+-60/roll,yaw+-12deg; light=full MJCF limits),\nhead sweep (orange; dark = neck<=20deg,head pitch+-10; light = all MJCF)'); ax.grid(alpha=.3)
# front view
ext_yz=[ys[0]-.5,ys[-1]+.5,zs[0]-.5,zs[-1]+.5]
ax=axs[1]
layer(ax,legs.any(0).T,(0.9,0.3,0.3,1),0.25,ext_yz); layer(ax,legs60.any(0).T,(0.8,0.1,0.1,1),0.35,ext_yz)
layer(ax,g_nom.any(0).T,(1,0.6,0,1),0.45,ext_yz); layer(ax,trunk.any(0).T,(0.1,0.1,0.1,1),0.9,ext_yz)
rect(ax,-15,15,158,187,'#00aa00','B stack (30 wide)'); rect(ax,-18.3,18.3,77,143,'#0044ff','C URT 36.5 wide',fill=False)
rect(ax,-39,39,77,143,'#ff0000','v1 backpack 78 wide',fill=False)
ax.set_xlim(-110,110); ax.set_ylim(0,290); ax.set_aspect('equal'); ax.set_title('front view (y left)'); ax.grid(alpha=.3)
# top view
ext_xy=[xs[0]-.5,xs[-1]+.5,ys[0]-.5,ys[-1]+.5]
ax=axs[2]
layer(ax,legs[:,:,(zs>=77)&(zs<=143)].any(2).T,(0.9,0.3,0.3,1),0.25,ext_xy); layer(ax,legs60[:,:,(zs>=77)&(zs<=143)].any(2).T,(0.8,0.1,0.1,1),0.35,ext_xy)
layer(ax,trunk.any(2).T,(0.1,0.1,0.1,1),0.9,ext_xy)
ax.add_patch(mp.Rectangle((-52,-15),65,30,fc='#00aa00',alpha=.5,ec='#00aa00')); ax.text(-50,17,'B',color='#00aa00')
ax.add_patch(mp.Rectangle((-89.5,-18.3),53.5,36.6,fc='none',ec='#0044ff',lw=2)); ax.text(-88,20,'C (depth 53.5 behind boss tips)',color='#0044ff')
ax.add_patch(mp.Rectangle((-61,-39),25,78,fc='none',ec='#ff0000',lw=2)); ax.text(-60,41,'v1 side-by-side backpack (78 wide, 25 deep)',color='#ff0000')
ax.set_xlim(-120,150); ax.set_ylim(-110,110); ax.set_aspect('equal'); ax.set_title('top view (legs z 77-143 band): leg sweep occupies the sides'); ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig('13_torso_options_keepout.png',dpi=50)
