import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from head_final import *
from viz import *
from headmodel import head_parts
parts,servos,m_,d_=head_parts()
new=trimesh.load(OUT+'head_shell_world_pi.stl'); old=parts['head_shell.stl']
sb=[to_world(b)[0] for b in stack_world_mesh()]
fig=plt.figure(figsize=(22,11))
def sc(me,col,a): return ('x',me,col,a)
base=[sc(parts[k],c,a) for k,c,a in [('head.stl',(.55,.55,.75),0.9),('jaw.stl',(.3,.7,.3),0.9),('head_servo_support.stl',(.7,.45,.25),0.9),('eye_shell.stl',(.95,.55,.1),0.9)]]
base+=[sc(s,(.85,.15,.15),0.95) for b,s in servos[:2]+servos[3:4]]
stk=[sc(b,(.1,.6,.2),0.95) for b in sb]
for i,(e,a,title,shell) in enumerate([(25,-60,'raised shell (ghost) + stack',new),(25,-125,'rear 3/4 view',new),(55,-90,'top view, old dome outline',old)]):
    ax=fig.add_subplot(1,3,i+1,projection='3d')
    items=base+stk+[sc(shell,(.3,.5,.9),0.16)]
    draw3d(ax,items,e,a)
    ax.set_xlim(-25,110); ax.set_ylim(-55,55); ax.set_zlim(200,275); ax.set_box_aspect((135,110,75)); ax.set_title(title)
plt.tight_layout(); plt.savefig('11_head_3d_stack.png',dpi=60)
# free-height map: top-down
D=np.load(OUT+'head_grid.npz'); xs_,ys_,zs_=D['xs'],D['ys'],D['zs']
B=np.load(OUT+'head_blocked.npz'); ceil=B['ceil_z']; others=B['others']
topobs=np.where(others.any(2),(others*zs_[None,None,:]).max(2),np.nan)
# top of highest non-shell obstacle UNDER ceiling
fh=ceil-np.where(np.isnan(topobs),212,topobs)
fig,axs=plt.subplots(1,2,figsize=(20,8))
im=axs[0].imshow(np.where(np.isnan(ceil),np.nan,ceil).T,origin='lower',extent=[xs_[0],xs_[-1],ys_[0],ys_[-1]],cmap='viridis'); plt.colorbar(im,ax=axs[0]); axs[0].set_title('old dome inner ceiling height z [mm] (top view, x fwd ->)')
im=axs[1].imshow(np.where(np.isnan(fh)|(fh<0),np.nan,fh).T,origin='lower',extent=[xs_[0],xs_[-1],ys_[0],ys_[-1]],cmap='RdYlGn',vmin=0,vmax=45); plt.colorbar(im,ax=axs[1]); axs[1].set_title('free height: ceiling - highest obstacle (servos/plate/base) [mm]; need 28.5+2.5 for the stack')
import matplotlib.patches as mp
for ax in axs:
    for nm,(x0,x1,y0,y1,c) in {'S_rear':(-6.4,23.6,-24.5,9.5,'r'),'S_yaw':(31,51,-24.5,9.5,'r'),'S_front':(31.5,65.5,11.5,40.5,'r'),'servo plate':(10,70.6,-36,32.5,'k'),'eye shell':(91.4,103,-15.3,14.7,'orange')}.items():
        ax.add_patch(mp.Rectangle((x0,y0),x1-x0,y1-y0,fill=False,ec=c,lw=1.5)); ax.text(x0+1,y0+1,nm,fontsize=8,color=c)
    # stack footprint
    corners=np.array([[0,0],[65,0],[65,30],[0,30],[0,0]]); c=centre; Rr=R[:2,:2]
    pts=(Rr@(corners-np.array([32.5,15])).T).T+c[:2]; ax.plot(pts[:,0],pts[:,1],'b-',lw=2); ax.text(pts[:,0].mean()-8,pts[:,1].mean(),'Pi stack\n(ports edge = +y)',color='b',fontsize=9)
    ax.set_xlim(-25,108); ax.set_ylim(-46,46)
plt.tight_layout(); plt.savefig('12_head_free_height_map.png',dpi=55)
