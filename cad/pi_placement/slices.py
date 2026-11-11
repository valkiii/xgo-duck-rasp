import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common import *
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
D=np.load(OUT+'head_grid.npz'); xs,ys,zs=D['xs'],D['ys'],D['zs']
names=['head_stl','head_shell_stl','jaw_stl','eye_shell_stl','head_servo_support_stl','head_yaw_STL','servo0_jaw_soft','servo1_jaw_soft','servo3_yaw_roll_motion']
cols={'head_stl':'#8888ff','head_shell_stl':'#bbbbbb','jaw_stl':'#44aa44','eye_shell_stl':'#ee8800','head_servo_support_stl':'#aa6633','head_yaw_STL':'#cc44cc','servo0_jaw_soft':'#dd2222','servo1_jaw_soft':'#dd2222','servo3_yaw_roll_motion':'#dd2222'}
lab=np.zeros((len(xs),len(ys),len(zs)),int)
for i,n in enumerate(names): lab[D[n]]=i+1
from matplotlib.colors import ListedColormap
cm=ListedColormap(['white']+[cols[n] for n in names])
def plot_xz(ax,y):
    j=np.argmin(abs(ys-y)); ax.imshow(lab[:,j,:].T,origin='lower',extent=[xs[0]-.5,xs[-1]+.5,zs[0]-.25,zs[-1]+.25],cmap=cm,vmin=0,vmax=len(names),aspect='equal',interpolation='nearest'); ax.set_title(f'XZ y={y}')
def plot_yz(ax,x):
    i=np.argmin(abs(xs-x)); ax.imshow(lab[i,:,:].T,origin='lower',extent=[ys[0]-.5,ys[-1]+.5,zs[0]-.25,zs[-1]+.25],cmap=cm,vmin=0,vmax=len(names),aspect='equal',interpolation='nearest'); ax.set_title(f'YZ x={x}')
if __name__=='__main__':
    fig,axs=plt.subplots(3,4,figsize=(24,15))
    for ax,y in zip(axs[0],[0,-12,-25,-35]): plot_xz(ax,y)
    for ax,y in zip(axs[1],[10,20,30,38]): plot_xz(ax,y)
    for ax,x in zip(axs[2],[-10,15,45,80]): plot_yz(ax,x)
    plt.tight_layout(); plt.savefig('02_head_cavity_slices.png',dpi=50)
