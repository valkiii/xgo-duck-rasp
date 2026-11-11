import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from envelope import *
from ribbon import *
from route_cables import add_tube
import numpy as np
g=voxelize_boxes(pi_envelope_boxes())
g|=ribbon_voxels(g.shape)
usb=np.load(OUT2+'path_usb.npy'); wires=np.load(OUT2+'path_wires.npy')
z=np.zeros_like(g)
z=add_tube(z,usb[np.linalg.norm(usb-usb[0],axis=1)>3],1.75+0.8)
z=add_tube(z,wires[np.linalg.norm(wires-wires[0],axis=1)>3],1.9+0.8)
g|=z
# camera dummy (upside-down board, lens, plate, connector)
import trimesh
from plan_grid import mesh_grid
need,top=need_from_grid(g)
need[xs>88.0,:]=0
print('max need %.1f'%need.max(),'at',[ (xs[i],ys[j]) for i,j in [np.unravel_index(need.argmax(),need.shape)]])
# where is need>0 : bbox
ii,jj=np.where(need>0.2); print('need>0.2 bbox x',xs[ii.min()],xs[ii.max()],'y',ys[jj.min()],ys[jj.max()])
np.savez_compressed(OUT2+'need_all.npz',need=need,top=top)
