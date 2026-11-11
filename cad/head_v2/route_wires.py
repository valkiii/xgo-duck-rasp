import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from route2 import *
O='plan_occ_cut_usb.npy'
S=np.array([64.0,-10.8,257.5]); END=np.array([18.0,16.4,211.0])
init=[[64.0,-14.5,255.0],[58.0,-15.5,251.5],[45.0,-15.5,250.0],[30.0,-14.5,249.0],[18.0,-13.5,247.0],[10.5,-10.5,245.0],[8.0,-2.0,244.5],[8.0,8.0,243.0],[10.5,14.0,240.5],[14.5,16.6,238.0],[18.0,16.6,231.0],[18.0,16.4,221.0]]
P,res=optimize(S,END,init,O,1.9,6.0,margin=0.8,skip_start=3.0,dev=3.0,iters=12000)
report('WIRES',P,1.9,O); np.save(OUT2+'path_wires.npy',P)
table(P,O,every=12)
