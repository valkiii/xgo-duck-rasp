import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from route_cables import *
from design import *
CUT_FLANGE=(23.0,26.4,14.0,19.2,221.0,238.2)   # x0,x1,y0,y1,z0,z1  : support plate west flange segment removed
CUT_GATE=(14.5,22.5,14.2,19.8,203.0,221.0)       # slot through head.stl floor/ledge for the 7-wire bundle
def apply_cuts(occ):
    occ=occ.copy()
    for (x0,x1,y0,y1,z0,z1) in (CUT_FLANGE,CUT_GATE):
        ix=(XS>=x0)&(XS<=x1); iy=(YS>=y0)&(YS<=y1); iz=(ZS>=z0)&(ZS<=z1)
        occ[np.ix_(ix,iy,iz)]=False
    return occ
if __name__=='__main__':
    occ=np.load(OUT2+'plan_occ_v0.npy'); occ=apply_cuts(occ)
    # allow planning below the artificial z<211 floor inside the gate column
    occ[np.ix_((XS>=14.5)&(XS<=22.5),(YS>=14.2)&(YS<=19.8),ZS<211)]=False
    np.save(OUT2+'plan_occ_cut.npy',occ)
    O='plan_occ_cut.npy'
    E=np.array([34.1,27.5,247.5]); END=np.array([27.4,16.6,212.0])
    init=[[31.0,27.4,246.9],[28.8,26.0,245.2],[27.6,24.0,243.0],[27.4,21.8,241.4],[27.4,19.5,240.6],[27.4,17.2,238.5],[27.4,16.6,232.0],[27.4,16.6,222.0]]
    P,res=optimize(E,END,init,O,1.75,9.0,margin=0.5,skip_start=3.0,dev=2.0,iters=15000)
    report('USB',P,1.75,O); np.save(OUT2+'path_usb.npy',P)
    occ2=add_tube(occ,P[np.linalg.norm(P-E,axis=1)>3],1.75+0.3); np.save(OUT2+'plan_occ_cut_usb.npy',occ2)
