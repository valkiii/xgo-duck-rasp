import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from optpath import *
import scipy.ndimage as ndi
O='plan_occ_cut.npy'
occ=np.load(OUT2+O)
def add_tube(occ,P,r):
    g=np.zeros_like(occ)
    for p in P:
        i=int(round(p[0]-XS[0])); j=int(round(p[1]-YS[0])); k=int(round((p[2]-ZS[0])/0.5))
        ri=int(np.ceil(r))+1
        for di in range(-ri,ri+1):
            for dj in range(-ri,ri+1):
                for dk in range(-2*ri,2*ri+1):
                    if di*di+dj*dj+(dk*0.5)**2<=r*r:
                        a,b,c=i+di,j+dj,k+dk
                        if 0<=a<g.shape[0] and 0<=b<g.shape[1] and 0<=c<g.shape[2]: g[a,b,c]=True
    return occ|g
if __name__=='__main__':
    # --- USB: re-optimise with fixed descent column x=25.4
    E=np.array([32.1,27.5,247.5]); END=np.array([25.4,15.8,212.0])
    init=[[29.0,27.2,246.8],[26.6,25.6,245.0],[25.6,23.8,242.6],[25.4,21.8,241.2],[25.4,19.5,240.6],[25.4,17.0,238.5],[25.4,15.8,232.0],[25.4,15.8,222.0]]
    P,res=optimize(E,END,init,O,1.75,7.0,skip_start=3.0)
    report('USB',P,1.75,O); np.save(OUT2+'path_usb.npy',P)
    occ2=add_tube(occ,P[np.linalg.norm(P-E,axis=1)>3],1.75+0.2)
    np.save(OUT2+'plan_occ_cut_usb.npy',occ2)
