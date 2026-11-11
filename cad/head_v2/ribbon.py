import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from evalpath import *
RIB_W=16.0; RIB_T=0.4
def ribbon_ctrl():
    return np.array([(87.0,0,250.6),(87.0,0,253.0),(86.2,0.3,255.2),(84.0,0.6,256.4),(80.0,1.0,256.6),(72,2.0,255.6),(62,3.5,253.6),(50,4.5,252.5),(35,5.2,252.3),(24,5.5,252.4),(18.0,5.5,252.5),(15.0,5.5,251.6),(13.8,5.5,249.9),(14.6,5.5,248.2),(16.6,5.5,247.7),(19.0,5.5,247.6)])
def ribbon_path(n=300): return spline_path(ribbon_ctrl(),n=n)
def ribbon_voxels(shape,half_w=RIB_W/2+0.5,clr=0.6):
    g=np.zeros(shape,bool); P=ribbon_path(600)
    for p in P:
        i=(XS>=p[0]-clr)&(XS<=p[0]+clr); j=(YS>=p[1]-half_w)&(YS<=p[1]+half_w); k=(ZS>=p[2]-RIB_T/2-clr)&(ZS<=p[2]+RIB_T/2+clr)
        g[np.ix_(i,j,k)]=True
    return g
if __name__=='__main__':
    P=ribbon_path(); R=radius(P); print('ribbon length %.0f mm, min radius %.1f, 5th pct %.1f'%(path_length(P),R[3:-3].min(),np.percentile(R,5)))
    table(P,'plan_occ_cut.npy',every=20)
