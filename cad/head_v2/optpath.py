import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from evalpath import *
from scipy.optimize import minimize
def optimize(fixed_start,fixed_end,init_mid,occ_name,r_tube,rmin,margin=0.3,seed=0,iters=6000,skip_start=0.0,tail=None,dev=2.0):
    init_mid=np.asarray(init_mid,float); n=len(init_mid)
    lo=np.array([5,10,205]); hi=np.array([60,40,265])
    def build(v):
        mid=v.reshape(n,3); return spline_path(np.vstack([fixed_start,mid,fixed_end]),n=240)
    def cost(v):
        P=build(v); c=clearance(P,occ_name); R=radius(P)
        s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]
        m=(s>skip_start)
        viol=np.maximum(0,r_tube+margin-c[m]); rv=np.maximum(0,rmin-np.minimum(R[3:-3],200)); 
        return 400*np.sum(viol**2)+8*np.sum(rv**2)/len(P)*10+0.02*s[-1]
    bnds=[(v-dev,v+dev) for v in init_mid.ravel()]
    res=minimize(cost,init_mid.ravel(),method='Powell',bounds=bnds,options={'maxiter':iters,'xtol':1e-2,'ftol':1e-4})
    P=build(res.x); return P,res
if __name__=='__main__':
    O='plan_occ_cut.npy'
    E=np.array([32.1,27.5,247.5]); END=np.array([27.0,16.5,214.0])
    init=[[29.0,27.2,246.8],[26.6,25.6,245.0],[25.6,23.8,242.6],[25.4,21.8,241.2],[25.4,19.5,240.8],[26.4,17.5,236.5],[27.0,16.5,226.0]]
    P,res=optimize(E,END,init,O,1.75,7.0,skip_start=3.0)
    report('USB opt',P,1.75,O); np.save(OUT2+'path_usb_opt.npy',P)
