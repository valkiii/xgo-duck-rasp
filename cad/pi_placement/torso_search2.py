import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from torso_search import *
trunk=S['trunk']
import pickle
allres={}
for lg in ['legs_full_mjcf','legs_pitch60','legs_p60_r12_y12','legs_gait_p60_r6_y6','legs_sagittal_p60']:
    for hv in ['none','head_nominal']:
        obst=trunk|S[lg]|(S[hv] if hv!='none' else False)
        r=search(obst,trunk,(66,31,29),lg+'/'+hv)
        allres[(lg,hv)]=r
        for perm,f,o,a in r:
            if o:
                lo=a.min(0); hi=a.max(0)
                print('    ',perm,'x',lo[0]+GX[0],hi[0]+GX[0],'y',lo[1]+GY[0],hi[1]+GY[0],'z',lo[2]+GZ[0],hi[2]+GZ[0])
pickle.dump(allres,open(OUT+'torso_search2.pkl','wb'))
