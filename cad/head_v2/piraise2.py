import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
from piraise import *
UB=1.5; P0=UB; P1=UB+1.4
H={
 'under':(0,65,0,30,0,UB),
 'pcb':(0,65,0,30,P0,P1),
 'top_parts':(0,65,0,30,P1,P1+3.3),
 'header_rest':(29.5,58.5,22.5,29.4,P1,P1+10.0),
 'jumper_zone':(6.0,29.5,22.5,29.4,P1,P1+21.0),
 'usb_plug':(35.4,47.4,-11,0,UB-2,UB+7),
 'sd_card':(-3,0,9,23,UB-2,P1+1),
}
if __name__=='__main__':
    print('yawg max z',zs[np.where(np.load(OUT2+'grid2.npz')['yawg'].any((0,1)))[0][-1]])
    variants={'hollow all':H,'hollow no usb/sd':{k:v for k,v in H.items() if k not in('usb_plug','sd_card')},'hollow no jumper/usb/sd':{k:v for k,v in H.items() if k in('under','pcb','top_parts','header_rest')}}
    for nm,bx in variants.items():
        b=best_for(bx); print(nm,'->',[(round(r[0],1),r[1]) for r in b[:8]],flush=True)
