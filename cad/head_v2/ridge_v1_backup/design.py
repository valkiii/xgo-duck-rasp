"""Shared definitions for head_v2: Pi pose, solids in the head (INIT world) frame, mm."""
import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2'); sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/pi_placement')
from common2 import *
import numpy as np, trimesh
# ---- Pi pose (yaw 180): stack (X,Y,Z) -> world
CX,CY=49.0,4.0
ZU=243.5          # world z of the underside-parts bottom (Z=0). checked against swept obstacles in verify_static.py
def S2W(p):
    p=np.atleast_2d(np.asarray(p,float)); return np.c_[CX+32.5-p[:,0], CY+15.0-p[:,1], ZU+p[:,2]]
def sbox(x0,x1,y0,y1,z0,z1,name=None):
    """box given in stack frame -> trimesh in world frame (axis aligned because yaw=180)"""
    a=S2W([x0,y0,z0])[0]; b=S2W([x1,y1,z1])[0]
    lo=np.minimum(a,b); hi=np.maximum(a,b)
    m=trimesh.creation.box(extents=hi-lo); m.apply_translation((lo+hi)/2); return m
def scyl(X,Y,z0,z1,r,sections=32):
    c=S2W([X,Y,z0])[0]; m=trimesh.creation.cylinder(radius=r,height=z1-z0,sections=sections); m.apply_translation([c[0],c[1],c[2]+(z1-z0)/2]); return m
# Pi geometry (stack frame)
UB=1.5; PCB_B=UB; PCB_T=UB+1.4; TOP=PCB_T+3.3
PCB=(0,65,0,30,PCB_B,PCB_T)
HDR_X0,HDR_X1=7.0,58.0
HDR=(HDR_X0,HDR_X1,22.6,27.6,PCB_T,PCB_T+8.5)           # housing+pins (8.5 mm above PCB)
JUMP=(5.8,29.5,22.5,29.4,PCB_T,PCB_T+21.0)               # reserved zone for 7 dupont housings + wire bend
HDR_REST=(29.5,58.5,22.5,29.4,PCB_T,PCB_T+10.0)
USB_PLUG=(35.4,47.4,-11.0,0.0,PCB_B-2.0,PCB_B+7.0)
SD_KEEP=(-3.0,0.0,9.0,23.0,PCB_B-2.0,PCB_T+1.0)
HOLES=[(3.5,3.5),(61.5,3.5),(3.5,26.5),(61.5,26.5)]
