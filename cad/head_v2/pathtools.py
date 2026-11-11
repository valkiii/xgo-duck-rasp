import sys; sys.path.insert(0,'/Users/vc/Research/xgo-duck/cad/head_v2')
import numpy as np, trimesh
def fillet_path(pts,radii,step=0.5):
    """polyline pts (N,3) with fillet radius per interior corner (len N-2). returns dense points and min radius used. raises if legs too short"""
    pts=np.asarray(pts,float); out=[pts[0]]; ok=True; info=[]
    prev=pts[0]
    segs=[]
    for i in range(1,len(pts)-1):
        a=pts[i-1]; b=pts[i]; c=pts[i+1]; r=radii[i-1]
        u=(a-b); lu=np.linalg.norm(u); u/=lu; v=(c-b); lv=np.linalg.norm(v); v/=lv
        cosang=np.clip(np.dot(u,v),-1,1); ang=np.arccos(cosang)       # interior angle
        if abs(ang-np.pi)<1e-6: segs.append((b,None,None,None,0)); continue
        t=r/np.tan(ang/2)
        info.append((i,t,lu,lv))
        segs.append((b,u,v,t,r))
    # build
    res=[pts[0]]
    for k,(b,u,v,t,r) in enumerate(segs):
        if u is None: res.append(b); continue
        p1=b+u*t; p2=b+v*t
        # circle centre
        bis=(u+v); bis/=np.linalg.norm(bis); ang=np.arccos(np.clip(np.dot(u,v),-1,1))
        cen=b+bis*(r/np.sin(ang/2))
        a1=p1-cen; a2=p2-cen
        th=np.arccos(np.clip(np.dot(a1,a2)/(np.linalg.norm(a1)*np.linalg.norm(a2)),-1,1))
        n=max(int(th*r/step),4)
        axis=np.cross(a1,a2); axis/=np.linalg.norm(axis)
        for s in np.linspace(0,1,n):
            ang_s=th*s
            # rodrigues
            rot=a1*np.cos(ang_s)+np.cross(axis,a1)*np.sin(ang_s)+axis*np.dot(axis,a1)*(1-np.cos(ang_s))
            res.append(cen+rot)
    res.append(pts[-1])
    res=np.array(res)
    # check legs
    bad=[]
    for (i,t,lu,lv) in info:
        pass
    return res,info
def leg_check(pts,radii):
    pts=np.asarray(pts,float); msgs=[]
    ts=[]
    for i in range(1,len(pts)-1):
        a,b,c=pts[i-1],pts[i],pts[i+1]
        u=(a-b)/np.linalg.norm(a-b); v=(c-b)/np.linalg.norm(c-b)
        ang=np.arccos(np.clip(np.dot(u,v),-1,1)); ts.append(radii[i-1]/np.tan(ang/2) if abs(ang-np.pi)>1e-6 else 0)
    for i in range(len(pts)-1):
        L=np.linalg.norm(pts[i+1]-pts[i]); need=(ts[i-1] if i>=1 else 0)+(ts[i] if i<len(ts) else 0)
        if need>L+1e-6: msgs.append(f'leg {i}-{i+1} length {L:.1f} < tangents {need:.1f}')
    return msgs
def tube(path,r,sections=16):
    # sweep a circle along dense path
    P=np.asarray(path); T=np.gradient(P,axis=0); T/=np.linalg.norm(T,axis=1)[:,None]
    verts=[];faces=[]
    ref=np.array([0,0,1.0])
    for i,(p,t) in enumerate(zip(P,T)):
        a=np.cross(t,ref); 
        if np.linalg.norm(a)<1e-3: a=np.cross(t,[1,0,0])
        a/=np.linalg.norm(a); b=np.cross(t,a)
        for s in range(sections):
            th=2*np.pi*s/sections; verts.append(p+r*(np.cos(th)*a+np.sin(th)*b))
    n=len(P)
    for i in range(n-1):
        for s in range(sections):
            a_=i*sections+s; b_=i*sections+(s+1)%sections; c_=(i+1)*sections+s; d_=(i+1)*sections+(s+1)%sections
            faces+=[[a_,b_,c_],[b_,d_,c_]]
    m=trimesh.Trimesh(np.array(verts),np.array(faces),process=False)
    # caps
    return m
def path_length(P): return float(np.sum(np.linalg.norm(np.diff(P,axis=0),axis=1)))
