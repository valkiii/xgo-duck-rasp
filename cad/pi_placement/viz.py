import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
def draw3d(ax, meshes, elev=20, azim=-60, alpha=0.5):
    cols=plt.cm.tab20(np.linspace(0,1,20))
    for i,(n,me,*c) in enumerate(meshes):
        col=c[0] if c else cols[i%20]
        a=c[1] if len(c)>1 else alpha
        tri=me.vertices[me.faces]
        pc=Poly3DCollection(tri,alpha=a,facecolor=col,edgecolor='none',linewidths=0)
        # simple shading
        nrm=me.face_normals; l=np.array([0.3,-0.5,0.8]); l/=np.linalg.norm(l)
        sh=0.45+0.55*np.abs(nrm@l)
        fc=np.array([col[:3]])*sh[:,None]
        pc.set_facecolor(np.c_[fc,np.full(len(fc),a)])
        ax.add_collection3d(pc)
    allv=np.vstack([m[1].vertices for m in meshes]); lo=allv.min(0); hi=allv.max(0); c=(lo+hi)/2; r=(hi-lo).max()/2
    ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
    ax.set_box_aspect((1,1,1)); ax.view_init(elev,azim)
    ax.set_xlabel('x fwd'); ax.set_ylabel('y left'); ax.set_zlabel('z up')
def section2d(ax, meshes, origin, normal, axes_uv, colors=None, lw=1.0):
    """plot slice outlines. axes_uv: (u_vec,v_vec) unit vectors in plane"""
    cols=plt.cm.tab20(np.linspace(0,1,20))
    for i,(n,me,*c) in enumerate(meshes):
        col=c[0] if c else cols[i%20]
        try: s=me.section(plane_origin=origin,plane_normal=normal)
        except Exception as e: s=None
        if s is None: continue
        for ent in s.entities:
            pts=s.vertices[ent.points]
            u=pts@axes_uv[0]; v=pts@axes_uv[1]
            ax.plot(u,v,color=col,lw=lw)
    ax.set_aspect('equal'); ax.grid(alpha=.3)
