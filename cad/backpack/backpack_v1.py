#!/usr/bin/env python
"""
XGO-Duck "backpack": carrier + cover that bolt onto the 4 printed bosses on the
BACK of the battery box (structure/battery.stl of LuwuDynamics/xgoduck_hardware).

Run:   /Users/vc/Research/xgo-duck/.venv/bin/python backpack.py
Out:   carrier.stl, cover.stl (print orientation), backpack_assembled.stl
       (battery-box frame), electronics_boxes.stl, previews/*.png, report.json

COORDINATE FRAMES
  pack frame  (u, v, w):  u = robot x (to the RIGHT when you look at the robot's
                          back), v = robot z (up), w = distance BEHIND the boss
                          tip plane (w = 0 is the plane the carrier touches the
                          bosses; w grows away from the robot).
  battery frame (x, y, z) = battery.stl:  x = u + U0,  z = v + V0,  y = -24.6 - w.

LAYOUT (looking at the robot's back, see previews/plan_view.png)
  LEFT column  : Feetech FE-URT-2, raised on 8 mm standoffs so the MPU6050 IMU
                 can sit flat on the carrier underneath it (IMU ends up ~9 mm from
                 the robot's centre line).  USB-C end up, terminal block end down.
                 3-pin servo ports face the left wall (servo cables leave there).
  GAP (7.4 mm) : holds the two cover screw posts and the IMU / GPIO jumper wires.
  RIGHT column : Pi Zero 2W on top of PiSugar 3 + LiPo, camera end up, micro-USB
                 ports + PiSugar USB-C/button face the right wall (long slot).
  BAY (bottom of the left column): rocker switch + DC-jack cradle on the bottom
                 end wall (a SIDE wall, not the back face).
"""
import json
import os
import sys

import numpy as np
import trimesh
import manifold3d as M
from shapely.geometry import Polygon, box as sbox, Point
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
REF_BATTERY = os.path.join(HERE, "..", "ref", "xgoduck_hardware", "structure", "battery.stl")
OUT_PREVIEW = os.path.join(HERE, "previews")

# =============================================================================
# 1. MEASURED values  (read from battery.stl by slicing it - verified)
# =============================================================================
BOSS_TIP_Y = -24.6            # y of the boss tips (= rail tips) -> pack frame w = 0
BOSS_OD = 5.5                 # boss outer diameter (6.0 for the top-middle one)
# (x, z, pilot_depth_from_tip).  NOTE: only 3 of the 4 bosses really have a pilot
# hole; boss 4 is a solid pin.  Hole depths measured by slicing:
BOSSES = {
    "B1": dict(x=19.1, z=33.2, pilot=3.9),    # top right (as seen in battery frame, +x)
    "B2": dict(x=-8.9, z=33.2, pilot=5.9),    # top middle
    "B3": dict(x=-24.1, z=-17.6, pilot=7.9),  # bottom, -x side
    "B4": dict(x=24.1, z=-18.9, pilot=0.0),   # bottom, +x side: NO HOLE (solid pin)
}
PILOT_D = 2.8                 # M3 self-tapping pilot in the bosses
BOX_X = (-26.9, 27.0)         # battery box extents
BOX_Z = (-35.0, 36.5)

# =============================================================================
# 2. ASSUMED values  (TO MEASURE - see README "What to measure")
# =============================================================================
# --- Feetech FE-URT-2 -------------------------------------------------------
URT_L, URT_W = 55.8, 36.6     # PCB length (terminal<->USB-C) x width            TO MEASURE
URT_PCB_T = 1.6               # PCB thickness
URT_TOTAL_H = 11.5            # PCB underside -> top of tallest connector        TO MEASURE
URT_HOLE_D = 3.2              # mounting hole diameter                           TO MEASURE
URT_HOLE_INSET = 3.5          # hole centre distance from the edges              TO MEASURE
# --- Raspberry Pi Zero 2 W + PiSugar 3 stack ---------------------------------
PI_L, PI_W = 65.0, 30.0       # (datasheet)
PI_HOLE_DX, PI_HOLE_DY = 58.0, 23.0   # M2.5 hole pattern (datasheet), 3.5 inset
PI_STACK_H = 16.0             # underside of LiPo -> top of Pi parts/header       TO MEASURE
LIPO_T = 6.0                  # LiPo thickness = distance LiPo underside -> PiSugar PCB underside  TO MEASURE
LIPO_L, LIPO_W = 50.0, 24.0   # LiPo footprint (must stay clear of the 4 holes)  TO MEASURE
PI_USB_PX = 41.4              # micro-USB "USB" port position from the SD end    (datasheet)
PI_PWR_PX = 54.0              # micro-USB "PWR" port position from the SD end    (datasheet)
# --- MPU6050 breakout (GY-521 style) -----------------------------------------
IMU_L, IMU_W = 21.0, 16.0     # PCB size                                          TO MEASURE
IMU_T = 1.6
IMU_TOP_PARTS = 1.2           # chip/solder height above PCB top                  TO MEASURE
IMU_HOLE_D = 3.0              # mounting holes                                    TO MEASURE
IMU_HOLE_SPAN = 15.5          # centre-to-centre of the two holes (along L)       TO MEASURE
IMU_SCREW = "M2.5"            # self-tapping into 2.2 mm pilots
# --- optional rocker switch / DC jack ---------------------------------------
ROCKER_CUT_W, ROCKER_CUT_H = 19.0, 13.0   # panel cut-out                          TO MEASURE
ROCKER_BODY_DEPTH = 15.0      # behind-panel depth                                 TO MEASURE
JACK_D = 12.0                 # barrel-jack adapter body diameter                  TO MEASURE
JACK_LEN_IN = 14.0            # how far the jack body reaches into the pack        TO MEASURE
# --- screws --------------------------------------------------------------------
M3_CLEAR = 3.4                # clearance hole for M3
M3_PILOT = 2.8                # self-tapping pilot for M3 in PLA
M25_PILOT = 2.2               # self-tapping pilot for M2.5 in PLA
M3_HEAD_D, M3_HEAD_H = 5.8, 2.4   # round/pan head (ASSUMED)                       TO MEASURE
M25_HEAD_D, M25_HEAD_H = 4.8, 2.0

# =============================================================================
# 3. DESIGN parameters
# =============================================================================
T = 1.6                       # shell wall / roof thickness
CLR = 0.3                     # clearance around every board (per side)
PLATE_T = 1.6                 # carrier plate thickness
FIT = 0.25                    # carrier plate <-> cover wall slip fit
GAP = 7.4                     # clear space between the two board columns
TOP_GUTTER = 7.2              # free space above the Pi stack (ribbon exit, B1 screw head)
V0 = 0.5                      # pack centre in battery z (chosen so boss B1 head fits in the gutter)
U0 = 2.0                      # pack centre in battery x (+2: keeps boss B2's screw head clear of the URT-2 corner post)
R_PLAN = 5.5                  # outer plan-view corner radius
R_EDGE = 8.0                  # radius of the rounded back edge (45 deg max -> no supports)
W_RIM = 0.3                   # cover rim stands 0.3 mm off the boss-tip plane
URT_STANDOFF = 8.5            # PCB underside height above carrier top face -> IMU fits under it
IMU_POST_H = 3.6
PI_POST_OD, URT_POST_OD, IMU_POST_OD = 5.4, 6.0, 5.0
STACK_BOTTOM = 4.2            # LiPo underside height (clears the 3.8 mm screw heads of the carrier)
HEADER_CLEAR = 3.0            # free height above the Pi stack for jumper-wire connectors
CBORE_DEPTH = 0.2             # centring pocket; head seat stays 1.4 mm above the bottom face (engagement maths)
CBORE_D = 5.9                 # head pocket diameter (round head 5.6 assumed)
COVER_POST_OD = 6.4           # carrier posts for the two cover screws (M3, user's M3x16)
COVER_BOSS_OD = 7.6
COVER_POST_V = (24.2, 32.0)   # v of the two cover screw posts (gap, above the URT-2 side connectors)
COVER_POST_TOP = 14.0         # w where carrier post and cover boss meet
COVER_CBORE_D, COVER_CBORE_DEPTH = 6.2, 2.8
SERVO_NOTCH_V = (-10.0, 30.0)  # (pack v) extent of the servo cable slot on the left wall
PLA_DENSITY = 1.24            # g/cm3
SEG = 48                      # circle segments

# =============================================================================
# derived layout (pack frame)
# =============================================================================
PI_COL_W = PI_W + 2 * CLR
PI_COL_L = PI_L + 2 * CLR
URT_COL_W = URT_W + 2 * CLR
URT_COL_L = URT_L + 2 * CLR
W_IN = PI_COL_W + GAP + URT_COL_W                       # cavity width
H_IN = PI_COL_L + TOP_GUTTER                            # cavity height
W_OUT, H_OUT = W_IN + 2 * T, H_IN + 2 * T
D = STACK_BOTTOM + PI_STACK_H + HEADER_CLEAR + T        # total depth behind boss tips
H_C = R_EDGE * np.cos(np.pi / 4)                        # height of the vertical-wall start below the roof
R_CAV = R_PLAN - T                                      # cavity plan corner radius (Pi board has r=3)

URT_COL = (-W_IN / 2, -W_IN / 2 + URT_COL_W, H_IN / 2 - URT_COL_L, H_IN / 2)       # u0,u1,v0,v1 (pocket)
PI_COL = (W_IN / 2 - PI_COL_W, W_IN / 2, -H_IN / 2, -H_IN / 2 + PI_COL_L)
GAP_U = (URT_COL[1], PI_COL[0])
GAP_CENTER_U = 0.5 * (GAP_U[0] + GAP_U[1])
BAY_V = (-H_IN / 2, URT_COL[2])                          # free zone below the URT-2

# board outlines (actual board = pocket inset by CLR)
URT_BRD = (URT_COL[0] + CLR, URT_COL[1] - CLR, URT_COL[2] + CLR, URT_COL[3] - CLR)
PI_BRD = (PI_COL[0] + CLR, PI_COL[1] - CLR, PI_COL[2] + CLR, PI_COL[3] - CLR)

URT_W0 = PLATE_T + URT_STANDOFF                          # URT PCB underside (w)
STACK_TOP = STACK_BOTTOM + PI_STACK_H
PI_POST_TOP = STACK_BOTTOM + LIPO_T                      # PiSugar PCB underside
FEATURE_W = 8.5                                          # w of centre of wall cut-outs / jack / rocker

BOSS_PACK = {k: (b["x"] - U0, b["z"] - V0) for k, b in BOSSES.items()}


def pi_xy(px, py):
    """Pi local (px along 65 mm from SD end, py across from ports edge) -> pack (u,v)."""
    return (PI_BRD[1] - py, PI_BRD[2] + px)


def urt_xy(ax, ay):
    """URT local (ax along 55.8 mm from terminal end, ay across from the servo-port (left) edge)."""
    return (URT_BRD[0] + ay, URT_BRD[2] + ax)


IMU_C = (URT_BRD[1] - 0.5 - CLR - IMU_W / 2, -V0)   # near robot centre (v = -V0  <=>  battery z = 0)
# IMU: long axis along v, header edge toward +u (gap)

# =============================================================================
# geometry helpers (manifold3d)
# =============================================================================
def rrect(u0, u1, v0, v1, r, n=16):
    r = max(0.05, min(r, (u1 - u0) / 2 - 1e-3, (v1 - v0) / 2 - 1e-3))
    pts = []
    for cx, cy, a0 in [(u1 - r, v1 - r, 0), (u0 + r, v1 - r, 90), (u0 + r, v0 + r, 180), (u1 - r, v0 + r, 270)]:
        for a in np.linspace(a0, a0 + 90, n):
            pts.append((cx + r * np.cos(np.radians(a)), cy + r * np.sin(np.radians(a))))
    return Polygon(pts)


def prism(poly, z0, z1):
    """Extrude a shapely polygon (with holes) between z0 and z1."""
    polys = [poly] if poly.geom_type == "Polygon" else list(poly.geoms)
    cs = None
    for p in polys:
        rings = [np.array(p.exterior.coords)[:-1]] + [np.array(h.coords)[:-1] for h in p.interiors]
        c = M.CrossSection(rings, M.FillRule.EvenOdd)
        cs = c if cs is None else cs + c
    return cs.extrude(z1 - z0).translate([0, 0, z0])


def bx(u0, u1, v0, v1, w0, w1):
    return M.Manifold.cube([u1 - u0, v1 - v0, w1 - w0]).translate([u0, v0, w0])


def cyl(u, v, w0, w1, d, seg=SEG):
    return M.Manifold.cylinder(w1 - w0, d / 2, d / 2, seg).translate([u, v, w0])


def to_tm(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3].copy(), np.asarray(mesh.tri_verts).copy(), process=False)


def union(items):
    items = [i for i in items if i is not None]
    return M.Manifold.batch_boolean(items, M.OpType.Add)


def ring_pts(d, w, n=12):
    p = rrect(-W_OUT / 2 + d, W_OUT / 2 - d, -H_OUT / 2 + d, H_OUT / 2 - d, max(R_PLAN - d, 0.8), n)
    xy = np.array(p.exterior.coords)[:-1]
    return np.c_[xy, np.full(len(xy), w)]


def ring_pts_in(d, w, n=12):
    p = rrect(-W_OUT / 2 + d, W_OUT / 2 - d, -H_OUT / 2 + d, H_OUT / 2 - d, max(R_CAV - (d - T), 0.8), n)
    xy = np.array(p.exterior.coords)[:-1]
    return np.c_[xy, np.full(len(xy), w)]


# =============================================================================
# CARRIER
# =============================================================================
def plate_outline():
    d = T + FIT
    return rrect(-W_OUT / 2 + d, W_OUT / 2 - d, -H_OUT / 2 + d, H_OUT / 2 - d, R_CAV - FIT)


def keepouts():
    """2D regions (shapely) where the plate must stay solid / no lightening windows."""
    ko = []
    for k, (u, v) in BOSS_PACK.items():
        ko.append(Point(u, v).buffer(4.6))
    for ax, ay in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET),
                   (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        ko.append(Point(*urt_xy(ax, ay)).buffer(4.2))
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            ko.append(Point(*pi_xy(px, py)).buffer(4.0))
    for v in COVER_POST_V:
        ko.append(Point(GAP_CENTER_U, v).buffer(4.6))
    ko.append(sbox(IMU_C[0] - IMU_W / 2 - 3, IMU_C[0] + IMU_W / 2 + 3, IMU_C[1] - IMU_L / 2 - 3, IMU_C[1] + IMU_L / 2 + 3))
    # jack cradle + rocker bay edge
    ko.append(sbox(-W_OUT / 2, URT_COL[0] + 15, BAY_V[0], BAY_V[0] + 6))
    # keep the strip that rests on the box rails solid (|x| 20.5..27.5) -> good contact
    for s in (-1, 1):
        ko.append(sbox(min(s * 20.5, s * 27.5) - U0, max(s * 20.5, s * 27.5) - U0, -H_OUT, H_OUT))
    return unary_union(ko)


def lightening_windows():
    outline = plate_outline().buffer(-3.5)
    free = outline.difference(keepouts().buffer(0.8))
    wins = []
    size, pitch = 9.0, 12.0
    nx, ny = int(W_OUT // pitch) + 1, int(H_OUT // pitch) + 1
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            c = (i * pitch, j * pitch + (pitch / 2 if i % 2 else 0))
            w = rrect(c[0] - size / 2, c[0] + size / 2, c[1] - size / 2, c[1] + size / 2, 2.5, 6)
            if free.contains(w):
                wins.append(w)
    return wins


def build_carrier(with_windows=True, use_b4_screw=False):
    parts = [prism(plate_outline(), 0, PLATE_T)]
    # --- URT-2 standoffs (M3 self-tapping)
    for ax, ay in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET),
                   (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        u, v = urt_xy(ax, ay)
        parts.append(cyl(u, v, PLATE_T - 0.2, URT_W0, URT_POST_OD))
    # --- Pi/PiSugar stack standoffs (M2.5)
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            u, v = pi_xy(px, py)
            parts.append(cyl(u, v, PLATE_T - 0.2, PI_POST_TOP, PI_POST_OD))
    # --- IMU posts + corner locators
    for s in (-1, 1):
        parts.append(cyl(IMU_C[0], IMU_C[1] + s * IMU_HOLE_SPAN / 2, PLATE_T - 0.2, PLATE_T + IMU_POST_H, IMU_POST_OD))
    iu0, iu1 = IMU_C[0] - IMU_W / 2 - CLR, IMU_C[0] + IMU_W / 2 + CLR
    iv0, iv1 = IMU_C[1] - IMU_L / 2 - CLR, IMU_C[1] + IMU_L / 2 + CLR
    tabh = PLATE_T + IMU_POST_H + 1.0
    for cu, su in ((iu0, -1), (iu1, 1)):          # IMU corner locators: small L-shaped walls just outside the pocket
        for cv, sv in ((iv0, -1), (iv1, 1)):
            t_ = 1.2
            ua, ub = sorted((cu, cu + su * t_))
            va, vb = sorted((cv - sv * 5.0, cv + sv * t_))
            parts.append(bx(ua, ub, va, vb, PLATE_T - 0.1, tabh))
            ua, ub = sorted((cu - su * 5.0, cu + su * t_))
            va, vb = sorted((cv, cv + sv * t_))
            parts.append(bx(ua, ub, va, vb, PLATE_T - 0.1, tabh))
    # --- cover screw posts (in the gap)
    for v in COVER_POST_V:
        parts.append(cyl(GAP_CENTER_U, v, PLATE_T - 0.2, COVER_POST_TOP, COVER_POST_OD))
    # --- DC jack cradle: two saddles carrying a cylinder along v at (JACK_U, FEATURE_W); zip-tie slots
    for v0 in JACK_SADDLE_V:
        saddle = bx(JACK_U - 6.5, JACK_U + 6.5, v0, v0 + 2.0, PLATE_T - 0.2, FEATURE_W)
        cut = M.Manifold.cylinder(2.4, (JACK_D + 0.4) / 2, (JACK_D + 0.4) / 2, SEG).rotate([-90, 0, 0]).translate([JACK_U, v0 - 0.2, FEATURE_W])
        parts.append(saddle - cut)
    carrier = union(parts)
    # --- holes
    cuts = []
    for k, (u, v) in BOSS_PACK.items():
        if k == "B4" and not use_b4_screw:
            continue
        cuts.append(cyl(u, v, -0.1, PLATE_T + 0.1, M3_CLEAR))
        cuts.append(cyl(u, v, PLATE_T - CBORE_DEPTH, PLATE_T + 0.2, CBORE_D))
    for ax, ay in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET),
                   (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        u, v = urt_xy(ax, ay)
        cuts.append(cyl(u, v, URT_W0 - 6.4, URT_W0 + 0.1, M3_PILOT, 24))
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            u, v = pi_xy(px, py)
            cuts.append(cyl(u, v, PI_POST_TOP - 7.8, PI_POST_TOP + 0.1, M25_PILOT, 24))
    for s in (-1, 1):
        cuts.append(cyl(IMU_C[0], IMU_C[1] + s * IMU_HOLE_SPAN / 2, 0.6, PLATE_T + IMU_POST_H + 0.1, M25_PILOT, 24))
    for v in COVER_POST_V:
        cuts.append(cyl(GAP_CENTER_U, v, COVER_POST_TOP - 9.0, COVER_POST_TOP + 0.1, M3_PILOT, 24))
    # --- lightening windows
    if with_windows:
        for w in lightening_windows():
            cuts.append(prism(w, -0.1, PLATE_T + 0.1))
    # --- snap grooves in the plate edge (top and bottom ends), V-section 0.6 deep
    for u in BUMPS_TOP_U:
        cuts.append(groove(u, +1))
    for u in BUMPS_BOT_U:
        cuts.append(groove(u, -1))
    carrier = carrier - union(cuts)
    # 45-degree lead-in chamfer on the top edge of the plate so the cover slides on easily
    return carrier


JACK_U = URT_COL[0] + 8.5
ROCKER_U = URT_COL[1] - 2.0 - ROCKER_CUT_W / 2
JACK_SADDLE_V = (BAY_V[0] + 2.5, BAY_V[0] + 9.5)
BUMPS_TOP_U = (-2.0, 6.0)
BUMPS_BOT_U = (1.5, 9.0)
BUMP_H = 0.5
BUMP_L = 5.0
BUMP_W = 0.8      # base width along w
BUMP_W_CENTER = 0.8


def _tri_prism_along_u(tri_vw, u, length):
    """Triangle given in (v, w) extruded along u, centred on u."""
    cs = M.CrossSection([np.array(tri_vw, dtype=float)], M.FillRule.EvenOdd)
    mesh = cs.extrude(length).to_mesh()
    p = np.array(mesh.vert_properties)[:, :3]
    t = np.array(mesh.tri_verts)
    p2 = np.c_[p[:, 2] + u - length / 2, p[:, 0], p[:, 1]]     # (x,y,z) -> (u,v,w): handedness flips, so reverse winding
    return M.Manifold(M.Mesh(vert_properties=p2.astype(np.float32), tri_verts=t[:, ::-1].astype(np.uint32)))


def groove(u, side):
    """V-groove on the plate edge at the top (side=+1) or bottom (-1) end wall."""
    ve = side * (H_OUT / 2 - T - FIT)
    g = BUMP_H + 0.1
    hw = 0.7
    tri = [(ve + side * 0.3, BUMP_W_CENTER - hw - 0.15), (ve - side * g, BUMP_W_CENTER), (ve + side * 0.3, BUMP_W_CENTER + hw + 0.15)]
    if side < 0:
        tri = tri[::-1]
    return _tri_prism_along_u(tri, u, BUMP_L + 1.0)


# =============================================================================
# COVER (built in the pack frame, exported rotated into print orientation)
# =============================================================================
def build_cover():
    # ---- outer shell (convex hull of rings)  ---------------------------------
    pts = [ring_pts(0, W_RIM)]
    pts.append(ring_pts(0, D - H_C))
    for a in np.radians(np.linspace(0, 45, 7)):
        d = R_EDGE * (1 - np.cos(a))
        h = H_C - R_EDGE * np.sin(a)
        pts.append(ring_pts(d, D - h))
    outer = M.Manifold.hull_points(np.vstack(pts))
    # ---- inner cavity ----------------------------------------------------------
    Ri = R_EDGE - T
    a_end = np.arcsin((H_C - T) / Ri)
    ipts = [ring_pts_in(T, W_RIM - 1.0), ring_pts_in(T, D - H_C)]
    for a in np.linspace(0, a_end, 6):
        d = R_EDGE - Ri * np.cos(a)
        h = H_C - Ri * np.sin(a)
        ipts.append(ring_pts_in(d, D - h))
    inner = M.Manifold.hull_points(np.vstack(ipts))
    shell = outer - inner

    add = []
    sub = []
    roof_in = D - T
    # ---- cover screw bosses + spine rib
    for v in COVER_POST_V:
        add.append(cyl(GAP_CENTER_U, v, COVER_POST_TOP, roof_in + 0.4, COVER_BOSS_OD))
        sub.append(cyl(GAP_CENTER_U, v, COVER_POST_TOP - 0.1, D + 0.2, M3_CLEAR, 24))
        sub.append(cyl(GAP_CENTER_U, v, D - COVER_CBORE_DEPTH, D + 0.2, COVER_CBORE_D, 32))
    add.append(bx(GAP_CENTER_U - 0.6, GAP_CENTER_U + 0.6, COVER_POST_V[0], COVER_POST_V[1], roof_in - 3.5, roof_in + 0.4))
    # ---- snap bumps on the inner face of the top / bottom walls
    for u in BUMPS_TOP_U:
        add.append(bump(u, +1))
    for u in BUMPS_BOT_U:
        add.append(bump(u, -1))
    # ---- vent slots in the roof (2 x 18), over the Pi stack and the URT-2
    for u0 in np.linspace(PI_BRD[0] + 5, PI_BRD[1] - 5, 5):
        for v0 in (-17.0, 7.0):
            sub.append(bx(u0 - 1.0, u0 + 1.0, v0 - 8.5, v0 + 8.5, D - T - 0.5, D + 0.5))
    for u0 in np.linspace(URT_BRD[0] + 5, URT_BRD[1] - 5, 6):
        for v0 in (-4.0, 22.0):
            if URT_COL[2] + 3 < v0 - 7 and v0 + 7 < URT_COL[3] - 3:
                sub.append(bx(u0 - 1.0, u0 + 1.0, v0 - 7, v0 + 7, D - T - 0.5, D + 0.5))
    # ---- wall cut-outs (all open towards the rim -> printable without supports)
    ww = T + 3.0
    xr, xl = W_OUT / 2, -W_OUT / 2
    yt, yb = H_OUT / 2, -H_OUT / 2
    # right wall: Pi micro-USB / PiSugar USB-C + button / HDMI, generous long slot
    sub.append(bx(xr - ww, xr + 1, PI_BRD[2] + 3.0, PI_BRD[3] - 3.0, -1.0, 19.0))
    # left wall: servo cable slot
    sub.append(bx(xl - 1, xl + ww, SERVO_NOTCH_V[0], SERVO_NOTCH_V[1], -1.0, 19.0))
    # top wall: camera ribbon (Pi column) and USB-C plug (URT column)
    pc = 0.5 * (PI_BRD[0] + PI_BRD[1])
    uc = 0.5 * (URT_BRD[0] + URT_BRD[1])
    sub.append(bx(pc - 11.0, pc + 11.0, yt - ww, yt + 1, -1.0, 17.0))
    sub.append(bx(uc - 7.0, uc + 7.0, yt - ww, yt + 1, -1.0, 16.0))
    # bottom wall: SD-card notch under the Pi column
    sub.append(bx(pc - 9.0, pc + 9.0, yb - 1, yb + ww, -1.0, 16.0))
    # bottom wall: rocker switch cut-out (closed rectangle) + DC-jack hole
    ru = ROCKER_U
    sub.append(bx(ru - ROCKER_CUT_W / 2, ru + ROCKER_CUT_W / 2, yb - 1, yb + ww, FEATURE_W - ROCKER_CUT_H / 2, FEATURE_W + ROCKER_CUT_H / 2))
    ju = JACK_U
    sub.append(M.Manifold.cylinder(ww + 1, (JACK_D + 0.6) / 2, (JACK_D + 0.6) / 2, SEG).rotate([-90, 0, 0]).translate([ju, yb - 1, FEATURE_W]))
    cover = (shell + union(add)) - union([s for s in sub if s is not None])
    return cover




def bump(u, side):
    ve = side * (H_OUT / 2 - T)
    tri = [(ve + side * 0.01, BUMP_W_CENTER - BUMP_W / 2), (ve - side * BUMP_H, BUMP_W_CENTER), (ve + side * 0.01, BUMP_W_CENTER + BUMP_W / 2)]
    if side < 0:
        tri = tri[::-1]
    wall = bx(u - BUMP_L / 2, u + BUMP_L / 2, min(ve, ve + side * 0.8), max(ve, ve + side * 0.8),
              BUMP_W_CENTER - BUMP_W / 2, BUMP_W_CENTER + BUMP_W / 2)
    return _tri_prism_along_u(tri, u, BUMP_L) + wall


# =============================================================================
# ELECTRONICS (simple boxes for the assembly view + collision check)
# =============================================================================
def electronics():
    """returns list of (name, Manifold, rgb)."""
    out = []
    # --- Pi stack
    def pi_box(px0, px1, py0, py1, w0, w1):
        (u0, v0), (u1, v1) = pi_xy(px0, py0), pi_xy(px1, py1)
        return bx(min(u0, u1), max(u0, u1), min(v0, v1), max(v0, v1), w0, w1)

    def holes_pi(m, w0, w1, d):
        for px in (3.5, PI_L - 3.5):
            for py in (3.5, PI_W - 3.5):
                u, v = pi_xy(px, py)
                m = m - cyl(u, v, w0 - 0.1, w1 + 0.1, d, 24)
        return m

    w = STACK_BOTTOM
    lipo = pi_box((PI_L - LIPO_L) / 2, (PI_L + LIPO_L) / 2, (PI_W - LIPO_W) / 2, (PI_W + LIPO_W) / 2, w, w + LIPO_T)
    out.append(("LiPo (PiSugar 3)", lipo, (0.35, 0.45, 0.85)))
    w += LIPO_T
    psb = holes_pi(prism(rrect(PI_BRD[0], PI_BRD[1], PI_BRD[2], PI_BRD[3], 3.0), w, w + 1.6), w, w + 1.6, 2.7)
    out.append(("PiSugar 3 PCB", psb, (0.15, 0.55, 0.3)))
    w += 1.6 + 2.4
    pib = holes_pi(prism(rrect(PI_BRD[0], PI_BRD[1], PI_BRD[2], PI_BRD[3], 3.0), w, w + 1.6), w, w + 1.6, 2.7)
    out.append(("Pi Zero 2W", pib, (0.1, 0.6, 0.25)))
    # spacer/pogo between PiSugar and Pi (ASSUMED) -> small blocks at the hole positions
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            u, v = pi_xy(px, py)
            out.append(("spacer", cyl(u, v, w - 2.4, w, 5.0), (0.8, 0.7, 0.2)))
    w_top = w + 1.6
    parts = []
    parts.append(pi_box(7.1, 57.9, 24.4, 29.5, w_top, STACK_TOP))             # 2x20 header
    parts.append(pi_box(PI_USB_PX - 4, PI_USB_PX + 4, -1.0, 4.6, w_top, w_top + 2.6))  # USB
    parts.append(pi_box(PI_PWR_PX - 4, PI_PWR_PX + 4, -1.0, 4.6, w_top, w_top + 2.6))  # PWR
    parts.append(pi_box(12.4 - 5.6, 12.4 + 5.6, -0.5, 6.5, w_top, w_top + 3.2))        # mini HDMI
    parts.append(pi_box(24, 36, 9, 21, w_top, w_top + 1.0))                              # SoC shield
    out.append(("Pi connectors / header", union(parts), (0.7, 0.7, 0.72)))
    # --- URT-2
    def u_box(ax0, ax1, ay0, ay1, w0, w1):
        (u0, v0), (u1, v1) = urt_xy(ax0, ay0), urt_xy(ax1, ay1)
        return bx(min(u0, u1), max(u0, u1), min(v0, v1), max(v0, v1), w0, w1)

    w0 = URT_W0
    pcb = prism(rrect(URT_BRD[0], URT_BRD[1], URT_BRD[2], URT_BRD[3], 3.0), w0, w0 + URT_PCB_T)
    for ax, ay in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET),
                   (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        u, v = urt_xy(ax, ay)
        pcb = pcb - cyl(u, v, w0 - 0.1, w0 + 2, URT_HOLE_D, 24)
    out.append(("FE-URT-2 PCB", pcb, (0.1, 0.25, 0.7)))
    wt = w0 + URT_PCB_T
    hmax = w0 + URT_TOTAL_H
    c = []
    c.append(u_box(0.0, 10.5, 8.2, 28.4, wt, hmax))                    # 4-screw terminal (blue) at terminal end
    c.append(u_box(19, 26.6, -0.8, 5.0, wt, wt + 6.0))                 # 3-pin servo port 1 (left edge)
    c.append(u_box(29, 36.6, -0.8, 5.0, wt, wt + 6.0))                 # 3-pin servo port 2
    c.append(u_box(18, 28.2, URT_W - 5.0, URT_W + 0.8, wt, wt + 6.0))  # 4-pin port 1 (gap side)
    c.append(u_box(31, 41.2, URT_W - 5.0, URT_W + 0.8, wt, wt + 6.0))  # 4-pin port 2
    c.append(u_box(URT_L - 6.5, URT_L + 1.0, 13.5, 22.5, wt, wt + 3.3))  # USB-C at the top end
    c.append(u_box(46.0, 54.0, 8.5, 12.5, wt, wt + 3.5))              # slide switch
    c.append(u_box(43.0, 45.5, 17.0, 29.7, wt, wt + 8.5))              # 5-pin header
    out.append(("FE-URT-2 connectors", union(c), (0.85, 0.85, 0.9)))
    # --- IMU (long axis along v, flat on its posts)
    iw = PLATE_T + IMU_POST_H
    imu = prism(rrect(IMU_C[0] - IMU_W / 2, IMU_C[0] + IMU_W / 2, IMU_C[1] - IMU_L / 2, IMU_C[1] + IMU_L / 2, 1.0), iw, iw + IMU_T)
    for s in (-1, 1):
        imu = imu - cyl(IMU_C[0], IMU_C[1] + s * IMU_HOLE_SPAN / 2, iw - 0.1, iw + IMU_T + 0.1, IMU_HOLE_D, 24)
    chip = bx(IMU_C[0] - 2, IMU_C[0] + 2, IMU_C[1] - 2, IMU_C[1] + 2, iw + IMU_T, iw + IMU_T + IMU_TOP_PARTS)
    out.append(("MPU6050 IMU", imu + chip, (0.85, 0.1, 0.1)))
    # --- optional rocker + jack
    # --- screws (for collision checks + preview).  Shank diameters are the thread minor diameters.
    def screw(u, v, w_seat, head_d, head_h, L, shank_d):
        return cyl(u, v, w_seat, w_seat + head_h, head_d) + cyl(u, v, w_seat - L, w_seat + 0.01, shank_d, 24)
    sb = []
    for k, L in (("B1", 5.0), ("B2", 6.0), ("B3", 8.0)):
        u, v = BOSS_PACK[k]
        sb.append(screw(u, v, PLATE_T - CBORE_DEPTH, M3_HEAD_D, M3_HEAD_H, L, 2.5))
    out.append(("screws: boss M3 (x3)", union(sb), (0.25, 0.25, 0.28)))
    sc = []
    for ax_, ay_ in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET),
                     (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        u, v = urt_xy(ax_, ay_)
        sc.append(screw(u, v, URT_W0 + URT_PCB_T, M3_HEAD_D, M3_HEAD_H, 8.0, 2.5))
    out.append(("screws: URT-2 M3x8 (x4)", union(sc), (0.25, 0.25, 0.28)))
    sp = []
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            u, v = pi_xy(px, py)
            sp.append(screw(u, v, w_top, M25_HEAD_D, M25_HEAD_H, 12.0, 2.0))
    out.append(("screws: Pi M2.5x12 (x4)", union(sp), (0.25, 0.25, 0.28)))
    si = [screw(IMU_C[0], IMU_C[1] + s_ * IMU_HOLE_SPAN / 2, PLATE_T + IMU_POST_H + IMU_T, M25_HEAD_D, M25_HEAD_H, 6.0, 2.0) for s_ in (-1, 1)]
    out.append(("screws: IMU M2.5x6 (x2)", union(si), (0.25, 0.25, 0.28)))
    cs_ = [screw(GAP_CENTER_U, v_, D - COVER_CBORE_DEPTH, M3_HEAD_D, M3_HEAD_H, 16.0, 2.5) for v_ in COVER_POST_V]
    out.append(("screws: cover M3x16 (x2)", union(cs_), (0.25, 0.25, 0.28)))
    yb = -H_OUT / 2
    rk = bx(ROCKER_U - ROCKER_CUT_W / 2, ROCKER_U + ROCKER_CUT_W / 2, yb, yb + ROCKER_BODY_DEPTH,
            FEATURE_W - ROCKER_CUT_H / 2, FEATURE_W + ROCKER_CUT_H / 2)
    rk = rk + bx(ROCKER_U - ROCKER_CUT_W / 2 - 1.5, ROCKER_U + ROCKER_CUT_W / 2 + 1.5, yb - 2.2, yb,
                 FEATURE_W - ROCKER_CUT_H / 2 - 1.5, FEATURE_W + ROCKER_CUT_H / 2 + 1.5)
    out.append(("rocker switch (optional)", rk, (0.9, 0.5, 0.1)))
    jk = M.Manifold.cylinder(JACK_LEN_IN + 1.0, JACK_D / 2, JACK_D / 2, SEG).rotate([-90, 0, 0]).translate([JACK_U, yb - 1.0, FEATURE_W])
    out.append(("DC jack (optional)", jk, (0.1, 0.65, 0.3)))
    return out


# =============================================================================
# frame transforms / export
# =============================================================================
def pack_to_battery(mesh):
    """pack (u,v,w) -> battery (x,y,z)."""
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0] + U0, BOSS_TIP_Y - v[:, 2], v[:, 1] + V0]
    m.invert() if False else None
    m.fix_normals()
    return m


def battery_to_pack(mesh):
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0] - U0, v[:, 2] - V0, -v[:, 1] + BOSS_TIP_Y]
    m.fix_normals()
    return m


def cover_print_orientation(mesh):
    """Roof on the bed: rotate 180 deg about u -> (u, v, w) -> (u, -v, D - w)."""
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0], -v[:, 1], D - v[:, 2]]
    m.fix_normals()
    return m


def check(name, tm):
    print(f"  {name:22s} watertight={tm.is_watertight!s:5s} winding={tm.is_winding_consistent!s:5s} "
          f"vol={tm.volume / 1000:.2f} cm3  mass(solid PLA)={tm.volume / 1000 * PLA_DENSITY:.1f} g  bounds={np.round(tm.bounds[1] - tm.bounds[0], 1)}")


def inter_volume(a, b):
    """intersection volume of two trimesh meshes via manifold."""
    ma = M.Manifold(M.Mesh(np.asarray(a.vertices, dtype=np.float32), np.asarray(a.faces, dtype=np.uint32)))
    mb = M.Manifold(M.Mesh(np.asarray(b.vertices, dtype=np.float32), np.asarray(b.faces, dtype=np.uint32)))
    return (ma ^ mb).volume()


def to_man(tm):
    return M.Manifold(M.Mesh(np.asarray(tm.vertices, dtype=np.float32), np.asarray(tm.faces, dtype=np.uint32)))


# =============================================================================
# PREVIEW RENDERING  (small numpy z-buffer rasteriser + matplotlib overlays)
# =============================================================================
def _basis(f, up):
    f = np.asarray(f, float); f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u


def rasterize(objs, f, up, size=(1500, 1150), ssaa=2, margin=0.07, center=None, scale=None, edges=True, bg=(1, 1, 1)):
    """objs: list of dict(mesh, color, alpha).  alpha<1 -> ghosted (drawn after the opaque scene, front-most layer only).
    returns image (H,W,3 float), project(points)->(px,py) in final-image pixels."""
    f, r, u = _basis(f, up)
    Wp, Hp = size[0] * ssaa, size[1] * ssaa
    allv = np.vstack([o["mesh"].vertices for o in objs])
    P = np.c_[allv @ r, allv @ u, allv @ f]
    lo, hi = P.min(0), P.max(0)
    if scale is None:
        scale = min(Wp * (1 - 2 * margin) / (hi[0] - lo[0]), Hp * (1 - 2 * margin) / (hi[1] - lo[1]))
    if center is None:
        center = (lo + hi) / 2

    def proj(pts):
        q = np.c_[pts @ r, pts @ u, pts @ f]
        x = (q[:, 0] - center[0]) * scale + Wp / 2
        y = Hp / 2 - (q[:, 1] - center[1]) * scale
        return x, y, q[:, 2]

    keyL = -0.55 * f + 0.6 * u - 0.55 * r
    keyL /= np.linalg.norm(keyL)
    fillL = -0.3 * f + 0.2 * u + 0.8 * r
    fillL /= np.linalg.norm(fillL)

    img = np.ones((Hp, Wp, 3)) * np.array(bg)
    zb = np.full((Hp, Wp), np.inf)
    idb = np.full((Hp, Wp), -1, dtype=np.int32)

    def draw(o_list, zbuf, cbuf, idbuf, test_buf=None, alpha=1.0, id0=0):
        for oi, o in enumerate(o_list):
            m = o["mesh"]
            V = m.vertices
            x, y, z = proj(V)
            F = m.faces
            N = m.face_normals
            col = np.asarray(o["color"], float)
            a = o.get("alpha", 1.0)
            facing = N @ f
            Nn = np.where(facing[:, None] > 0, -N, N)
            lam = 0.46 + 0.50 * np.clip(Nn @ keyL, 0, 1) + 0.22 * np.clip(Nn @ fillL, 0, 1)
            lam = np.clip(lam, 0, 1.15)
            for t in range(len(F)):
                i0, i1, i2 = F[t]
                x0, x1, x2 = x[i0], x[i1], x[i2]
                y0, y1, y2 = y[i0], y[i1], y[i2]
                xmin = max(int(np.floor(min(x0, x1, x2))), 0); xmax = min(int(np.ceil(max(x0, x1, x2))), Wp - 1)
                ymin = max(int(np.floor(min(y0, y1, y2))), 0); ymax = min(int(np.ceil(max(y0, y1, y2))), Hp - 1)
                if xmin > xmax or ymin > ymax:
                    continue
                den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
                if abs(den) < 1e-12:
                    continue
                gx, gy = np.meshgrid(np.arange(xmin, xmax + 1) + 0.5, np.arange(ymin, ymax + 1) + 0.5)
                l0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / den
                l1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / den
                l2 = 1 - l0 - l1
                inside = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
                if not inside.any():
                    continue
                zz = l0 * z[i0] + l1 * z[i1] + l2 * z[i2]
                sl = (slice(ymin, ymax + 1), slice(xmin, xmax + 1))
                ok = inside & (zz < zbuf[sl])
                if test_buf is not None:
                    ok &= zz < test_buf[sl]
                if not ok.any():
                    continue
                zbuf[sl][ok] = zz[ok]
                cbuf[sl][ok] = np.clip(col * lam[t], 0, 1)
                idbuf[sl][ok] = id0 + oi

    opaque = [o for o in objs if o.get("alpha", 1.0) >= 1.0]
    ghost = [o for o in objs if o.get("alpha", 1.0) < 1.0]
    draw(opaque, zb, img, idb)
    if edges:
        gy_, gx_ = np.gradient(np.where(np.isfinite(zb), zb, np.nanmax(zb[np.isfinite(zb)]) + 5))
        mag = np.hypot(gx_, gy_)
        eid = np.zeros_like(zb, dtype=bool)
        eid[:, 1:] |= idb[:, 1:] != idb[:, :-1]
        eid[1:, :] |= idb[1:, :] != idb[:-1, :]
        thr = 0.9 * ssaa / max(scale, 1e-9) * 6.0 + 0.6
        e = (mag > thr) | eid
        img[e] = img[e] * 0.35
    if ghost:
        gz = np.full((Hp, Wp), np.inf)
        gc = np.zeros((Hp, Wp, 3))
        gi = np.full((Hp, Wp), -1, dtype=np.int32)
        draw(ghost, gz, gc, gi, test_buf=zb)
        al = np.array([ghost[i]["alpha"] if i >= 0 else 0 for i in gi.ravel()]).reshape(gi.shape) if False else None
        amap = np.zeros(gi.shape)
        for i, o in enumerate(ghost):
            amap[gi == i] = o["alpha"]
        msk = gi >= 0
        img[msk] = img[msk] * (1 - amap[msk, None]) + gc[msk] * amap[msk, None]
    # downsample
    out = img.reshape(size[1], ssaa, size[0], ssaa, 3).mean((1, 3))

    def project(pts):
        x, y, _ = proj(np.atleast_2d(pts))
        return x / ssaa, y / ssaa

    return np.clip(out, 0, 1), project


def _save(img, path, labels=(), project=None, title=None, dpi=110, extra=None):
    """labels: (text, point3d, side) with side 'L' or 'R' -> text in the margin, leader line to the point."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    Hh, Ww = img.shape[:2]
    fig = plt.figure(figsize=(Ww / dpi, Hh / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img, interpolation="bilinear")
    ax.set_xlim(0, Ww); ax.set_ylim(Hh, 0)
    ax.axis("off")
    placed = {"L": [], "R": []}
    items = []
    for text, pt, side in labels:
        x, y = project(np.array(pt, float))
        items.append((text, float(x[0]), float(y[0]), side))
    for side in ("L", "R"):
        its = sorted([i for i in items if i[3] == side], key=lambda i: i[2])
        last = -1e9
        for text, x, y, _ in its:
            ty = max(y, last + 46)
            last = ty
            tx = 12 if side == "L" else Ww - 12
            ax.annotate(text, (x, y), (tx, ty), fontsize=9, color="#111", ha="left" if side == "L" else "right", va="center",
                        arrowprops=dict(arrowstyle="-", color="#333", lw=0.8, shrinkA=0, shrinkB=0, connectionstyle="arc3,rad=0.0"),
                        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#888", lw=0.6, alpha=0.95))
            ax.plot([x], [y], "o", ms=3.5, color="#c0392b")
    if title:
        ax.text(14, 22, title, fontsize=12, weight="bold", color="#111", va="top")
    if extra:
        extra(ax, project)
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def _obj(mesh, color, alpha=1.0, shift=(0, 0, 0)):
    m = mesh.copy()
    m.apply_translation(shift)
    return dict(mesh=m, color=color, alpha=alpha)


def plan_view(path, carrier_tm, elec):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly, Rectangle, Circle
    fig, ax = plt.subplots(figsize=(11.5, 11))
    ax.set_aspect("equal")

    def poly(p, **kw):
        ax.add_patch(MPoly(np.array(p.exterior.coords), closed=True, **kw))

    # battery box footprint
    ax.add_patch(Rectangle((BOX_X[0] - U0, BOX_Z[0] - V0), BOX_X[1] - BOX_X[0], BOX_Z[1] - BOX_Z[0], fill=False, ls="--", ec="#c0392b", lw=1.6, zorder=20))
    ax.text(BOX_X[0] - U0 + 0.8, BOX_Z[0] - V0 + 1.2, "battery box outline", color="#c0392b", fontsize=8, zorder=21)
    # cover outline / cavity / plate
    poly(rrect(-W_OUT / 2, W_OUT / 2, -H_OUT / 2, H_OUT / 2, R_PLAN), fc="#f4f1ea", ec="#222", lw=1.6)
    poly(rrect(-W_IN / 2, W_IN / 2, -H_IN / 2, H_IN / 2, R_CAV), fc="#ffffff", ec="#888", lw=0.8)
    poly(plate_outline(), fc="#e8edf3", ec="#34495e", lw=1.0)
    for w in lightening_windows():
        poly(w, fc="white", ec="#34495e", lw=0.6)
    # pockets
    for (u0, u1, v0, v1), c, name in [(URT_COL, "#cfe0ff", "FE-URT-2 column"), (PI_COL, "#d6f0d6", "Pi + PiSugar column")]:
        ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=True, fc=c, ec="none", alpha=0.35))
    ax.add_patch(Rectangle((GAP_U[0], -H_IN / 2), GAP_U[1] - GAP_U[0], H_IN, fill=True, fc="#fff3c4", ec="none", alpha=0.6))
    ax.text(GAP_CENTER_U, -H_IN / 2 + 2, "gap: cover screws,\nIMU / GPIO wires", fontsize=7, ha="center", va="bottom", rotation=90)
    # boards
    poly(rrect(*URT_BRD[:2], *URT_BRD[2:], 3.0), fc="#3b6fd6", ec="#123", lw=1.2, alpha=0.85)
    poly(rrect(*PI_BRD[:2], *PI_BRD[2:], 3.0), fc="#2e9e4f", ec="#123", lw=1.2, alpha=0.85)
    lu0 = pi_xy((PI_L - LIPO_L) / 2, (PI_W - LIPO_W) / 2); lu1 = pi_xy((PI_L + LIPO_L) / 2, (PI_W + LIPO_W) / 2)
    ax.add_patch(Rectangle((min(lu0[0], lu1[0]), min(lu0[1], lu1[1])), abs(lu0[0] - lu1[0]), abs(lu0[1] - lu1[1]), fc="#6c7fd1", ec="#223", lw=0.8, alpha=0.7))
    ax.add_patch(Rectangle((IMU_C[0] - IMU_W / 2, IMU_C[1] - IMU_L / 2), IMU_W, IMU_L, fc="#d62828", ec="#400", lw=1.0, alpha=0.9))
    ax.text(IMU_C[0], IMU_C[1], "IMU\n(under\nURT-2)", color="w", fontsize=7, ha="center", va="center", weight="bold")
    ax.text(*(np.mean(URT_COL[:2]), np.mean(URT_COL[2:]) + 10), "FE-URT-2\n55.8 x 36.6", ha="center", va="center", fontsize=9, color="w", weight="bold")
    ax.text(*(np.mean(PI_COL[:2]) + 4, np.mean(PI_COL[2:]) + 8), "Pi Zero 2W\n+ PiSugar 3\n+ LiPo\n65 x 30", ha="center", va="center", fontsize=9, color="w", weight="bold")
    # posts
    for ax_, ay_ in [(URT_HOLE_INSET, URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_HOLE_INSET), (URT_HOLE_INSET, URT_W - URT_HOLE_INSET), (URT_L - URT_HOLE_INSET, URT_W - URT_HOLE_INSET)]:
        ax.add_patch(Circle(urt_xy(ax_, ay_), URT_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8))
    for px in (3.5, PI_L - 3.5):
        for py in (3.5, PI_W - 3.5):
            ax.add_patch(Circle(pi_xy(px, py), PI_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8))
    for s in (-1, 1):
        ax.add_patch(Circle((IMU_C[0], IMU_C[1] + s * IMU_HOLE_SPAN / 2), IMU_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8))
    for v in COVER_POST_V:
        ax.add_patch(Circle((GAP_CENTER_U, v), COVER_POST_OD / 2, fc="#e67e22", ec="#333", lw=0.8))
    ax.text(GAP_CENTER_U + 5, np.mean(COVER_POST_V), "2x M3x16\ncover screws", fontsize=7, va="center")
    # bosses
    for k, (u, v) in BOSS_PACK.items():
        scr = k != "B4"
        ax.add_patch(Circle((u, v), 2.95 if scr else 2.75, fc="#c0392b" if scr else "none", ec="#c0392b", lw=1.3, ls="-" if scr else ":"))
        ax.add_patch(Circle((u, v), BOSS_OD / 2, fill=False, ec="#c0392b", lw=0.8))
        ax.annotate(f"{k}  M3 screw\n(pilot {BOSSES[k]['pilot']} mm)" if scr else f"{k}  no hole:\nrest pad", (u, v), (u + (7 if u > 0 else -7), v + (4 if v > 0 else -4)), fontsize=7, color="#c0392b",
                    arrowprops=dict(arrowstyle="-", color="#c0392b", lw=0.6), ha="left" if u > 0 else "right")
    # jack + rocker
    ax.add_patch(Rectangle((ROCKER_U - ROCKER_CUT_W / 2, -H_OUT / 2), ROCKER_CUT_W, ROCKER_BODY_DEPTH, fc="#f39c12", ec="#333", alpha=0.85))
    ax.text(ROCKER_U, -H_OUT / 2 + 7, "rocker\n(optional)", fontsize=7, ha="center", va="center")
    ax.add_patch(Rectangle((JACK_U - JACK_D / 2, -H_OUT / 2), JACK_D, JACK_LEN_IN, fc="#27ae60", ec="#333", alpha=0.85))
    ax.text(JACK_U, -H_OUT / 2 + 7, "DC jack\n(optional)", fontsize=7, ha="center", va="center")
    # wall openings (arrows)
    def arrow(txt, xy, dxy, **kw):
        ax.annotate(txt, xy, (xy[0] + dxy[0], xy[1] + dxy[1]), fontsize=8, ha=kw.get("ha", "center"), va=kw.get("va", "center"),
                    arrowprops=dict(arrowstyle="<-", color="#555"), color="#222")
    arrow("Pi micro-USB + PiSugar\nUSB-C / button slot", (W_OUT / 2, 0), (11, 0), ha="left")
    arrow("servo cables\n(3-pin ports)", (-W_OUT / 2, 10), (-11, 0), ha="right")
    arrow("camera ribbon", (np.mean(PI_BRD[:2]), H_OUT / 2), (0, 6))
    arrow("USB-C plug\n(URT-2)", (np.mean(URT_BRD[:2]), H_OUT / 2), (0, 6))
    arrow("SD card", (np.mean(PI_BRD[:2]), -H_OUT / 2), (0, -6))
    ax.text(-W_OUT / 2 + 1, H_OUT / 2 + 8.5, "robot up (+z)", fontsize=8)
    ax.set_xlim(-W_OUT / 2 - 22, W_OUT / 2 + 24); ax.set_ylim(-H_OUT / 2 - 10, H_OUT / 2 + 12)
    ax.set_title(f"Plan view from behind the robot (cover removed)   pack {W_OUT:.1f} x {H_OUT:.1f} mm   u = robot x (right), v = robot z (up)", fontsize=10)
    ax.set_xlabel("u [mm]"); ax.set_ylabel("v [mm]")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def make_previews(c_tm, v_tm, elec, bat_p):
    os.makedirs(OUT_PREVIEW, exist_ok=True)
    C_CAR = (0.93, 0.93, 0.9)
    C_COV = (0.98, 0.78, 0.2)
    ecol = {n: col for n, _, col in elec}
    sel = lambda skip=(): [(n, m, col) for n, m, col in elec if n not in skip]
    print("rendering previews ...")
    # 1. exploded assembly
    f = (-0.5, 0.75, -0.8)
    up = (0, 0, 1)
    def off(n):
        if n == "MPU6050 IMU": return 9
        if n.startswith("screws: IMU"): return 20
        if n.startswith("screws: boss"): return 16
        if n.startswith("screws: URT") or n.startswith("screws: Pi"): return 24 + 18
        if n.startswith("screws: cover"): return 118
        if "rocker" in n or "jack" in n: return 24
        return 24
    OFF_COVER = 92
    objs = [_obj(c_tm, C_CAR)]
    for n, m, col in elec:
        objs.append(_obj(m, col, shift=(0, 0, off(n))))
    objs.append(_obj(v_tm, C_COV, shift=(0, 0, OFF_COVER)))
    img, pj = rasterize(objs, f, up, size=(1500, 1350), margin=0.1)
    labels = [
        ("COVER  (cover.stl)", (-25, 36, D + OFF_COVER - 0.5), "L"),
        ("2x M3x16 cover screws -> carrier posts", (GAP_CENTER_U, COVER_POST_V[1], D + 118 - 0), "R"),
        ("Pi Zero 2W + PiSugar 3 + LiPo", (*pi_xy(32, 15), STACK_TOP + 24 - 3), "R"),
        ("FE-URT-2 servo board on 8.5 mm standoffs", (*urt_xy(30, 10), URT_W0 + URT_TOTAL_H + 24 - 3), "L"),
        ("MPU6050 IMU (flat, under the URT-2)", (IMU_C[0], IMU_C[1] + 6, PLATE_T + IMU_POST_H + 2 + 9), "L"),
        ("CARRIER  (carrier.stl)", (-30, 10, PLATE_T), "L"),
        ("3x M3 into the box bosses (B1-B3)", (*BOSS_PACK["B3"], PLATE_T - CBORE_DEPTH + M3_HEAD_H + 16), "L"),
        ("optional rocker switch + DC jack (bottom wall)", (ROCKER_U + 5, -H_OUT / 2 + 8, FEATURE_W + 24 + 6), "L"),
    ]
    _save(img, os.path.join(OUT_PREVIEW, "1_exploded_assembly.png"), labels, pj,
          title="XGO-Duck backpack - exploded view (seen from the robot's feet; the robot's back faces UP in this picture)")
    # 2. assembled, cover removed (URT-2 ghosted so the IMU underneath is visible)
    f = (-0.45, 0.7, -0.8)
    objs = [_obj(c_tm, C_CAR)]
    for n, m, col in elec:
        if n.startswith("screws: cover"):
            continue
        a_ = 0.45 if n.startswith("FE-URT-2") else 1.0
        objs.append(_obj(m, col, alpha=a_))
    img, pj = rasterize(objs, f, up, size=(1500, 1250), margin=0.1)
    _save(img, os.path.join(OUT_PREVIEW, "2_assembly_cover_removed.png"), [
        ("Pi + PiSugar stack (ports face the right wall)", (*pi_xy(32, 0), STACK_BOTTOM + 8), "R"),
        ("FE-URT-2 (ghosted)", (*urt_xy(40, 8), URT_W0 + 6), "L"),
        ("IMU under the URT-2, flat on the carrier", (IMU_C[0], IMU_C[1] - 6, PLATE_T + IMU_POST_H + 2), "L"),
        ("DC-jack saddles + rocker bay", (JACK_U, -H_IN / 2 + 5, FEATURE_W), "L"),
        ("cover screw posts (gap)", (GAP_CENTER_U, COVER_POST_V[0], COVER_POST_TOP - 2), "R"),
    ], pj, title="Assembly with the cover removed (view from the robot's feet; back faces up)")
    # 3. back view of the finished pack (cover on) with box outline
    f = (0, 0, -1)
    objs = [_obj(v_tm, C_COV)]
    img, pj = rasterize(objs, f, (0, 1, 0), size=(1300, 1250), edges=True)
    def box_outline(ax, project):
        corners = np.array([[BOX_X[0] - U0, BOX_Z[0] - V0, 0], [BOX_X[1] - U0, BOX_Z[0] - V0, 0], [BOX_X[1] - U0, BOX_Z[1] - V0, 0],
                            [BOX_X[0] - U0, BOX_Z[1] - V0, 0], [BOX_X[0] - U0, BOX_Z[0] - V0, 0]])
        x, y = project(corners)
        ax.plot(x, y, "--", color="#c0392b", lw=1.3)
        ax.text(x[0] + 6, y[0] - 8, "battery box outline (behind)", color="#c0392b", fontsize=8)
    _save(img, os.path.join(OUT_PREVIEW, "3_back_view_cover_on.png"), [], pj,
          title=f"Back view (what the floor sees): {W_OUT:.1f} x {H_OUT:.1f} mm, flat back + 45-deg rounded edges, vent slots, side cut-outs", extra=box_outline)
    # 4. side view with ghosted battery box
    f = (1, 0, 0)
    objs = [_obj(c_tm, C_CAR), _obj(v_tm, C_COV)] + [_obj(m, col) for n, m, col in elec] + [_obj(bat_p, (0.4, 0.45, 0.55), alpha=0.35)]
    objs[1]["alpha"] = 0.45          # ghost the cover so the boards show
    img, pj = rasterize(objs, f, (0, 1, 0), size=(1700, 1000))
    def dims(ax, project):
        p = lambda w, v: tuple(float(a[0]) for a in project(np.array([-30.0, v, w])))
        (x0, y0), (x1, y1) = p(0, -H_OUT / 2 - 5), p(D, -H_OUT / 2 - 5)
        ax.annotate("", (x1, y0), (x0, y0), arrowprops=dict(arrowstyle="<->", color="#c0392b"))
        ax.text((x0 + x1) / 2, y0 + 16, f"{D:.1f} mm behind boss tips", ha="center", color="#c0392b", fontsize=10, weight="bold")
        xb, _ = p(0, 0)
        ax.axvline(xb, color="#c0392b", lw=0.8, ls=":")
        ax.text(xb - 6, 62, "boss-tip plane (w = 0)", ha="right", color="#c0392b", fontsize=8)
    _save(img, os.path.join(OUT_PREVIEW, "4_side_view_with_battery_box.png"), [], pj,
          title="Side view (robot back to the right): ghosted battery box on the left, ghosted cover, boards inside", extra=dims)
    # 5. iso with battery
    f = (-0.75, -0.55, -0.62)
    objs = [_obj(c_tm, C_CAR), _obj(v_tm, C_COV, 1.0), _obj(bat_p, (0.35, 0.45, 0.6), alpha=0.28)]
    img, pj = rasterize(objs, f, (0, 1, 0), size=(1500, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "5_iso_on_battery_box.png"), [], pj, title="Finished backpack on the battery box (box ghosted), robot up = screen up")
    # 6/7 print orientation views
    cp = cover_print_orientation(v_tm)
    img, pj = rasterize([_obj(cp, C_COV)], (0.25, -0.35, -1), (0, 1, 0), size=(1400, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "6_cover_print_orientation.png"), [], pj, title="cover.stl in print orientation (roof on the bed, open side up) - no supports")
    img, pj = rasterize([_obj(c_tm, C_CAR)], (0.2, 0.35, -1), (0, 1, 0), size=(1400, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "7_carrier_print_orientation.png"), [], pj, title="carrier.stl in print orientation (flat face down) - no supports")
    plan_view(os.path.join(OUT_PREVIEW, "8_plan_view_layout.png"), c_tm, elec)
    print("previews written to", OUT_PREVIEW)


def main():
    os.makedirs(OUT_PREVIEW, exist_ok=True)
    print(f"pack outer {W_OUT:.1f} (u) x {H_OUT:.1f} (v) x {D:.1f} (w, behind boss tips) mm;  cavity {W_IN:.1f} x {H_IN:.1f}")
    print("building carrier ...")
    carrier = build_carrier()
    print('  lightening windows:', len(lightening_windows()))
    print("building cover ...")
    cover = build_cover()
    elec = electronics()

    c_tm = to_tm(carrier)
    v_tm = to_tm(cover)
    print("mesh checks (pack frame):")
    check("carrier", c_tm)
    check("cover", v_tm)

    # ---- collision checks ------------------------------------------------------
    rep = {}
    print("collision checks (volumes in mm3, must be ~0):")
    el_all = [(n, m) for n, m, _ in elec]
    for n, m in el_all:
        vc = (carrier ^ m).volume()
        vv = (cover ^ m).volume()
        rep[n] = dict(carrier=vc, cover=vv)
        flag = "  <-- CHECK" if vc > 0.5 or vv > 0.5 else ""
        print(f"  {n:28s} carrier {vc:8.2f}   cover {vv:8.2f}{flag}")
    cc_ = [c_.volume() for c_ in (carrier ^ cover).decompose()]
    bad = sum(x for x in cc_ if x > 0.05)
    print(f"  carrier vs cover: real overlap {bad:.2f} mm3 (coplanar-contact slivers ignored: {len(cc_)} bits)")
    rep['carrier_vs_cover'] = bad
    # board vs board
    names = [n for n, _ in el_all if n not in ("spacer",)]
    for i in range(len(el_all)):
        for j in range(i + 1, len(el_all)):
            vol = (el_all[i][1] ^ el_all[j][1]).volume()
            if vol > 0.5 and not (el_all[i][0] == "spacer" or el_all[j][0] == "spacer"):
                print(f"  OVERLAP {el_all[i][0]} / {el_all[j][0]}: {vol:.2f}")

    # ---- battery box fit check -------------------------------------------------
    bat = trimesh.load(REF_BATTERY)
    bat_p = battery_to_pack(bat)
    bm = to_man(bat_p)
    vcar = (carrier ^ bm).volume()
    vcov = (cover ^ bm).volume()
    gap_cov = cover.min_gap(bm, 5.0)
    gap_car = carrier.min_gap(bm, 5.0)
    print(f"battery box fit: carrier∩box = {vcar:.3f} mm3, cover∩box = {vcov:.3f} mm3, min gap carrier {gap_car:.3f} mm, cover {gap_cov:.3f} mm")
    rep["battery_overlap"] = dict(carrier=vcar, cover=vcov, min_gap_carrier=gap_car, min_gap_cover=gap_cov)

    # ---- exports ---------------------------------------------------------------
    c_tm.export(os.path.join(HERE, "carrier.stl"))
    cover_print = cover_print_orientation(v_tm)
    cover_print.export(os.path.join(HERE, "cover.stl"))
    asm = trimesh.util.concatenate([pack_to_battery(c_tm), pack_to_battery(v_tm)])
    asm.export(os.path.join(HERE, "backpack_assembled_battery_frame.stl"))
    el_tm = trimesh.util.concatenate([to_tm(m) for n, m, _ in elec])
    el_tm.export(os.path.join(HERE, "electronics_boxes_pack_frame.stl"))
    print("exported STLs; re-check of the exported print-orientation files:")
    cp = trimesh.load(os.path.join(HERE, "cover.stl"))
    cc = trimesh.load(os.path.join(HERE, "carrier.stl"))
    check("cover.stl (print)", cp)
    check("carrier.stl (print)", cc)
    rep.update(dict(
        outer=[W_OUT, H_OUT, D], carrier_cm3=c_tm.volume / 1000, cover_cm3=v_tm.volume / 1000,
        carrier_g=c_tm.volume / 1000 * PLA_DENSITY, cover_g=v_tm.volume / 1000 * PLA_DENSITY,
        watertight=dict(carrier=bool(cc.is_watertight), cover=bool(cp.is_watertight)),
        cover_print_bounds=cp.bounds.tolist(), carrier_bounds=cc.bounds.tolist(),
    ))
    with open(os.path.join(HERE, "report.json"), "w") as f:
        json.dump(rep, f, indent=1, default=float)

    # ---- previews ----------------------------------------------------------------
    make_previews(c_tm, v_tm, [(n, to_tm(m), col) for n, m, col in elec], bat_p)


if __name__ == "__main__":
    main()
