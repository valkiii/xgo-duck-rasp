#!/usr/bin/env python
"""
XGO-Duck "backpack" v2: carrier + cover (+ header hood) that bolt onto the printed bosses on the
BACK of the battery box (structure/battery.stl of LuwuDynamics/xgoduck_hardware).

Run:   /Users/vc/Research/xgo-duck/.venv/bin/python backpack.py
Out:   carrier.stl, cover.stl, hood.stl (print orientation), backpack_assembled_battery_frame.stl,
       electronics_boxes_pack_frame.stl, previews/*.png, report.json

COORDINATE FRAMES
  pack frame (u, v, w): u = robot x (to the RIGHT when looking at the robot's back), v = robot z (up),
                        w = distance BEHIND the boss-tip plane (w = 0 is where the carrier touches the bosses).
  battery frame (x, y, z) = battery.stl:  x = u + U0,  z = v + V0,  y = -24.6 - w.

LAYOUT (looking at the robot's back, see previews/8_plan_view_layout.png)
  LEFT column : Feetech FE-URT-2 on 8 mm standoffs; the MPU6050 IMU lies flat on the carrier UNDER it
                (its 5 right-angle jumpers run in-plane toward the bottom bay).
  GAP         : cover screw post (top) and the wire lane from the Pi header hood down to the IMU.
  RIGHT column: Pi Zero 2W on PiSugar 3 + LiPo, camera end up, SD / pin-1 end down.  Micro-USB, mini-HDMI,
                PiSugar USB-C + 2 buttons face the right wall (one long slot).
  ROOF        : 25.0 mm deep over most of the pack, 30.5 mm over the GPIO strip, 39.0 mm over the pin-1 end
                (straight female Dupont jumpers).  The raised strip is a separate printed part, hood.stl.
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
# 1. MEASURED values
# =============================================================================
# --- from battery.stl (sliced) ---
BOSS_TIP_Y = -24.6            # y of the boss tips (= tips of side rails) -> pack frame w = 0
BOSS_OD = 5.5
BOSSES = {                    # x, z (battery frame), pilot depth from the tip (measured by slicing)
    "B1": dict(x=19.1, z=33.2, pilot=3.9),
    "B2": dict(x=-8.9, z=33.2, pilot=5.9),
    "B3": dict(x=-24.1, z=-17.6, pilot=7.9),
    "B4": dict(x=24.1, z=-18.9, pilot=0.0),   # solid pin, NO hole -> plate just rests on it
}
BOX_X = (-26.9, 27.0)
BOX_Z = (-35.0, 36.5)
# --- measured by the user (v2) ---
URT_L, URT_W = 56.2, 36.5     # FE-URT-2 PCB (L between terminal end and USB-C end)
URT_HOLE_DX, URT_HOLE_DY = 49.2, 29.2     # hole rectangle (along L, along W)  -> insets 3.5 / 3.65
PI_L, PI_W = 65.0, 30.0       # Pi Zero 2W
PI_HOLE_DX, PI_HOLE_DY = 58.0, 23.0
PI_STACK_H = 17.5             # LiPo bottom -> top surface of the Pi PCB (user measurement)
HDR_PIN_H = 8.5               # bare male header pins above the Pi PCB
HDR_LEN = 50.8                # 2x20 strip length (pin pitch 2.54)
HDR_ZONE = 9.5                # clearance reserved above the Pi PCB over the whole header strip (bare pins)
HOOD_ZONE = 18.0              # clearance above the Pi PCB over the pin-1 end (straight female Dupont plugs + wire bend)
HOOD_LEN = 25.4               # length of the strip (from pin 1) that gets the tall hood  (pins 1-17 both rows)
ZONE_OTHER = 4.0              # clearance above the Pi PCB everywhere else (connectors ~3 mm)
IMU_W, IMU_L = 22.0, 29.5     # MPU6050 board: W (edge with header / edge with holes), L
IMU_HOLE_SPAN = 16.5          # two holes on the edge opposite the header, centred
IMU_HOLE_INSET = 2.5          # hole centre from that edge (assumed)
IMU_PIN_OUT = 7.0             # right-angle header pins stick out beyond the header edge (in-plane)
IMU_JUMPER_L = 14.0           # Dupont housings (in-plane) beyond the pins
IMU_T = 1.6
# =============================================================================
# 2. ASSUMED values (TO MEASURE)
# =============================================================================
URT_HOLE_D = 3.2              # mounting hole dia (user: 3.0-3.2)
URT_PCB_T = 1.6
URT_TOTAL_H = 11.5            # PCB underside -> tallest part                            TO MEASURE
LIPO_T = 8.0                  # LiPo thickness = LiPo underside -> PiSugar PCB underside TO MEASURE (drives post height)
LIPO_L, LIPO_W = 50.0, 24.0   # LiPo footprint                                           TO MEASURE
IMU_HOLE_D = 3.0
IMU_PIN_PITCH_W = 20.3        # 8 pins x 2.54 (width taken by the jumpers)
IMU_JUMPER_H = 2.5            # housing height
PI_USB_PX, PI_PWR_PX = 41.4, 54.0         # datasheet
M3_CLEAR, M3_PILOT, M25_PILOT = 3.4, 2.8, 2.2
M3_HEAD_D, M3_HEAD_H = 5.8, 2.4                           # TO MEASURE (round head)
M25_HEAD_D, M25_HEAD_H = 4.8, 2.0

# =============================================================================
# 3. DESIGN parameters
# =============================================================================
T = 1.6
CLR = 0.3
PLATE_T = 1.6
FIT = 0.25
GAP = 7.4
TOP_GUTTER = 7.2
V0 = 0.5
U0 = 2.0
R_PLAN = 5.5
R_EDGE = 8.0
W_RIM = 0.3
URT_STANDOFF = 8.0
IMU_POST_H = 3.0
PI_POST_OD, URT_POST_OD, IMU_POST_OD = 5.4, 6.0, 5.0
STACK_BOTTOM = PLATE_T + 0.3  # LiPo underside height (no screw heads under the Pi stack any more)
CBORE_DEPTH = 0.2
CBORE_D = 5.9
COVER_POST_OD = 6.4
COVER_BOSS_OD = 7.6
COVER_POST_TOP = 14.0
COVER_CBORE_D, COVER_CBORE_DEPTH = 6.2, 2.8
SERVO_NOTCH_V = (-10.0, 30.0)
PLA_DENSITY = 1.24
SEG = 48
R_HOOD_EDGE = 3.0
HOOD_SKIRT = 2.0

# ---- derived layout (pack frame) ---------------------------------------------
PI_COL_W, PI_COL_L = PI_W + 2 * CLR, PI_L + 2 * CLR
URT_COL_W, URT_COL_L = URT_W + 2 * CLR, URT_L + 2 * CLR
W_IN = PI_COL_W + GAP + URT_COL_W
H_IN = PI_COL_L + TOP_GUTTER
W_OUT, H_OUT = W_IN + 2 * T, H_IN + 2 * T
PI_TOP = STACK_BOTTOM + PI_STACK_H                       # top surface of the Pi PCB (w)
D = PI_TOP + ZONE_OTHER + T                              # roof depth over most of the pack
D_STRIP = PI_TOP + HDR_ZONE + T                          # roof depth over the GPIO strip
D_HOOD = PI_TOP + HOOD_ZONE + T                          # roof depth over the pin-1 end
H_C = R_EDGE * np.cos(np.pi / 4)
R_CAV = R_PLAN - T

URT_COL = (-W_IN / 2, -W_IN / 2 + URT_COL_W, H_IN / 2 - URT_COL_L, H_IN / 2)
PI_COL = (W_IN / 2 - PI_COL_W, W_IN / 2, -H_IN / 2, -H_IN / 2 + PI_COL_L)
GAP_U = (URT_COL[1], PI_COL[0])
GAP_CENTER_U = 0.5 * (GAP_U[0] + GAP_U[1])
BAY_V = (-H_IN / 2, URT_COL[2])
URT_BRD = (URT_COL[0] + CLR, URT_COL[1] - CLR, URT_COL[2] + CLR, URT_COL[3] - CLR)
PI_BRD = (PI_COL[0] + CLR, PI_COL[1] - CLR, PI_COL[2] + CLR, PI_COL[3] - CLR)
URT_W0 = PLATE_T + URT_STANDOFF
PI_POST_TOP = STACK_BOTTOM + LIPO_T
BOSS_PACK = {k: (b["x"] - U0, b["z"] - V0) for k, b in BOSSES.items()}
COVER_POSTS = [(GAP_CENTER_U, 32.4), (-33.5, -32.2)]    # (u, v) of the 2 cover screw posts

URT_INSET_L = (URT_L - URT_HOLE_DX) / 2
URT_INSET_W = (URT_W - URT_HOLE_DY) / 2
URT_HOLES = [(URT_INSET_L, URT_INSET_W), (URT_L - URT_INSET_L, URT_INSET_W),
             (URT_INSET_L, URT_W - URT_INSET_W), (URT_L - URT_INSET_L, URT_W - URT_INSET_W)]


def pi_xy(px, py):
    """Pi local (px along 65 mm from the SD / pin-1 end, py across from the ports edge) -> pack (u, v)."""
    return (PI_BRD[1] - py, PI_BRD[2] + px)


def urt_xy(ax, ay):
    return (URT_BRD[0] + ay, URT_BRD[2] + ax)


# header strip (Pi local)
HDR_PY = (PI_W - 3.5 - 2.54, PI_W - 3.5 + 2.54)
HDR_PX = (PI_L / 2 - HDR_LEN / 2, PI_L / 2 + HDR_LEN / 2)       # 7.1 .. 57.9 ; pin 1 at px = 7.1 (SD end)
# IMU: long axis along v, header edge toward -v (bottom bay), holes edge toward +v
IMU_CU, IMU_CV = -18.0, 4.0
IMU_V0 = IMU_CV - IMU_L / 2                               # header edge
IMU_V1 = IMU_CV + IMU_L / 2
IMU_PCB_W0 = PLATE_T + IMU_POST_H                         # PCB underside
IMU_HOLES = [(IMU_CU - IMU_HOLE_SPAN / 2, IMU_V1 - IMU_HOLE_INSET), (IMU_CU + IMU_HOLE_SPAN / 2, IMU_V1 - IMU_HOLE_INSET)]

# hood windows (pack frame).  C = cavity rectangle, E = outer rectangle of the hood body, window = E + clearance
def _rect_pad(r, p):
    return (r[0] - p, r[1] + p, r[2] - p, r[3] + p)


HOOD_C_A = (1.8, pi_xy(0, HDR_PY[0])[0] + 1.5, pi_xy(HDR_PX[0], 0)[1] - 1.5, pi_xy(HDR_PX[1], 0)[1] + 1.5)
HOOD_C_B = (HOOD_C_A[0], HOOD_C_A[1], HOOD_C_A[2], pi_xy(HDR_PX[0] + HOOD_LEN, 0)[1] + 1.5)
HOOD_E_A = _rect_pad(HOOD_C_A, T)
HOOD_E_B = _rect_pad(HOOD_C_B, T)
HOOD_WINDOW = _rect_pad(HOOD_E_A, 0.15)


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


def _ring(rect, d, w, rc0, n=12):
    p = rrect(rect[0] + d, rect[1] - d, rect[2] + d, rect[3] - d, max(rc0 - d, 0.8), n)
    xy = np.array(p.exterior.coords)[:-1]
    return np.c_[xy, np.full(len(xy), w)]


def shell_hulls(rect, rc0, w_bot, w_top, R, t):
    """outer / inner convex hulls of a rounded shell whose outside face is at w_top (45-deg-limited edge radius R)."""
    Hc = R * np.cos(np.pi / 4)
    pts = [_ring(rect, 0, w_bot, rc0), _ring(rect, 0, w_top - Hc, rc0)]
    for a in np.radians(np.linspace(0, 45, 7)):
        pts.append(_ring(rect, R * (1 - np.cos(a)), w_top - (Hc - R * np.sin(a)), rc0))
    outer = M.Manifold.hull_points(np.vstack(pts))
    Ri = R - t
    a_end = np.arcsin((Hc - t) / Ri)
    ip = [_ring(rect, t, w_bot - 1.0, rc0), _ring(rect, t, w_top - Hc, rc0)]
    for a in np.linspace(0, a_end, 6):
        ip.append(_ring(rect, R - Ri * np.cos(a), w_top - (Hc - Ri * np.sin(a)), rc0))
    inner = M.Manifold.hull_points(np.vstack(ip))
    return outer, inner


# =============================================================================
# CARRIER
# =============================================================================
def plate_outline():
    d = T + FIT
    return rrect(-W_OUT / 2 + d, W_OUT / 2 - d, -H_OUT / 2 + d, H_OUT / 2 - d, R_CAV - FIT)


def pi_post_pts():
    return [pi_xy(px, py) for px in (3.5, PI_L - 3.5) for py in (3.5, PI_W - 3.5)]


def keepouts():
    ko = [Point(u, v).buffer(4.6) for (u, v) in BOSS_PACK.values()]
    ko += [Point(*urt_xy(ax, ay)).buffer(4.2) for ax, ay in URT_HOLES]
    ko += [Point(*p).buffer(4.0) for p in pi_post_pts()]
    ko += [Point(u, v).buffer(4.6) for (u, v) in COVER_POSTS]
    ko.append(sbox(IMU_CU - IMU_W / 2 - 3, IMU_CU + IMU_W / 2 + 3, IMU_V0 - IMU_PIN_OUT - IMU_JUMPER_L - 3, IMU_V1 + 3))
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


def _tri_prism_along_u(tri_vw, u, length):
    cs = M.CrossSection([np.array(tri_vw, dtype=float)], M.FillRule.EvenOdd)
    mesh = cs.extrude(length).to_mesh()
    p = np.array(mesh.vert_properties)[:, :3]
    t = np.array(mesh.tri_verts)
    p2 = np.c_[p[:, 2] + u - length / 2, p[:, 0], p[:, 1]]
    return M.Manifold(M.Mesh(vert_properties=p2.astype(np.float32), tri_verts=t[:, ::-1].astype(np.uint32)))


BUMPS_TOP_U = (-2.0, 6.0)
BUMPS_BOT_U = (1.5, 9.0)
BUMP_H, BUMP_L, BUMP_W, BUMP_W_CENTER = 0.5, 5.0, 0.8, 0.8


def groove(u, side):
    ve = side * (H_OUT / 2 - T - FIT)
    g, hw = BUMP_H + 0.1, 0.7
    tri = [(ve + side * 0.3, BUMP_W_CENTER - hw - 0.15), (ve - side * g, BUMP_W_CENTER), (ve + side * 0.3, BUMP_W_CENTER + hw + 0.15)]
    if side < 0:
        tri = tri[::-1]
    return _tri_prism_along_u(tri, u, BUMP_L + 1.0)


def bump(u, side):
    ve = side * (H_OUT / 2 - T)
    tri = [(ve + side * 0.01, BUMP_W_CENTER - BUMP_W / 2), (ve - side * BUMP_H, BUMP_W_CENTER), (ve + side * 0.01, BUMP_W_CENTER + BUMP_W / 2)]
    if side < 0:
        tri = tri[::-1]
    wall = bx(u - BUMP_L / 2, u + BUMP_L / 2, min(ve, ve + side * 0.8), max(ve, ve + side * 0.8),
              BUMP_W_CENTER - BUMP_W / 2, BUMP_W_CENTER + BUMP_W / 2)
    return _tri_prism_along_u(tri, u, BUMP_L) + wall


def build_carrier(with_windows=True):
    parts = [prism(plate_outline(), 0, PLATE_T)]
    for ax, ay in URT_HOLES:
        u, v = urt_xy(ax, ay)
        parts.append(cyl(u, v, PLATE_T - 0.2, URT_W0, URT_POST_OD))
    for u, v in pi_post_pts():
        parts.append(cyl(u, v, PLATE_T - 0.2, PI_POST_TOP, PI_POST_OD))
    for u, v in IMU_HOLES:
        parts.append(cyl(u, v, PLATE_T - 0.2, IMU_PCB_W0, IMU_POST_OD))
    # IMU corner locators (small L walls just outside the PCB corners; the header edge stays open in the middle)
    iu0, iu1 = IMU_CU - IMU_W / 2 - CLR, IMU_CU + IMU_W / 2 + CLR
    iv0, iv1 = IMU_V0 - CLR, IMU_V1 + CLR
    tabh = IMU_PCB_W0 + 1.0
    for cu, su in ((iu0, -1), (iu1, 1)):
        for cv, sv in ((iv0, -1), (iv1, 1)):
            ua, ub = sorted((cu, cu + su * 1.2)); va, vb = sorted((cv - sv * 5.0, cv + sv * 1.2))
            parts.append(bx(ua, ub, va, vb, PLATE_T - 0.1, tabh))
            ua, ub = sorted((cu - su * 5.0, cu + su * 1.2)); va, vb = sorted((cv, cv + sv * 1.2))
            parts.append(bx(ua, ub, va, vb, PLATE_T - 0.1, tabh))
    for u, v in COVER_POSTS:
        parts.append(cyl(u, v, PLATE_T - 0.2, COVER_POST_TOP, COVER_POST_OD))
    carrier = union(parts)
    cuts = []
    for k, (u, v) in BOSS_PACK.items():
        if k == "B4":
            continue
        cuts.append(cyl(u, v, -0.1, PLATE_T + 0.1, M3_CLEAR))
        cuts.append(cyl(u, v, PLATE_T - CBORE_DEPTH, PLATE_T + 0.2, CBORE_D))
    for ax, ay in URT_HOLES:
        u, v = urt_xy(ax, ay)
        cuts.append(cyl(u, v, URT_W0 - 7.0, URT_W0 + 0.1, M3_PILOT, 24))
    for u, v in pi_post_pts():
        cuts.append(cyl(u, v, PI_POST_TOP - 7.8, PI_POST_TOP + 0.1, M25_PILOT, 24))
    for u, v in IMU_HOLES:
        cuts.append(cyl(u, v, 0.6, IMU_PCB_W0 + 0.1, M25_PILOT, 24))
    for u, v in COVER_POSTS:
        cuts.append(cyl(u, v, COVER_POST_TOP - 9.0, COVER_POST_TOP + 0.1, M3_PILOT, 24))
    if with_windows:
        for w in lightening_windows():
            cuts.append(prism(w, -0.1, PLATE_T + 0.1))
    for u in BUMPS_TOP_U:
        cuts.append(groove(u, +1))
    for u in BUMPS_BOT_U:
        cuts.append(groove(u, -1))
    return carrier - union(cuts)


# =============================================================================
# COVER and HOOD
# =============================================================================
def build_cover():
    outer, inner = shell_hulls((-W_OUT / 2, W_OUT / 2, -H_OUT / 2, H_OUT / 2), R_PLAN, W_RIM, D, R_EDGE, T)
    shell = outer - inner
    add, sub = [], []
    roof_in = D - T
    for (u, v) in COVER_POSTS:
        add.append(cyl(u, v, COVER_POST_TOP, roof_in + 0.4, COVER_BOSS_OD))
        sub.append(cyl(u, v, COVER_POST_TOP - 0.1, D + 0.2, M3_CLEAR, 24))
        sub.append(cyl(u, v, D - COVER_CBORE_DEPTH, D + 0.2, COVER_CBORE_D, 32))
    for u in BUMPS_TOP_U:
        add.append(bump(u, +1))
    for u in BUMPS_BOT_U:
        add.append(bump(u, -1))
    # hood window (hood.stl plugs into it)
    win = HOOD_WINDOW
    sub.append(prism(rrect(*win, 3.2), roof_in - 0.5, D + 0.5))
    # vent slots in the roof (2 x 14-18), skipped where they would touch the hood window / screw bosses
    excl = unary_union([sbox(*_rect_pad(win, 2.5))] + [Point(u, v).buffer(6.0) for (u, v) in COVER_POSTS] +
                       [sbox(PI_BRD[0], PI_BRD[1], PI_BRD[3] - 1, H_OUT / 2)])
    vents = []
    for u0 in np.linspace(PI_BRD[0] + 5, PI_BRD[1] - 5, 5):
        for v0 in (-17.0, 8.0):
            vents.append((u0, v0, 8.5))
    for u0 in np.linspace(URT_BRD[0] + 5, URT_BRD[1] - 5, 6):
        for v0 in (-4.0, 22.0):
            if URT_COL[2] + 3 < v0 - 7 and v0 + 7 < URT_COL[3] - 3:
                vents.append((u0, v0, 7.0))
    for u0, v0, hl in vents:
        if not excl.intersects(sbox(u0 - 1.0, u0 + 1.0, v0 - hl, v0 + hl)):
            sub.append(bx(u0 - 1.0, u0 + 1.0, v0 - hl, v0 + hl, D - T - 0.5, D + 0.5))
    # wall cut-outs: all open to the rim (printable without supports)
    ww = T + 3.0
    xr, xl, yt, yb = W_OUT / 2, -W_OUT / 2, H_OUT / 2, -H_OUT / 2
    # right wall: Pi micro-USB / mini-HDMI / PiSugar USB-C + 2 buttons: from the rim (below the LiPo bottom) to above the Pi connectors
    sub.append(bx(xr - ww, xr + 1, PI_BRD[2] + 3.0, PI_BRD[3] - 3.0, -1.0, PI_TOP + 3.3))
    sub.append(bx(xl - 1, xl + ww, SERVO_NOTCH_V[0], SERVO_NOTCH_V[1], -1.0, 18.0))           # servo cables
    pc, uc = 0.5 * (PI_BRD[0] + PI_BRD[1]), 0.5 * (URT_BRD[0] + URT_BRD[1])
    sub.append(bx(pc - 11.0, pc + 11.0, yt - ww, yt + 1, -1.0, PI_TOP + 2.5))                 # camera ribbon
    sub.append(bx(uc - 7.0, uc + 7.0, yt - ww, yt + 1, -1.0, 16.0))                           # URT-2 USB-C plug
    sub.append(bx(pc - 9.0, pc + 9.0, yb - 1, yb + ww, -1.0, PI_TOP + 2.5))                   # SD card
    return (shell + union(add)) - union(sub)


def build_hood():
    """Raised, rounded cap over the GPIO strip (30.5 mm) with a taller part over pin 1.. 17 (39 mm).
    Plug fits into the cover's roof window; skirt rests on the roof; no skirt on the gap (-u) side so it prints lying on that side."""
    wb = D - T                                    # plug bottom
    oA, iA = shell_hulls(HOOD_E_A, 3.2, wb, D_STRIP, R_HOOD_EDGE, T)
    oB, iB = shell_hulls(HOOD_E_B, 3.2, wb, D_HOOD, R_HOOD_EDGE, T)
    # skirt (frustum): outward on +u, -v, +v sides only
    e = HOOD_E_A
    sk = HOOD_SKIRT
    ring_lo = np.vstack([_ring((e[0], e[1] + sk, e[2] - sk, e[3] + sk), 0, D, 3.2)])
    ring_hi = _ring(e, 0, D + sk, 3.2)
    skirt = M.Manifold.hull_points(np.vstack([ring_lo, ring_hi]))
    body = (oA + oB + skirt)
    cav = iA + iB
    body = body - cav
    # keep the u- side flat (the skirt ring is rounded: trim anything beyond the window edge on -u)
    body = body - bx(e[0] - 10, e[0] - 0.0, -200, 200, D - 5, D_HOOD + 5)
    return body


# =============================================================================
# ELECTRONICS stand-ins (assembly renders + collision checks)
# =============================================================================
def electronics():
    out = []

    def pi_box(px0, px1, py0, py1, w0, w1):
        (u0, v0), (u1, v1) = pi_xy(px0, py0), pi_xy(px1, py1)
        return bx(min(u0, u1), max(u0, u1), min(v0, v1), max(v0, v1), w0, w1)

    def holes_pi(m, w0, w1, d):
        for u, v in pi_post_pts():
            m = m - cyl(u, v, w0 - 0.1, w1 + 0.1, d, 24)
        return m

    w = STACK_BOTTOM
    lipo = pi_box((PI_L - LIPO_L) / 2, (PI_L + LIPO_L) / 2, (PI_W - LIPO_W) / 2, (PI_W + LIPO_W) / 2, w, w + LIPO_T)
    out.append(("LiPo (PiSugar 3)", lipo, (0.35, 0.45, 0.85)))
    w += LIPO_T
    psb = holes_pi(prism(rrect(*PI_BRD[:2], *PI_BRD[2:], 3.0), w, w + 1.6), w, w + 1.6, 2.7)
    out.append(("PiSugar 3 PCB", psb, (0.15, 0.55, 0.3)))
    ps_top = w + 1.6
    # PiSugar USB-C + 2 buttons on the ports edge (positions ASSUMED, preview only)
    ps = [pi_box(17, 26, -1.0, 6.5, ps_top, ps_top + 3.3), pi_box(32, 35.5, -0.5, 3.0, ps_top, ps_top + 3.0), pi_box(38, 41.5, -0.5, 3.0, ps_top, ps_top + 3.0)]
    out.append(("PiSugar port + buttons", union(ps), (0.75, 0.75, 0.78)))
    pib_bot = PI_TOP - 1.6
    pib = holes_pi(prism(rrect(*PI_BRD[:2], *PI_BRD[2:], 3.0), pib_bot, PI_TOP), pib_bot, PI_TOP, 2.7)
    out.append(("Pi Zero 2W", pib, (0.1, 0.6, 0.25)))
    for u, v in pi_post_pts():
        out.append(("spacer", cyl(u, v, ps_top, pib_bot, 5.0), (0.8, 0.7, 0.2)))
    parts = [pi_box(HDR_PX[0], HDR_PX[1], HDR_PY[0], HDR_PY[1], PI_TOP, PI_TOP + HDR_PIN_H),
             pi_box(PI_USB_PX - 4, PI_USB_PX + 4, -1.0, 4.6, PI_TOP, PI_TOP + 2.6),
             pi_box(PI_PWR_PX - 4, PI_PWR_PX + 4, -1.0, 4.6, PI_TOP, PI_TOP + 2.6),
             pi_box(12.4 - 5.6, 12.4 + 5.6, -0.5, 6.5, PI_TOP, PI_TOP + 3.2),
             pi_box(24, 36, 9, 21, PI_TOP, PI_TOP + 1.0)]
    out.append(("Pi connectors / header pins", union(parts), (0.7, 0.7, 0.72)))
    # five straight female Dupont jumpers on pins 1,3,5,6,17 (2.8 x 2.8 housings, 14 mm tall) + wires bending toward the gap
    pin_pos = [(HDR_PX[0], HDR_PY[0] + 1.27 + 0), (HDR_PX[0] + 2.54, HDR_PY[0] + 1.27), (HDR_PX[0] + 5.08, HDR_PY[0] + 1.27),
               (HDR_PX[0] + 5.08, HDR_PY[0] + 1.27 + 2.54), (HDR_PX[0] + 8 * 2.54, HDR_PY[0] + 1.27)]
    jm = []
    for px, py in pin_pos:
        jm.append(pi_box(px - 1.4, px + 1.4, py - 1.4, py + 1.4, PI_TOP + 2.5, PI_TOP + 16.5))
    jm.append(pi_box(HDR_PX[0] - 1.4, HDR_PX[0] + 8 * 2.54 + 1.4, HDR_PY[0] - 0.1, 34.5, PI_TOP + 16.5, PI_TOP + 17.5))
    out.append(("Dupont jumpers (straight)", union(jm), (0.9, 0.55, 0.1)))
    # --- URT-2
    def u_box(ax0, ax1, ay0, ay1, w0, w1):
        (u0, v0), (u1, v1) = urt_xy(ax0, ay0), urt_xy(ax1, ay1)
        return bx(min(u0, u1), max(u0, u1), min(v0, v1), max(v0, v1), w0, w1)

    w0 = URT_W0
    pcb = prism(rrect(*URT_BRD[:2], *URT_BRD[2:], 3.0), w0, w0 + URT_PCB_T)
    for ax, ay in URT_HOLES:
        u, v = urt_xy(ax, ay)
        pcb = pcb - cyl(u, v, w0 - 0.1, w0 + 2, URT_HOLE_D, 24)
    out.append(("FE-URT-2 PCB", pcb, (0.1, 0.25, 0.7)))
    wt, hmax = w0 + URT_PCB_T, w0 + URT_TOTAL_H
    c = [u_box(0.0, 10.5, 8.2, 28.4, wt, hmax), u_box(19, 26.6, -0.8, 5.0, wt, wt + 6.0), u_box(29, 36.6, -0.8, 5.0, wt, wt + 6.0),
         u_box(18, 28.2, URT_W - 5.0, URT_W + 0.8, wt, wt + 6.0), u_box(31, 41.2, URT_W - 5.0, URT_W + 0.8, wt, wt + 6.0),
         u_box(URT_L - 6.5, URT_L + 1.0, 13.5, 22.5, wt, wt + 3.3), u_box(46.0, 54.0, 8.5, 12.5, wt, wt + 3.5),
         u_box(43.0, 45.5, 17.0, 29.7, wt, wt + 8.5)]
    out.append(("FE-URT-2 connectors", union(c), (0.85, 0.85, 0.9)))
    # --- IMU (flat on its posts, long axis along v, header edge down) with right-angle pins + in-plane Dupont jumpers
    iw = IMU_PCB_W0
    imu = prism(rrect(IMU_CU - IMU_W / 2, IMU_CU + IMU_W / 2, IMU_V0, IMU_V1, 1.0), iw, iw + IMU_T)
    for u, v in IMU_HOLES:
        imu = imu - cyl(u, v, iw - 0.1, iw + IMU_T + 0.1, IMU_HOLE_D, 24)
    imu = imu + bx(IMU_CU - 2, IMU_CU + 2, IMU_CV - 2, IMU_CV + 2, iw + IMU_T, iw + IMU_T + 1.2)
    out.append(("MPU6050 IMU", imu, (0.85, 0.1, 0.1)))
    wm = iw + IMU_T / 2
    jb = bx(IMU_CU - IMU_PIN_PITCH_W / 2, IMU_CU + IMU_PIN_PITCH_W / 2, IMU_V0 - IMU_PIN_OUT - IMU_JUMPER_L, IMU_V0, wm - IMU_JUMPER_H / 2, wm + IMU_JUMPER_H / 2)
    out.append(("IMU pins + Dupont housings", jb, (0.9, 0.55, 0.1)))
    # --- screws
    def screw(u, v, w_seat, head_d, head_h, L, shank_d):
        return cyl(u, v, w_seat, w_seat + head_h, head_d) + cyl(u, v, w_seat - L, w_seat + 0.01, shank_d, 24)
    sb = [screw(*BOSS_PACK[k], PLATE_T - CBORE_DEPTH, M3_HEAD_D, M3_HEAD_H, L, 2.5) for k, L in (("B1", 5.0), ("B2", 6.0), ("B3", 8.0))]
    out.append(("screws: boss M3 (x3)", union(sb), (0.25, 0.25, 0.28)))
    sc = [screw(*urt_xy(ax, ay), URT_W0 + URT_PCB_T, M3_HEAD_D, M3_HEAD_H, 8.0, 2.5) for ax, ay in URT_HOLES]
    out.append(("screws: URT-2 M3x8 (x4)", union(sc), (0.25, 0.25, 0.28)))
    sp = [screw(u, v, PI_TOP, M25_HEAD_D, M25_HEAD_H, 16.0, 2.0) for u, v in pi_post_pts()]
    out.append(("screws: Pi M2.5x16 (x4)", union(sp), (0.25, 0.25, 0.28)))
    si = [screw(u, v, IMU_PCB_W0 + IMU_T, M25_HEAD_D, M25_HEAD_H, 5.0, 2.0) for u, v in IMU_HOLES]
    out.append(("screws: IMU M2.5x5 (x2)", union(si), (0.25, 0.25, 0.28)))
    cs_ = [screw(u, v, D - COVER_CBORE_DEPTH, M3_HEAD_D, M3_HEAD_H, 16.0, 2.5) for u, v in COVER_POSTS]
    out.append(("screws: cover M3x16 (x2)", union(cs_), (0.25, 0.25, 0.28)))
    return out


# =============================================================================
# frame transforms / helpers
# =============================================================================
def battery_to_pack(mesh):
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0] - U0, v[:, 2] - V0, -v[:, 1] + BOSS_TIP_Y]
    m.fix_normals()
    return m


def pack_to_battery(mesh):
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0] + U0, BOSS_TIP_Y - v[:, 2], v[:, 1] + V0]
    m.fix_normals()
    return m


def print_orient(mesh, d_top):
    """roof / ridge on the bed: rotate 180 deg about u -> (u, v, w) -> (u, -v, d_top - w)."""
    m = mesh.copy()
    v = m.vertices
    m.vertices = np.c_[v[:, 0], -v[:, 1], d_top - v[:, 2]]
    m.fix_normals()
    return m


def hood_print_orient(mesh):
    """hood is printed lying on its flat gap-side wall (u- face on the bed): (u, v, w) -> (w, v, u-u_min)."""
    m = mesh.copy()
    v = m.vertices
    umin = v[:, 0].min()
    m.vertices = np.c_[v[:, 2], -v[:, 1], v[:, 0] - umin]
    m.fix_normals()
    return m


def check(name, tm):
    print(f"  {name:22s} watertight={tm.is_watertight!s:5s} winding={tm.is_winding_consistent!s:5s} "
          f"vol={tm.volume / 1000:.2f} cm3  mass(solid PLA)={tm.volume / 1000 * PLA_DENSITY:.1f} g  size={np.round(tm.bounds[1] - tm.bounds[0], 1)}")


def to_man(tm):
    return M.Manifold(M.Mesh(np.asarray(tm.vertices, dtype=np.float32), np.asarray(tm.faces, dtype=np.uint32)))


def lipo_sink_analysis(bat_tm):
    """How far could the LiPo (assumed footprint) sink behind the boss-tip plane?  Ray-cast the box from behind."""
    u0, v0 = pi_xy((PI_L - LIPO_L) / 2, (PI_W + LIPO_W) / 2)
    u1, v1 = pi_xy((PI_L + LIPO_L) / 2, (PI_W - LIPO_W) / 2)
    uu = np.arange(min(u0, u1), max(u0, u1) + 0.01, 0.5) + U0
    vv = np.arange(min(v0, v1), max(v0, v1) + 0.01, 0.5) + V0
    X, Z = np.meshgrid(uu, vv)
    o = np.c_[X.ravel(), np.full(X.size, BOSS_TIP_Y - 15.0), Z.ravel()]
    d = np.tile([0, 1.0, 0], (len(o), 1))
    loc, ri, _ = bat_tm.ray.intersects_location(o, d, multiple_hits=False)
    y = np.full(len(o), np.nan)
    y[ri] = loc[:, 1]
    free = np.where(np.isnan(y), 99.0, y - BOSS_TIP_Y)         # mm of free space between tip plane and the first box surface
    res = dict(min_free=float(free.min()), frac_ge_3=float((free >= 3.0).mean()), frac_zero=float((free < 0.2).mean()),
               x_range=[float(uu.min()), float(uu.max())], z_range=[float(vv.min()), float(vv.max())])
    # best shift of the footprint inside the Pi column (+-3 mm u slack, +-8 mm v slack)
    best = (-1, 0, 0)
    for du in np.arange(-3.0, 3.01, 1.0):
        for dv in np.arange(-8.0, 8.01, 2.0):
            sel_x = (X + du); sel_z = (Z + dv)
            o2 = np.c_[sel_x.ravel(), np.full(X.size, BOSS_TIP_Y - 15.0), sel_z.ravel()]
            loc2, ri2, _ = bat_tm.ray.intersects_location(o2, d, multiple_hits=False)
            y2 = np.full(len(o2), np.nan); y2[ri2] = loc2[:, 1]
            f2 = np.where(np.isnan(y2), 99.0, y2 - BOSS_TIP_Y).min()
            if f2 > best[0]:
                best = (float(f2), float(du), float(dv))
    res["best_shift_min_free"] = best
    return res


def inter_vol(a, b):
    return sum(c.volume() for c in (a ^ b).decompose() if c.volume() > 0.05)



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




# =============================================================================
# PREVIEWS: plan view + 3D renders
# =============================================================================
def plan_view(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly, Rectangle, Circle
    fig, ax = plt.subplots(figsize=(11.5, 11))
    ax.set_aspect("equal")

    def poly(p, **kw):
        ax.add_patch(MPoly(np.array(p.exterior.coords), closed=True, **kw))

    ax.add_patch(Rectangle((BOX_X[0] - U0, BOX_Z[0] - V0), BOX_X[1] - BOX_X[0], BOX_Z[1] - BOX_Z[0], fill=False, ls="--", ec="#c0392b", lw=1.6, zorder=20))
    ax.text(BOX_X[0] - U0 + 0.8, BOX_Z[0] - V0 + 1.0, "battery box outline", color="#c0392b", fontsize=8, zorder=21)
    poly(rrect(-W_OUT / 2, W_OUT / 2, -H_OUT / 2, H_OUT / 2, R_PLAN), fc="#f4f1ea", ec="#222", lw=1.6)
    poly(rrect(-W_IN / 2, W_IN / 2, -H_IN / 2, H_IN / 2, R_CAV), fc="#ffffff", ec="#888", lw=0.8)
    poly(plate_outline(), fc="#e8edf3", ec="#34495e", lw=1.0)
    for w in lightening_windows():
        poly(w, fc="white", ec="#34495e", lw=0.6)
    ax.add_patch(Rectangle((GAP_U[0], -H_IN / 2), GAP_U[1] - GAP_U[0], H_IN, fc="#fff3c4", ec="none", alpha=0.6))
    poly(rrect(*URT_BRD[:2], *URT_BRD[2:], 3.0), fc="#3b6fd6", ec="#123", lw=1.2, alpha=0.85)
    poly(rrect(*PI_BRD[:2], *PI_BRD[2:], 3.0), fc="#2e9e4f", ec="#123", lw=1.2, alpha=0.85)
    lu0 = pi_xy((PI_L - LIPO_L) / 2, (PI_W - LIPO_W) / 2); lu1 = pi_xy((PI_L + LIPO_L) / 2, (PI_W + LIPO_W) / 2)
    ax.add_patch(Rectangle((min(lu0[0], lu1[0]), min(lu0[1], lu1[1])), abs(lu0[0] - lu1[0]), abs(lu0[1] - lu1[1]), fc="#6c7fd1", ec="#223", lw=0.8, alpha=0.6))
    # IMU PCB + pins + jumper housings (under the URT-2)
    ax.add_patch(Rectangle((IMU_CU - IMU_W / 2, IMU_V0 - IMU_PIN_OUT - IMU_JUMPER_L), IMU_PIN_PITCH_W, IMU_PIN_OUT + IMU_JUMPER_L, fc="#f39c12", ec="#333", alpha=0.9, zorder=5))
    ax.add_patch(Rectangle((IMU_CU - IMU_W / 2, IMU_V0), IMU_W, IMU_L, fc="#d62828", ec="#400", lw=1.0, alpha=0.95, zorder=6))
    ax.text(IMU_CU, IMU_CV - 1, "MPU6050\n22 x 29.5\n(under URT-2)", color="w", fontsize=7, ha="center", va="center", weight="bold", zorder=7)
    ax.text(IMU_CU, IMU_V0 - 14, "7 mm pins + 14 mm Dupont\nhousings, in-plane", fontsize=7, ha="center", va="center", zorder=7)
    ax.text(np.mean(URT_COL[:2]) + 4, np.mean(URT_COL[2:]) + 18, "FE-URT-2\n56.2 x 36.5", ha="center", va="center", fontsize=9, color="w", weight="bold", zorder=7)
    ax.text(np.mean(PI_COL[:2]) + 4, np.mean(PI_COL[2:]) + 6, "Pi Zero 2W\n+ PiSugar 3\n+ LiPo\n65 x 30", ha="center", va="center", fontsize=9, color="w", weight="bold")
    for ax_, ay_ in URT_HOLES:
        ax.add_patch(Circle(urt_xy(ax_, ay_), URT_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8, zorder=8))
    for p in pi_post_pts():
        ax.add_patch(Circle(p, PI_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8, zorder=8))
    for p in IMU_HOLES:
        ax.add_patch(Circle(p, IMU_POST_OD / 2, fc="#f1c40f", ec="#333", lw=0.8, zorder=8))
    for p in COVER_POSTS:
        ax.add_patch(Circle(p, COVER_POST_OD / 2, fc="#e67e22", ec="#333", lw=0.8, zorder=8))
        ax.annotate("M3x16\ncover screw", p, (p[0] + (11 if p[0] > -10 else 8), p[1] + 2), fontsize=7, arrowprops=dict(arrowstyle="-", lw=0.6), va="center")
    # header strip, hood zones
    hx0, hx1 = pi_xy(0, HDR_PY[1])[0], pi_xy(0, HDR_PY[0])[0]
    hv0, hv1 = pi_xy(HDR_PX[0], 0)[1], pi_xy(HDR_PX[1], 0)[1]
    ax.add_patch(Rectangle((hx0, hv0), hx1 - hx0, hv1 - hv0, fc="#222", ec="none", alpha=0.85, zorder=9))
    ax.text(hx0 - 0.5, hv1 + 1.0, "GPIO\n2x20", fontsize=6, ha="center", va="bottom", zorder=9)
    for r, ls, col, lab in [(HOOD_E_A, "-", "#8e44ad", f"roof {D_STRIP:.1f} mm over the strip"), (HOOD_E_B, "-", "#c0392b", f"hood {D_HOOD:.1f} mm over pins 1-17")]:
        ax.add_patch(Rectangle((r[0], r[2]), r[1] - r[0], r[3] - r[2], fill=False, ec=col, lw=1.8, ls=ls, zorder=12))
    ax.annotate(f"hood.stl: {D_HOOD:.1f} mm deep (pin-1 end)", (HOOD_E_B[1], 0.5 * (HOOD_E_B[2] + HOOD_E_B[3])), (HOOD_E_B[1] + 9, -48),
                fontsize=8, color="#c0392b", arrowprops=dict(arrowstyle="-", color="#c0392b"))
    ax.annotate(f"strip roof {D_STRIP:.1f} mm", (HOOD_E_A[1], HOOD_E_A[3] - 4), (HOOD_E_A[1] + 14, 44), fontsize=8, color="#8e44ad", arrowprops=dict(arrowstyle="-", color="#8e44ad"))
    ax.text(PI_BRD[1] - 15, hv0 - 3.2, "pin 1 / SD end", fontsize=7, ha="center", zorder=9)
    for k, (u, v) in BOSS_PACK.items():
        scr = k != "B4"
        ax.add_patch(Circle((u, v), 2.95 if scr else 2.75, fc="#c0392b" if scr else "none", ec="#c0392b", lw=1.3, ls="-" if scr else ":", zorder=10))
        ax.annotate(f"{k} M3 (pilot {BOSSES[k]['pilot']} mm)" if scr else f"{k} no hole: rest pad", (u, v), (u + (6 if u > 0 else -6), v + (5 if v > 0 else -6)),
                    fontsize=7, color="#c0392b", arrowprops=dict(arrowstyle="-", color="#c0392b", lw=0.6), ha="left" if u > 0 else "right", zorder=11)

    def arrow(txt, xy, dxy, **kw):
        ax.annotate(txt, xy, (xy[0] + dxy[0], xy[1] + dxy[1]), fontsize=8, ha=kw.get("ha", "center"), va=kw.get("va", "center"),
                    arrowprops=dict(arrowstyle="<-", color="#555"), color="#222")
    arrow(f"Pi micro-USB / HDMI + PiSugar USB-C\n+ 2 buttons: slot from rim to {PI_TOP + 3.3:.1f} mm", (W_OUT / 2, 0), (12, 0), ha="left")
    arrow("servo cables\n(3-pin ports)", (-W_OUT / 2, 10), (-11, 0), ha="right")
    arrow("camera ribbon", (np.mean(PI_BRD[:2]), H_OUT / 2), (0, 6))
    arrow("USB-C plug\n(URT-2)", (np.mean(URT_BRD[:2]), H_OUT / 2), (0, 6))
    arrow("SD card", (np.mean(PI_BRD[:2]), -H_OUT / 2), (0, -6))
    ax.set_xlim(-W_OUT / 2 - 22, W_OUT / 2 + 30); ax.set_ylim(-H_OUT / 2 - 12, H_OUT / 2 + 12)
    ax.set_title(f"Plan view from behind the robot (cover removed)   pack {W_OUT:.1f} x {H_OUT:.1f} mm   u = robot x (right), v = robot z (up)", fontsize=10)
    ax.set_xlabel("u [mm]"); ax.set_ylabel("v [mm]")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def make_previews(c_tm, v_tm, h_tm, elec, bat_p):
    os.makedirs(OUT_PREVIEW, exist_ok=True)
    C_CAR, C_COV, C_HOOD = (0.93, 0.93, 0.9), (0.98, 0.78, 0.2), (0.95, 0.5, 0.25)
    up = (0, 0, 1)
    print("rendering previews ...")
    # 1. exploded
    f = (-0.5, 0.75, -0.8)
    def off(n):
        if n == "MPU6050 IMU" or n.startswith("IMU pins"): return 9
        if n.startswith("screws: IMU"): return 20
        if n.startswith("screws: boss"): return 16
        if n.startswith("screws: URT") or n.startswith("screws: Pi"): return 24 + 18
        if n.startswith("screws: cover"): return 118
        return 24
    OFF_COVER, OFF_HOOD = 92, 118
    objs = [_obj(c_tm, C_CAR)] + [_obj(m, col, shift=(0, 0, off(n))) for n, m, col in elec]
    objs += [_obj(v_tm, C_COV, shift=(0, 0, OFF_COVER)), _obj(h_tm, C_HOOD, shift=(0, 0, OFF_HOOD))]
    img, pj = rasterize(objs, f, up, size=(1500, 1500), margin=0.1)
    labels = [
        ("COVER  (cover.stl)", (-25, 36, D + OFF_COVER - 0.5), "L"),
        ("HOOD  (hood.stl) over the GPIO strip", (HOOD_E_B[1] - 2, HOOD_E_B[2] + 8, D_HOOD + OFF_HOOD), "R"),
        ("2x M3x16 cover screws", (COVER_POSTS[0][0], COVER_POSTS[0][1], D + 118), "R"),
        ("Pi Zero 2W + PiSugar 3 + LiPo + 5 Dupont jumpers", (*pi_xy(40, 15), PI_TOP + 24), "R"),
        ("FE-URT-2 servo board on 8 mm standoffs", (*urt_xy(30, 10), URT_W0 + URT_TOTAL_H + 24 - 3), "L"),
        ("MPU6050 IMU (flat, under the URT-2)", (IMU_CU, IMU_CV + 4, IMU_PCB_W0 + 2 + 9), "L"),
        ("CARRIER  (carrier.stl)", (-30, 10, PLATE_T), "L"),
        ("3x M3 into the box bosses (B1-B3)", (*BOSS_PACK["B3"], PLATE_T - CBORE_DEPTH + M3_HEAD_H + 16), "L"),
    ]
    _save(img, os.path.join(OUT_PREVIEW, "1_exploded_assembly.png"), labels, pj,
          title="XGO-Duck backpack v2 - exploded view (seen from the robot's feet; the robot's back faces UP in this picture)")
    # 2. cover removed, URT ghosted
    f = (-0.45, 0.7, -0.8)
    objs = [_obj(c_tm, C_CAR)]
    for n, m, col in elec:
        if n.startswith("screws: cover"):
            continue
        objs.append(_obj(m, col, alpha=0.4 if n.startswith("FE-URT") else 1.0))
    img, pj = rasterize(objs, f, up, size=(1500, 1250), margin=0.1)
    _save(img, os.path.join(OUT_PREVIEW, "2_assembly_cover_removed.png"), [
        ("Pi + PiSugar stack (ports face the right wall)", (*pi_xy(32, 0), STACK_BOTTOM + 8), "R"),
        ("5 straight Dupont jumpers on pins 1,3,5,6,17", (*pi_xy(HDR_PX[0] + 5, 27), PI_TOP + 12), "R"),
        ("FE-URT-2 (ghosted)", (*urt_xy(40, 8), URT_W0 + 6), "L"),
        ("IMU under the URT-2 + in-plane jumpers", (IMU_CU, IMU_V0 - 8, IMU_PCB_W0 + 1), "L"),
        ("cover screw posts", (COVER_POSTS[0][0], COVER_POSTS[0][1], COVER_POST_TOP - 2), "R"),
    ], pj, title="Assembly with the cover removed (view from the robot's feet; back faces up)")
    # 3. back view
    objs = [_obj(v_tm, C_COV), _obj(h_tm, C_HOOD)]
    img, pj = rasterize(objs, (0, 0, -1), (0, 1, 0), size=(1300, 1250))
    def box_outline(ax, project):
        corners = np.array([[BOX_X[0] - U0, BOX_Z[0] - V0, 0], [BOX_X[1] - U0, BOX_Z[0] - V0, 0], [BOX_X[1] - U0, BOX_Z[1] - V0, 0],
                            [BOX_X[0] - U0, BOX_Z[1] - V0, 0], [BOX_X[0] - U0, BOX_Z[0] - V0, 0]])
        x, y = project(corners)
        ax.plot(x, y, "--", color="#c0392b", lw=1.3)
        ax.text(x[0] + 6, y[0] - 8, "battery box outline (behind)", color="#c0392b", fontsize=8)
    _save(img, os.path.join(OUT_PREVIEW, "3_back_view_cover_on.png"), [], pj,
          title=f"Back view: {W_OUT:.1f} x {H_OUT:.1f} mm, flat back, 45-deg rounded edges, hood bump on the right-centre", extra=box_outline)
    # 4. side view with ghosted battery box
    objs = [_obj(c_tm, C_CAR), _obj(v_tm, C_COV, 0.4), _obj(h_tm, C_HOOD, 0.5)] + [_obj(m, col) for n, m, col in elec] + [_obj(bat_p, (0.4, 0.45, 0.55), alpha=0.35)]
    img, pj = rasterize(objs, (1, 0, 0), (0, 1, 0), size=(1900, 1000))
    def dims(ax, project):
        p = lambda w, v: tuple(float(a[0]) for a in project(np.array([-30.0, v, w])))
        xb, _ = p(0, 0)
        ax.axvline(xb, color="#c0392b", lw=0.8, ls=":")
        ax.text(xb - 6, 62, "boss-tip plane (w = 0)", ha="right", color="#c0392b", fontsize=8)
        for dd, vv, lab in [(D, 33.0, f"{D:.1f} mm  roof (most of the pack)"), (D_STRIP, 4.0, f"{D_STRIP:.1f} mm  GPIO strip"), (D_HOOD, -33.5, f"{D_HOOD:.1f} mm  hood (pin 1-17)")]:
            (x0, y0), (x1, y1) = p(0, vv), p(dd, vv)
            ax.annotate("", (x1, y0), (x0, y0), arrowprops=dict(arrowstyle="<->", color="#c0392b"))
            ax.text(x1 + 8, y0, lab, ha="left", va="center", color="#c0392b", fontsize=9, weight="bold")
    _save(img, os.path.join(OUT_PREVIEW, "4_side_view_with_battery_box.png"), [], pj,
          title="Side view (robot back to the right): ghosted battery box left, ghosted cover + hood, boards inside", extra=dims)
    # 5. iso on battery box
    objs = [_obj(c_tm, C_CAR), _obj(v_tm, C_COV), _obj(h_tm, C_HOOD), _obj(bat_p, (0.35, 0.45, 0.6), alpha=0.28)]
    img, pj = rasterize(objs, (-0.75, -0.55, -0.62), (0, 1, 0), size=(1500, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "5_iso_on_battery_box.png"), [], pj, title="Finished backpack on the battery box (box ghosted), robot up = screen up")
    # 6/7/9 print orientation views
    cp = print_orient(v_tm, D)
    img, pj = rasterize([_obj(cp, C_COV)], (0.25, -0.35, -1), (0, 1, 0), size=(1400, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "6_cover_print_orientation.png"), [], pj, title="cover.stl in print orientation (roof on the bed, open side up), no supports")
    img, pj = rasterize([_obj(c_tm, C_CAR)], (0.2, 0.35, -1), (0, 1, 0), size=(1400, 1150))
    _save(img, os.path.join(OUT_PREVIEW, "7_carrier_print_orientation.png"), [], pj, title="carrier.stl in print orientation (flat face down), no supports")
    plan_view(os.path.join(OUT_PREVIEW, "8_plan_view_layout.png"))
    hp = hood_print_orient(h_tm)
    img, pj = rasterize([_obj(hp, C_HOOD)], (0.3, 0.9, -0.5), (0, 1, 0), size=(1400, 1000))
    _save(img, os.path.join(OUT_PREVIEW, "9_hood_print_orientation.png"), [], pj, title="hood.stl in print orientation (lying on its flat gap-side wall), no supports")
    print("previews written to", OUT_PREVIEW)


def main():
    os.makedirs(OUT_PREVIEW, exist_ok=True)
    print(f"pack outer {W_OUT:.1f} (u) x {H_OUT:.1f} (v); depth behind boss tips: roof {D:.1f}, GPIO strip {D_STRIP:.1f}, hood {D_HOOD:.1f} mm; cavity {W_IN:.1f} x {H_IN:.1f}")
    carrier, cover, hood = build_carrier(), build_cover(), build_hood()
    elec = electronics()
    c_tm, v_tm, h_tm = to_tm(carrier), to_tm(cover), to_tm(hood)
    print("mesh checks (pack frame):")
    check("carrier", c_tm); check("cover", v_tm); check("hood", h_tm)
    rep = {}
    print("collision checks (mm3, must be ~0):")
    cover_all = cover + hood
    for n, m, _ in elec:
        if n == "spacer":
            continue
        vc, vv = inter_vol(carrier, m), inter_vol(cover_all, m)
        rep[n] = dict(carrier=vc, cover_hood=vv)
        print(f"  {n:30s} carrier {vc:8.2f}   cover+hood {vv:8.2f}{'  <-- CHECK' if vc > 0.3 or vv > 0.3 else ''}")
    rep["carrier_vs_cover"] = inter_vol(carrier, cover_all)
    rep["hood_vs_cover"] = inter_vol(hood, cover)
    print(f"  carrier vs cover+hood: {rep['carrier_vs_cover']:.2f}   hood vs cover: {rep['hood_vs_cover']:.2f}")
    el = [(n, m) for n, m, _ in elec if n != "spacer"]
    for i in range(len(el)):
        for j in range(i + 1, len(el)):
            vol = inter_vol(el[i][1], el[j][1])
            if vol > 0.3:
                print(f"  OVERLAP {el[i][0]} / {el[j][0]}: {vol:.2f}")
    bat = trimesh.load(REF_BATTERY)
    bat_p = battery_to_pack(bat)
    bm = to_man(bat_p)
    rep["battery"] = dict(carrier=inter_vol(carrier, bm), cover=inter_vol(cover, bm), hood=inter_vol(hood, bm),
                          min_gap_carrier=carrier.min_gap(bm, 5.0), min_gap_cover=cover.min_gap(bm, 5.0))
    print("battery box fit:", {k: round(v, 3) for k, v in rep["battery"].items()})
    sink = lipo_sink_analysis(bat)
    rep["lipo_sink"] = sink
    print("LiPo sink analysis (free depth behind the tip plane under the LiPo footprint):", sink)
    # exports
    c_tm.export(os.path.join(HERE, "carrier.stl"))
    print_orient(v_tm, D).export(os.path.join(HERE, "cover.stl"))
    hood_print_orient(h_tm).export(os.path.join(HERE, "hood.stl"))
    trimesh.util.concatenate([pack_to_battery(c_tm), pack_to_battery(v_tm), pack_to_battery(h_tm)]).export(os.path.join(HERE, "backpack_assembled_battery_frame.stl"))
    trimesh.util.concatenate([to_tm(m) for n, m, _ in elec]).export(os.path.join(HERE, "electronics_boxes_pack_frame.stl"))
    print("re-loaded exports:")
    wt = {}
    for nm in ("carrier", "cover", "hood"):
        t = trimesh.load(os.path.join(HERE, nm + ".stl"))
        check(nm + ".stl", t)
        wt[nm] = bool(t.is_watertight)
    asm = trimesh.load(os.path.join(HERE, "backpack_assembled_battery_frame.stl"))
    rep.update(dict(outer=[W_OUT, H_OUT], depth=dict(roof=D, strip=D_STRIP, hood=D_HOOD), watertight=wt,
                    mass_g=dict(carrier=c_tm.volume / 1000 * PLA_DENSITY, cover=v_tm.volume / 1000 * PLA_DENSITY, hood=h_tm.volume / 1000 * PLA_DENSITY),
                    assembled_bounds_battery_frame=asm.bounds.tolist(),
                    imu_battery_xz=[IMU_CU + U0, IMU_CV + V0]))
    rep["mass_g"]["total"] = sum(rep["mass_g"].values())
    with open(os.path.join(HERE, "report.json"), "w") as f:
        json.dump(rep, f, indent=1, default=float)
    print("mass (solid PLA):", {k: round(v, 1) for k, v in rep["mass_g"].items()})
    print("assembled bounds (battery frame):", np.round(asm.bounds, 1).tolist())
    make_previews(c_tm, v_tm, h_tm, [(n, to_tm(m), col) for n, m, col in elec], bat_p)


if __name__ == "__main__":
    main()
