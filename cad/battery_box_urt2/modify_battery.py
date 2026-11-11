#!/usr/bin/env python3
"""
Modify structure/battery.stl (XGO-Duck 2S-18650 box) so its BACK face (-y) carries
a Feetech FE-URT-2 servo board + an MPU6050 IMU instead of the Arduino-UNO stack.

Coordinate frame is the STL's own (drop-in): x = left/right, z = up/down,
y = depth, the back of the box is toward -y.  "Height above the back wall" below
means distance toward -y from the plate's outer surface (y = BACK_WALL_OUT_Y).

Run:  /Users/vc/Research/xgo-duck/.venv/bin/python modify_battery.py
Out:  battery_urt2.stl, standins_*.stl, preview_*.png, report.txt
"""
import os, sys, json, warnings
import numpy as np
import trimesh
from shapely.geometry import Point, Polygon

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "ref", "xgoduck_hardware", "structure", "battery.stl")
OUT = os.path.join(HERE, "battery_urt2.stl")

# ------------------------------------------------------------------ measured on battery.stl
BACK_WALL_OUT_Y = -20.6     # outer surface of the back plate (min y of the plate)
BACK_WALL_IN_Y = -18.6      # inner surface of the back plate (cavity side)
OLD_BOSS_TIP_Y = -24.6      # tips of the old UNO bosses / side rails
BOX_X_LIMIT = 26.9          # |x| outline of the box, new features must stay inside
BOX_Z_MIN, BOX_Z_MAX = -35.0, 36.5

# ------------------------------------------------------------------ FE-URT-2 (user measured)
URT_PCB_X = 36.5            # PCB short side -> along x
URT_PCB_Z = 56.2            # PCB long side  -> along z (vertical)
URT_PCB_T = 1.6             # PCB thickness (assumed)
URT_HOLE_DX = 29.2          # hole spacing along x (36.5 - 2*3.65)
URT_HOLE_DZ = 49.2          # hole spacing along z (56.2 - 2*3.5)
URT_CENTER_X = 0.0
URT_CENTER_Z = 0.75         # = centre of the box's z range (-35..36.5)
URT_TERMINAL_SIDE = "+z"    # blue screw terminal on the top short edge, USB-C at the bottom
URT_STANDOFF_H = 9.0        # standoff height above BACK_WALL_OUT_Y  (>= 6.5 required)
URT_STANDOFF_OD = 6.0
URT_PILOT_D = 2.5           # M3 self-tapping (use 2.2 for M2.5)
URT_PILOT_DEPTH = 6.0       # measured from the standoff top

# ------------------------------------------------------------------ MPU6050 (user measured)
IMU_W = 22.0                # along x
IMU_L = 29.5                # PCB length along z
IMU_PCB_T = 1.6
IMU_HOLE_SPACING = 16.5
IMU_HOLE_INSET = 2.5        # hole centres from the 22 mm edge opposite the header
IMU_PIN_LEN = 7.0           # right-angle pins in the board plane
IMU_JUMPER_LEN = 14.0       # Dupont housings, in-plane
IMU_STACK_H = 4.5           # max height of board+pins+jumpers above its mounting surface
IMU_BOSS_H = 2.0            # IMU standoff height above BACK_WALL_OUT_Y
IMU_BOSS_OD = 5.5
IMU_PILOT_D = 2.2           # M2.5 self-tapping
IMU_PILOT_DEPTH = 3.0       # from boss top, runs 1.0 mm into the 2.0 mm plate (1.0 mm left)
IMU_JUMPER_TIP_Z = -34.0    # jumper ends point to the box bottom (-z); wires leave downward
IMU_CENTER_X = 0.0

SECTIONS = 64
EPS = 0.05


# ------------------------------------------------------------------ helpers
def cyl_y(x, z, y_a, y_b, r):
    """cylinder along y from y_a to y_b centred at (x, z)"""
    y0, y1 = min(y_a, y_b), max(y_a, y_b)
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=SECTIONS)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1) / 2, z])
    return c


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def section_xz(mesh, y):
    s = mesh.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0])
    res = Polygon()
    polys = sorted([Polygon(e[:, [0, 2]]) for e in s.discrete], key=lambda p: -p.area)
    for p in polys:
        res = res.symmetric_difference(p)
    return res


def vol_inter(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold")
        return 0.0 if i is None or len(i.faces) == 0 else abs(i.volume)
    except Exception:
        return 0.0


# ------------------------------------------------------------------ layout (derived)
URT_HOLES = [(URT_CENTER_X + sx * URT_HOLE_DX / 2, URT_CENTER_Z + sz * URT_HOLE_DZ / 2)
             for sz in (1, -1) for sx in (-1, 1)]
URT_TOP_Y = BACK_WALL_OUT_Y - URT_STANDOFF_H          # standoff top = PCB underside
URT_PCB_TOP_Y = URT_TOP_Y - URT_PCB_T

IMU_PIN_END_Z = IMU_JUMPER_TIP_Z + IMU_JUMPER_LEN      # where the jumpers meet the pins
IMU_PCB_Z0 = IMU_PIN_END_Z + IMU_PIN_LEN               # header edge of the PCB
IMU_PCB_Z1 = IMU_PCB_Z0 + IMU_L                        # mounting-hole edge of the PCB
IMU_HOLE_Z = IMU_PCB_Z1 - IMU_HOLE_INSET
IMU_HOLES = [(IMU_CENTER_X + s * IMU_HOLE_SPACING / 2, IMU_HOLE_Z) for s in (-1, 1)]
IMU_BOSS_TOP_Y = BACK_WALL_OUT_Y - IMU_BOSS_H


def build():
    base = trimesh.load(SRC)
    assert base.is_watertight

    # ---- verify the measurements the constants rely on
    wall = section_xz(base, (BACK_WALL_OUT_Y + BACK_WALL_IN_Y) / 2 - 0.5)   # solid plate level
    solid = section_xz(base, -19.0)
    assert section_xz(base, BACK_WALL_OUT_Y - 0.2).area < 0.2 * solid.area, "plate not at expected y"
    assert section_xz(base, BACK_WALL_OUT_Y + 0.2).area > 0.8 * solid.area
    for name, (x, z) in ([(f"URT{i}", p) for i, p in enumerate(URT_HOLES)]
                         + [(f"IMU{i}", p) for i, p in enumerate(IMU_HOLES)]):
        r = (URT_STANDOFF_OD if name.startswith("URT") else IMU_BOSS_OD) / 2
        miss = Point(x, z).buffer(r).difference(solid).area
        assert miss < 1e-6, f"{name} footprint not fully on the back plate ({miss:.2f} mm2 overhangs)"
        assert abs(x) + r <= BOX_X_LIMIT and BOX_Z_MIN <= z - r and z + r <= BOX_Z_MAX
    assert URT_STANDOFF_H >= 6.5
    assert IMU_BOSS_H + IMU_STACK_H <= URT_STANDOFF_H, "IMU stack would hit the URT-2"

    # ---- additive features
    adds = []
    for x, z in URT_HOLES:
        adds.append(cyl_y(x, z, BACK_WALL_OUT_Y + 1.4, URT_TOP_Y, URT_STANDOFF_OD / 2))
    for x, z in IMU_HOLES:
        adds.append(cyl_y(x, z, BACK_WALL_OUT_Y + 1.4, IMU_BOSS_TOP_Y, IMU_BOSS_OD / 2))
    m = trimesh.boolean.union([base] + adds, engine="manifold")

    # ---- pilot holes
    cuts = []
    for x, z in URT_HOLES:
        cuts.append(cyl_y(x, z, URT_TOP_Y - EPS, URT_TOP_Y + URT_PILOT_DEPTH, URT_PILOT_D / 2))
    for x, z in IMU_HOLES:
        cuts.append(cyl_y(x, z, IMU_BOSS_TOP_Y - EPS, IMU_BOSS_TOP_Y + IMU_PILOT_DEPTH, IMU_PILOT_D / 2))
    m = trimesh.boolean.difference([m] + cuts, engine="manifold")
    m.merge_vertices()
    m.remove_unreferenced_vertices()
    return base, m


def standins():
    urt_pcb = box(URT_CENTER_X - URT_PCB_X / 2, URT_CENTER_X + URT_PCB_X / 2,
                  URT_PCB_TOP_Y, URT_TOP_Y,
                  URT_CENTER_Z - URT_PCB_Z / 2, URT_CENTER_Z + URT_PCB_Z / 2)
    # conservative IMU envelope: whole 50.5 mm x 22 mm x IMU_STACK_H slab above the bosses
    imu_env = box(IMU_CENTER_X - IMU_W / 2, IMU_CENTER_X + IMU_W / 2,
                  IMU_BOSS_TOP_Y - IMU_STACK_H, IMU_BOSS_TOP_Y,
                  IMU_JUMPER_TIP_Z, IMU_PCB_Z1)
    imu_pcb = box(IMU_CENTER_X - IMU_W / 2, IMU_CENTER_X + IMU_W / 2,
                  IMU_BOSS_TOP_Y - IMU_PCB_T, IMU_BOSS_TOP_Y, IMU_PCB_Z0, IMU_PCB_Z1)
    return urt_pcb, imu_env, imu_pcb


# ------------------------------------------------------------------ previews
def project(v, d, up=(0, 0, 1)):
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    fwd = -d
    r = np.cross(fwd, up); r /= np.linalg.norm(r)
    u = np.cross(r, fwd)
    return np.c_[v @ r, v @ u], v @ d


def faces_for(mesh, d, color_fn, alpha=1.0, light=(0.4, -0.5, 0.8)):
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    n = mesh.face_normals
    vis = n @ d > -0.02 if alpha == 1.0 else np.ones(len(n), bool)
    L = np.asarray(light, float); L /= np.linalg.norm(L)
    sh = 0.45 + 0.55 * np.clip(np.abs(n @ L), 0, 1)
    cols = color_fn(mesh)
    out = []
    xy, dep = project(mesh.vertices, d)
    for i in np.where(vis)[0]:
        f = mesh.faces[i]
        c = np.array(cols[i][:3]) * sh[i]
        out.append((dep[f].mean(), xy[f], (*c, alpha)))
    return out


def render(items, d, ax):
    from matplotlib.collections import PolyCollection
    allf = []
    for mesh, fn, a in items:
        allf += faces_for(mesh, d, fn, a)
    allf.sort(key=lambda t: t[0])
    pc = PolyCollection([t[1] for t in allf], facecolors=[t[2] for t in allf],
                        edgecolors=[t[2] for t in allf], linewidths=0.15)
    ax.add_collection(pc)
    ax.autoscale_view(); ax.set_aspect("equal")


def previews(base, m, urt_pcb, imu_env, imu_pcb):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    BB = dict(fc="white", ec="none", alpha=0.8, pad=1.5)
    GREY, ORANGE = (0.62, 0.64, 0.68), (0.95, 0.5, 0.1)
    new_xz = np.array(URT_HOLES + IMU_HOLES)
    new_r = np.array([URT_STANDOFF_OD / 2 + 0.2] * 4 + [IMU_BOSS_OD / 2 + 0.2] * 2)

    def part_color(mesh):
        c = mesh.triangles_center
        d = np.hypot(c[:, None, 0] - new_xz[None, :, 0], c[:, None, 2] - new_xz[None, :, 1])
        new = ((d < new_r[None, :]).any(1)) & (c[:, 1] < BACK_WALL_OUT_Y + 0.5)
        return [ORANGE if k else GREY for k in new]
    flat = lambda col: (lambda mesh: [col] * len(mesh.faces))

    # ---------------- back view
    fig, ax = plt.subplots(figsize=(9, 10.5))
    render([(m, part_color, 1.0)], (0, -1, 0), ax)
    ax.add_patch(Rectangle((URT_CENTER_X - URT_PCB_X / 2, URT_CENTER_Z - URT_PCB_Z / 2), URT_PCB_X, URT_PCB_Z,
                           fill=False, ec="#0a58ca", lw=2))
    for x, z in URT_HOLES:
        ax.add_patch(Circle((x, z), 1.5, fill=False, ec="#0a58ca", lw=1.5))
    ax.text(URT_CENTER_X, URT_CENTER_Z + URT_PCB_Z / 2 - 3.0, "FE-URT-2 56.2 x 36.5  -  screw terminal (top)",
            ha="center", color="#0a58ca", fontsize=9, weight="bold", bbox=BB)
    ax.text(URT_CENTER_X, URT_CENTER_Z - URT_PCB_Z / 2 - 2.2, "USB-C (bottom edge)", ha="center",
            color="#0a58ca", fontsize=9, bbox=BB)
    ax.add_patch(Rectangle((IMU_CENTER_X - IMU_W / 2, IMU_PCB_Z0), IMU_W, IMU_L, fill=False, ec="#c0392b", lw=2, ls="--"))
    ax.add_patch(Rectangle((IMU_CENTER_X - IMU_W / 2, IMU_JUMPER_TIP_Z), IMU_W, IMU_PIN_LEN + IMU_JUMPER_LEN,
                           fill=False, ec="#c0392b", lw=1.2, ls=":"))
    ax.annotate("", xy=(0, IMU_JUMPER_TIP_Z - 0.5), xytext=(0, IMU_PCB_Z0 - 4), arrowprops=dict(arrowstyle="->", color="#c0392b", lw=2))
    ax.text(0, IMU_PCB_Z0 + IMU_L / 2, "MPU6050 (under URT-2)\n22 x 29.5 board\ncomponent side out (-y)",
            ha="center", va="center", color="#c0392b", fontsize=9, weight="bold", bbox=BB)
    ax.text(0, IMU_JUMPER_TIP_Z + 8, "pins+Dupont\nwires exit down", ha="center", color="#c0392b", fontsize=8, bbox=BB)
    for x, z in IMU_HOLES:
        ax.add_patch(Circle((x, z), 1.5, fill=False, ec="#c0392b", lw=1.5))
    # old UNO bosses
    for (x, z) in [(-8.9, 33.2), (19.0, 33.2), (-24.1, -17.6), (24.1, -18.9)]:
        ax.text(x, z, "UNO\n(kept)", ha="center", va="center", fontsize=6, color="k")
    ax.add_patch(Rectangle((-BOX_X_LIMIT, BOX_Z_MIN), 2 * BOX_X_LIMIT, BOX_Z_MAX - BOX_Z_MIN, fill=False, ec="k", lw=0.6, ls=":"))
    ax.annotate("+x", xy=(32, -38), xytext=(24, -38), arrowprops=dict(arrowstyle="->"), va="center")
    ax.annotate("+z", xy=(-31, -28), xytext=(-31, -36), arrowprops=dict(arrowstyle="->"), ha="center")
    ax.set_xlim(-32, 32); ax.set_ylim(-40, 41)
    ax.set_title("battery_urt2.stl - BACK view (looking along +y, x right, z up)\n"
                 "orange = new standoffs/bosses; grey = original", fontsize=11)
    ax.grid(alpha=.25); ax.set_xlabel("x [mm]"); ax.set_ylabel("z [mm]")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "preview_back.png"), dpi=130); plt.close(fig)

    BLUE, RED = (0.2, 0.45, 0.85), (0.8, 0.25, 0.2)
    urt_pcb = urt_pcb.subdivide_to_size(3.0)
    imu_env = imu_env.subdivide_to_size(3.0)
    # ---------------- iso view
    fig, ax = plt.subplots(figsize=(10, 10))
    render([(m, part_color, 1.0), (urt_pcb, flat(BLUE), 0.55), (imu_env, flat(RED), 0.45)], (0.75, -1, 0.6), ax)
    ax.set_title("battery_urt2.stl - iso from back (URT-2 blue, IMU envelope red = stand-ins)")
    ax.axis("off"); fig.tight_layout(); fig.savefig(os.path.join(HERE, "preview_iso.png"), dpi=130); plt.close(fig)

    # ---------------- side view (from +x, y to the right)
    fig, ax = plt.subplots(figsize=(13, 7.5))
    render([(m, part_color, 1.0), (urt_pcb, flat(BLUE), 0.6), (imu_env, flat(RED), 0.5)], (1, 0, 0), ax)
    ys = [(BACK_WALL_OUT_Y, "back wall outer\ny=%.1f" % BACK_WALL_OUT_Y, "k"),
          (OLD_BOSS_TIP_Y, "old UNO boss tips\ny=%.1f" % OLD_BOSS_TIP_Y, "gray"),
          (URT_TOP_Y, "URT-2 underside\ny=%.1f (+%.1f)" % (URT_TOP_Y, URT_STANDOFF_H), "#0a58ca"),
          (URT_PCB_TOP_Y, "URT-2 PCB top\ny=%.1f (+%.1f)" % (URT_PCB_TOP_Y, URT_STANDOFF_H + URT_PCB_T), "#0a58ca")]
    for y, lab, c in ys:
        ax.axvline(y, color=c, lw=0.8, ls="--")
        k = [t[0] for t in ys].index(y)
        ax.text(y, 40 + 7 * k, lab.replace("\n", "  "), color=c, ha="right" if k % 2 == 0 else "left", fontsize=8,
                bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    ax.set_xlim(-48, 8); ax.set_ylim(-37, 70)
    ax.set_xlabel("y [mm] (back of box to the left)"); ax.set_ylabel("z [mm]")
    ax.set_title("side view from +x: depth behind the back wall = %.1f mm to the URT-2 underside, %.1f mm to its PCB top"
                 % (URT_STANDOFF_H, URT_STANDOFF_H + URT_PCB_T), fontsize=10)
    ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(os.path.join(HERE, "preview_side.png"), dpi=130); plt.close(fig)


# ------------------------------------------------------------------ main
if __name__ == "__main__":
    base, m = build()
    m.export(OUT)
    m = trimesh.load(OUT)
    urt_pcb, imu_env, imu_pcb = standins()
    urt_pcb.export(os.path.join(HERE, "standin_urt2.stl"))
    imu_env.export(os.path.join(HERE, "standin_imu_envelope.stl"))

    rep = {}
    rep["watertight"] = bool(m.is_watertight)
    rep["winding_consistent"] = bool(m.is_winding_consistent)
    rep["volume_mm3"] = float(m.volume)
    rep["bounds"] = m.bounds.round(3).tolist()
    rep["orig_bounds"] = base.bounds.round(3).tolist()
    # everything that was removed from / added to the original
    removed = trimesh.boolean.difference([base, m], engine="manifold")
    added = trimesh.boolean.difference([m, base], engine="manifold")
    rep["removed_from_original_mm3"] = 0.0 if len(removed.faces) == 0 else float(removed.volume)
    rep["removed_bounds"] = None if len(removed.faces) == 0 else removed.bounds.round(2).tolist()
    rep["added_mm3"] = float(added.volume)
    rep["added_bounds"] = added.bounds.round(2).tolist()
    # keep-clear checks
    rep["collision_urt2_pcb_mm3"] = vol_inter(m, urt_pcb)
    rep["collision_imu_envelope_mm3"] = vol_inter(m, imu_env)
    rep["urt2_x_range"] = [URT_CENTER_X - URT_PCB_X / 2, URT_CENTER_X + URT_PCB_X / 2]
    rep["urt2_z_range"] = [URT_CENTER_Z - URT_PCB_Z / 2, URT_CENTER_Z + URT_PCB_Z / 2]
    rep["imu_z_range"] = [IMU_JUMPER_TIP_Z, IMU_PCB_Z1]
    rep["urt2_underside_y"] = URT_TOP_Y
    rep["urt2_pcb_top_y"] = URT_PCB_TOP_Y
    rep["depth_to_urt2_underside"] = URT_STANDOFF_H
    rep["depth_to_urt2_top"] = URT_STANDOFF_H + URT_PCB_T
    rep["depth_beyond_old_boss_tips"] = OLD_BOSS_TIP_Y - URT_PCB_TOP_Y
    rep["imu_clearance_to_urt2_mm"] = URT_STANDOFF_H - IMU_BOSS_H - IMU_STACK_H
    rep["urt_holes_xz"] = URT_HOLES
    rep["imu_holes_xz"] = IMU_HOLES
    # new-feature extent (must stay inside |x|<=26.9)
    rep["new_feature_max_abs_x"] = float(max(abs(x) for x, _ in URT_HOLES) + URT_STANDOFF_OD / 2)
    # nothing but the new features changed outside the pilot-hole region:
    rep["cavity_side_untouched"] = bool(len(removed.faces) == 0 or removed.bounds[1][1] < BACK_WALL_IN_Y - 0.5)
    # pilot hole probe: ray from far behind along +y at each hole centre must travel the pilot depth
    for tag, holes, top, depth in (("urt", URT_HOLES, URT_TOP_Y, URT_PILOT_DEPTH),
                                   ("imu", IMU_HOLES, IMU_BOSS_TOP_Y, IMU_PILOT_DEPTH)):
        ds = []
        for x, z in holes:
            hit, _, _ = m.ray.intersects_location([[x, top - 5, z]], [[0, 1, 0]])
            ys = sorted(h[1] for h in hit)
            ds.append(round(ys[0] - top, 2) if ys else None)
        rep[f"pilot_depth_measured_{tag}"] = ds
    print(json.dumps(rep, indent=1))
    json.dump(rep, open(os.path.join(HERE, "report.json"), "w"), indent=1)
    previews(base, m, urt_pcb, imu_env, imu_pcb)
