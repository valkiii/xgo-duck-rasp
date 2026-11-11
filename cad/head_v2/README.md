# XGO-Duck head v2 - bare Pi Zero 2W + OV5647 camera in the head

All coordinates below are in the **head frame = MJCF INIT world** (x forward, y left, z up, mm; head level, roll = 0).
Printable STLs are exported in the **original part frames** of `xgoduck_hardware/structure/` (same file origin as the original),
so they drop in where the originals were.  `*_world*.stl` are the same parts in the head frame (for CAD overlays only).

## Files to print / replace

| file | replaces | what changed |
|---|---|---|
| `head_shell_v2.stl` | `head_shell.stl` | local dome ridge (smooth warp, wall thickness unchanged); Ø13 lens bore through the eye membrane; 4 screw-head pockets; 2 camera-plate columns (Ø5.6, M2 pilots Ø1.7 x 6) |
| `eye_shell_v2.stl` | `eye_shell.stl` | rear bore enlarged Ø11 -> Ø12.6 (x 91.0..96.3) so the 8.5 mm square lens block (diag 12.0) passes |
| `eye_cam.stl` | `eye.stl` | cap only (hub and tabs removed), Ø7.5 lens hole + 70 deg countersink to Ø12.4 on the dome |
| `head_servo_support_v2.stl` | `head_servo_support.stl` | (a) 2.6 x 5 mm piece of the west cradle flange removed at x 23-26.4, y 14-19.2 (USB cable passes there); (b) 8 x 5.6 mm cable slot through the rear tab at x 14.5-22.5, y 14.2-19.8 |
| `head_v2.stl` | `head.stl` | matching 8 x 5.6 mm cable slot through the base floor at x 14.5-22.5, y 14.2-19.8 |
| `pi_tray.stl` (2 bodies) | new | rear beam + front beam, 3 mm thick, 4 posts (4.0 mm standoff, Ø5.5, M2.5 self-tap pilot Ø2.2); anchor holes **M3** (Ø3.4, head counterbore Ø6.4 x 1.5, round pad r 4.6 around each) |
| `tray_spacer.stl` x2 | new | tube Ø7.2 / Ø3.4 (**M3**), 17.7 long, spaces the tray beams off the support-plate tabs |
| `camera_mount.stl` | new | 39 x 20 x 1.6 plate, 4 bosses (Ø4.6, 1.2 tall, Ø1.7 M2 pilots 6 deep), 2 ear holes Ø2.5 |

Unchanged originals: `jaw.stl`, `head_yaw`, `head_pitch`, servos, TPU lips.  Everything above is watertight (manifold booleans, re-loaded and checked).

### Print orientation (PLA/PETG, 0.2 mm, 3 walls, 20 % infill)
* `head_shell_v2.stl`: dome up, rim on the bed (as the original).  The two camera columns are horizontal stubs: tree supports, or print the shell on its front face (then the dome ridge needs no support at all).
* `eye_cam.stl`: print with the flat back face on the bed, dome up (the file is in the original eye frame: rotate it so the file's +y axis points up; the flat face is at the smallest y).  Resolution 0.1 mm layers recommended (visible part).
* `eye_shell_v2.stl`: as the original.
* `pi_tray.stl`: as exported, flat bottom on the bed, posts up (no supports).  Two separate bodies - print both.
* `tray_spacer.stl`: standing up, 2 pieces.  `camera_mount.stl`: plate face down, bosses up.
* `head_servo_support_v2.stl`, `head_v2.stl`: as the originals.

### Screws (all self-tapping into PLA)
| use | screw | qty |
|---|---|---|
| Pi -> tray posts (holes Ø2.75, 58 x 23) | M2.5 x 6 pan head | 4 |
| tray beam -> support-plate tab through spacer, into the head pilot (the plate's own M2 screws at these two holes are replaced). **Existing printed parts: drill the plate's two holes to 3.2-3.4 mm and the head pilots below them to 2.5 mm, ~10 mm deep (hole A boss is only ~Ø5: drill straight, screw in slowly, stop when snug)**. `head_servo_support_v2`/`head_v2` already have Ø3.4 / Ø2.5. Tip ends at z 209.5: free space under hole B, solid floor (to z 205.5) under hole A. | M3 x 30 pan head (head sits in a Ø6.4 x 1.5 pocket) | 2  (holes (21.0, 25.0) and (67.1, 0.0)) |
| camera board -> mount bosses (heads face the lens, sit in the pockets of the membrane) | M2 x 6 **button head ISO 7380** (head 3.8 x 1.1) | 4 |
| camera plate ears -> shell columns (heads behind the plate) | M2 x 8 | 2 |
Total: 4 x M2.5x6, 2 x M3x30, 4 x M2x6 button, 2 x M2x8.

## Pose and keep-outs (all verified)
* Pi Zero 2W lying flat, **long axis along x, SD/pin-1 end toward the face (+x), camera (CSI) end toward the back, USB-port edge toward +y (robot left), header edge toward -y**.
  Stack coordinates -> world: x = 81.5 - X, y = 19 - Y, z = 243.5 + Z (X along the board from the SD end, Y from the USB edge, Z from the underside-parts level).
  PCB bottom z = 245.0, PCB top 246.4, top parts to 249.7, header pins to 254.9, Dupont reserve to 267.4 (21 mm) over x 52-75.7, y -10.4..-3.5.
* Why this pose (vs. USB edge toward the back, SD toward the back): see the pose study in `../pi_placement`; it is the only one with a small dome ridge (10.5 mm).  The other way round (header at +y, CSI at the front) needs 20 mm of dome and collides with the camera.
* **Dome** (ridge v2, 2026-11-08, after a test fit: the v1 ridge only covered pins 1-17): the 21 mm Dupont reserve now covers the WHOLE Pi length (x 14.5-83.5, y -10.4..-3.5) -> max local lift 16.5 mm at the back end of the Pi (x 14-20), 10-14 mm along the rest of the header, ~4-5 mm over the middle of the Pi; eye/face zone (x > 86, z < 246) and the rim (z < 230) unchanged.  Highest point of the head: 271.6 (v1 270.5, original 262.0).  v1 kept in `ridge_v1_backup/`, comparison `c5_ridge_old_vs_new.png`.  Wall thickness preserved vertically (thinner on the steep flank at x 5-15).
* **Moving parts checked** (voxel sweeps, 1 mm / 0.5 mm):
  * jaw 0-30 deg about the axis y through (x 41, z 225.4) (sign checked: + opens, no collision with head/shell): min distance of every new part to the swept jaw = 13.0 mm (spacers), tray 17.8, Pi 14.1, camera 16.1, cables 17.6-18.5.
  * head roll +-25 deg about the x axis at (y 0, z 222.6): the yaw servo and the head_yaw bracket (and cables on them) sweep inside the shell up to z = 245; min distance of new parts to that sweep: Pi envelope 1.0 (0.8 mm design clearance), USB cable 1.0, wire bundle 1.5, tray 3.0, spacers 6.0, ribbon 12, camera >27.  (Software-limiting roll to +-15 deg would lower the Pi by about 2 mm = about 2 mm less ridge.)
  * head yaw / head pitch move the whole head relative to the neck; nothing in the head moves relative to the shell for those.
  * static overlaps of every new part with shell, head, support, eye shell, S_front, S_rear, S_yaw: 0 mm3 (camera lens/PCB vs shell 0.1 mm3 = contact).

## USB (data) - right-angle micro-USB
* Port: micro-USB marked "USB", centre 41.4 mm from the SD end, on the long edge opposite the header.  Plug keep-out used: 12 (along edge) x 11 (out) x 9 mm high, z = PCB bottom -2 ... +7 (world x 34.1-46.1, y 19-30, z 243-252).
* **Variant to buy: right-angle where the cable leaves the plug sideways, parallel to the board edge, toward the CSI/camera end of the Pi (away from the SD card).**
  Held like the usual Pi Zero picture (SD card at the left, USB ports on the bottom edge, components up) the cable goes to the RIGHT, plug flat side up.
  Vendors define "left/right angle" differently: look at the product photo - the cable must leave toward the right in that picture, not up or down.
  In the head this means the cable leaves toward the rear of the head (-x) and then turns down into the neck (the Pi pose cannot face the port edge to the back because the Pi is 65 long and the head is only 88 wide at that height).
* Cable: soft silicone, Ø <= 3.5 mm, micro-USB right-angle -> USB-C (URT-2 side).
* In-head route (spline checked against all parts, clearance >= 2.0 mm to every surface for r 1.75): exit (34.1, 27.5, 247.5) -> down the pocket at (27.4, 16.6) behind S_front through the removed flange -> leaves the head floor at z 212.  Length 45 mm; bend radius 6 mm on the first turn, >= 4.9 mm minimum at one point, >= 10 mm elsewhere.  (The requested 10 mm radius is not possible here: the pocket is 6 mm wide; use a soft cable.)
* **Length: 30 cm.**  Budget: plug body 12 + in head 45 + head floor -> torso top about 80 (neck rear face, 40 mm slack for the 4 neck joints included separately) + torso to URT-2 about 100 + 40 slack = about 280 mm.  If the URT-2 sits lower/further back than assumed, take 50 cm.
* 7 Dupont wires (pins 1,3,4,5,6,14,17): out of the Dupont zone toward -y at (64, -10.8, 257.5), along the left flank (z 248-251, clearance to the dome >= 3 mm), around the rear end of the Pi at x 7-9, then forward to the gate at (18, 16.5) and through the slot in tab + head floor.  Bundle Ø <= 3.8, in-head length 125 mm, min bend radius 5.9 mm.  Use 20 cm wires (the in-head run alone is 125 mm) + the neck.

## CSI camera (ASSUMED OV5647, rpi-cam-v1 form factor - TO MEASURE)
* Mounted **upside down** (FFC connector on the top edge) so the ribbon runs back over the top of the Pi: set `rotation=180` / `--rotation 180` in the camera software.
* Lens axis = eye axis (y 0, z 237.1).  PCB lens-side face on the rear face of the membrane (x = 89.3), PCB x 88.3-89.3, lens block 8.5 x 8.5 x 5.5 to x 94.8 (inside the Ø12.6 bore), lens tip about 4.1 mm behind the eye apex (98.9), 1.4 mm behind the cap's rear face, through the Ø7.5 hole + countersink (no vignetting for 54 x 41 deg: hole edge 3.75 vs lens 1.5 at 1.4 mm distance).  If you want the lens closer to the surface, the membrane must be pocketed (not done).
* Board holes (21 x 12.5, 2 mm from edges) -> bosses of `camera_mount.stl`; plate back face x = 85.5, front 87.1; plate ears screw to the two new shell columns at y = +-16.5, z = 237.1.
* Ribbon (22 -> 15 pin, assumed 16 cm): needed length about 90 mm (camera connector (87, 0, 250.6) -> up, back over the Pi at z about 252.4 (2.2 mm above the top parts, 5.5 mm sideways drift to the CSI centre) -> U-turn behind the rear edge of the Pi (radius 2.2-2.7 mm, apex x = 14) -> CSI at x = 16.5).  Take a 10-12 cm ribbon if possible; with 16 cm leave a loose slack loop on top of the Pi.  Position of the CSI connector along the short edge assumed centred (TO MEASURE).
* The Pi's SD card end is 3.5 mm behind the camera plate: remove the shell (and pull the Pi tray) to change the card; no SD slot (it would face the eye assembly).

## Assembly order
1. Print; deburr; test-fit the lens block in the bore of `eye_shell_v2` (Ø12.6) and the tray on the plate.
2. Replace `head`, `head_servo_support` by the v2 parts (re-using servos and screws).  Do NOT fit the removed flange segment back.
3. Fix the tray: drill the two tab holes to 3.2-3.4 and the head pilots to 2.5 (unless reprinted as v2), spacer on each hole, beam on top, 2 x M3x30 down into the head pilots.
4. Pi onto tray posts (4 x M2.5x6).  Plug the 7 Dupont wires (pins 1,3,4,5,6,14,17) and the USB plug first, route cables as drawn (`b3_sections_plan.png`, `b1_sections_xz.png`).
5. Camera: fix `camera_mount` to the two shell columns (2 x M2x8 from the rear), board onto its bosses (4 x M2x6 button head from the front, heads go into the pockets), eye shell + `eye_cam` glued from the front, ribbon over the top of the Pi.
6. Close the shell over the head (original fixing).

## Mass added to the head (estimate)
Pi Zero 2W 10 g + camera 3 g + tray 3.3 g solid (~2 g printed) + 2 spacers 0.9 g + camera mount 1.6 g (~1 g) + ribbon/Dupont/USB in head 3 g + screws 2 g + shell/eye/support/head changes -1 g  => **about 21-25 g** (vs 55 g with the PiSugar).  Head group 214 g -> about 237 g (+11 %).

## ASSUMED / TO MEASURE
* Pi Zero 2W header: 2 x 20, X 7.0-58.0 from the SD end, housing at Y 22.6-27.6 from the USB edge, pins 8.5 above the PCB; CSI connector mid-edge; underside parts <= 1.5, top parts <= 3.3.
* micro-USB plug body 12 x 11 x 9 and cable exit at the body's rear side face; Dupont reserve 21 / 10 mm.
* OV5647 board: 25 x 24 x 1, holes 21 x 12.5 / 2 mm, lens block 8.5 sq x 5.5, connector on the back of the top edge, 2.5 mm tall; total depth 9.
* Support plate screw holes are M2 originally (Ø2.2 in the plate, Ø1.8 pilot in head.stl); switched to M3 on 2026-11-07 (user request): plate holes -> Ø3.4, head pilots -> Ø2.5; M2 versions of tray/spacer/support/head kept in `m2_backup/`. Wall around the head pilot at hole A is ~1 mm (boss ~Ø4.6), at hole B >= 2 mm.
* Registration of the printable STLs to the MJCF meshes has an ICP error of about 1 mm (RMS 0.4-1.4); fit-check the first print (tray and camera plate) before printing the shell.
* Print orientation of the shell/eye is not known from the repository; chosen as above.

## Preview images
* `b1_sections_xz.png`, `b2_sections_yz.png`, `b3_sections_plan.png`: sections showing Pi, header + Dupont zone, USB plug and cable, wire bundle, ribbon, camera + mount, tray, old (grey) vs new (blue) shell.
* `b4_jaw_roll_sweep_check.png`: jaw 0/10/20/30 deg and clearance to jaw and head-roll sweeps.
* `b5_exploded.png`: exploded view.
* Scripts (all reproducible, run with `../../.venv/bin/python`): `design.py` (pose), `solids.py`, `build_parts.py`, `route2.py`, `route_wires.py`, `final_need.py`, `verify_v2.py`, `previews.py`.
