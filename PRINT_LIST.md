# XGO-Duck: parts to print (Pi Zero 2W build)

All files are drop-in replacements in the same frame as the original Luwu parts. Print in PLA/PETG, 0.2 mm, 3 walls, 20 % infill unless noted.

## 1. Battery box (print first)
`cad/battery_box_urt2/battery_urt2.stl`, replaces `battery.stl`. Print it like the original, no supports.
- **URT-2 servo board:** on the 4 new tall posts on the back (4× M3×6). Blue screw terminal at the top, USB-C at the bottom.
- **MPU6050:** flat on the 2 small posts **under** the URT-2 (2× M2.5×4), header pins pointing down. Its jumper wires leave through the notch at the bottom of the box.
- The 2S battery stays inside the box as before.

## 2. Head: test parts (small, quick, check the fit before the shell)
In `cad/head_v2/`:
- `pi_tray.stl`: the Pi Zero 2W screws onto it (4× M2.5×6).
- `tray_spacer.stl` ×2: under the tray. The tray is fixed with 2× **M3×30** into two of the support plate's existing holes, replacing the plate's own M2 screws there. **Drill those two plate holes to 3.2 mm and the head holes under them to 2.5 mm (~10 mm deep)** so the M3 fits; at hole A the plastic around the hole is thin, so drill straight and tighten gently.
  The tray is 2 separate crosswise beams (rear + front), one spacer + one screw each. Remove the plate's short M2 screw at hole A (rear, robot-left corner) and hole B (front, centre line), stand a spacer on the hole, lay the beam on top, M3×30 down through beam + spacer + plate into the head. Tray before Pi (screw B ends up under the Pi). Pi right side up: components and header pins pointing UP, not down. Drawing: `cad/head_v2/c1_tray_placement.png`.
- `camera_mount.stl`: the DAOKAI camera screws onto it (4× M2×6 button head). The plate then screws to the 2 posts inside the head shell (2× M2×8).

## 3. Head: main parts
In `cad/head_v2/`:
- `head_shell_v2.stl`, replaces `head_shell.stl` (**re-made 2026-11-08: the raised room now runs over the whole length of the Pi; reprint it**): taller ridge on top for the plugged-in jumper wires, the 2 camera posts inside the front wall, and the lens hole. Dome up, with tree supports for the camera posts, or print it on its front face (no supports).
- `eye_shell_v2.stl`, replaces `eye_shell.stl` (wider opening for the lens).
- `eye_cam.stl`, replaces `eye.stl` (eye with a Ø7.5 mm lens hole). Flat back on the bed, 0.1 mm layers (visible part).

## 4. Optional (only small cable slots, can be cut instead)
- `head_v2.stl` and `head_servo_support_v2.stl`, replacing `head.stl` and `head_servo_support.stl`: they only add an 8 × 5.6 mm cable slot each. You can cut these slots into your existing parts with a knife or drill instead (positions in `cad/head_v2/README.md`).

## What goes where
- **Torso, back of the battery box:** URT-2, MPU6050, UBEC, the 2 power Wago clamps, the servo Y cable (3 Wago clamps).
- **Head:** Pi Zero 2W on the tray (no PiSugar), camera behind the eye, ribbon from the camera over the top of the Pi to its camera connector.
- **Pi jumper wires in the head:** header → right side of the Pi → around its back end → down through the cable slot next to spacer A (drawings: `cad/head_v2/c2_wire_route.png`, `cad/head_v2/c3_cable_routes.png` = USB + Dupont with the beak). Use ~40 cm female-female jumpers so they reach the torso without joints.
- **Through the neck:** the link to the URT-2 (USB cable, or 4 thin wires), the 5 MPU6050 wires and the 2 UBEC power wires. Leave a little slack at each joint.
- Wiring diagram: `wiring_v1.png`. Build details: `cad/head_v2/README.md`.
