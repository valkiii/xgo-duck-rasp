# XGO Duck on a Raspberry Pi Zero 2 W

A port of [LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino) from the Arduino UNO Q to a Raspberry Pi, with the servos on a Feetech URT-2 and an MPU6050 IMU.

The UNO Q is two computers: Linux runs the 50 Hz policies and web UI, and an STM32 runs the 100 Hz servo/IMU loop. Here, the Linux half is **upstream's code, unchanged**: `controller.py`, `policy.py`, `rl_core.py`, `recovery.py`, `proc.py`, `wire.py`, the four ONNX policies and the web pages. `fetch_upstream.sh` pins it at commit `9dc3678`. The STM32 half is ported to Python in `xgoduck_pi/mcu.py`, which speaks the same binary messages (`duck_state`, `duck_command`, ...), so upstream's code runs as it does on the UNO Q.

| File | Replaces |
| --- | --- |
| `xgoduck_pi/mcu.py` | `sketch/sketch.ino`: 100 Hz loop, arming, command TTL/freshness checks, calibration and servo-setup operations |
| `xgoduck_pi/scs_bus.py` | `sketch/scs_bus.h`: Feetech packets, sync read/write, reply parser |
| `xgoduck_pi/imu.py` | `sketch/qmi8658.h`: MPU6050 driver (QMI8658 too, for the Luwu board) with configurable mounting axes |
| `xgoduck_pi/web.py` | Arduino App Lab's WebUI brick: same `/api/...` routes, serves upstream `assets/` |
| `run.py` | upstream `main.py` |

**Status:** tested in simulation only (28 tests on macOS, including the real upstream controller and policies driving simulated servos). **It has not yet run on a Pi, the URT-2, real servos or the MPU6050.** Loop timing on the Pi Zero 2 W is unmeasured.

## Guided setup (start here)

```
cd ~/Research/xgo-duck/pi_runtime
../.venv/bin/python setup_wizard.py        # opens http://localhost:8790
```

The wizard runs on the Mac and goes through every step below. It gives each servo its ID through the URT-2 on USB, checks the assembled robot, and checks the Pi connection over SSH. It also copies and installs the software, and finds the IMU and its mounting direction. Finally it starts the robot software with live status and links to the calibration and control pages. Progress is saved in `wizard_state.json`. Both "Simulator" adapters and the "Try it" button work without hardware. It listens on localhost only.

## Wiring

```
URT-2  USB-C ──── micro-USB OTG cable ──── Pi Zero "USB" port (not "PWR")
URT-2  power terminal ── switch ── 2S battery (7.4–8.4 V)
URT-2  TTL bus ports ─── servo chains (IDs 10–14, 20–24, 30–34)

MPU6050  VCC → pin 1 (3.3 V)   GND → pin 6   SDA → pin 3   SCL → pin 5
         AD0 → pin 17 (3.3 V)  → address 0x69 (the PiSugar 3 clock uses 0x68)
PiSugar 3 powers the Pi only.
```

The Pi and the servos then share ground through the USB cable.

## Pi setup

1. Flash **Raspberry Pi OS Lite (64-bit)** with Raspberry Pi Imager. Set the hostname (e.g. `xgoduck`), your user, Wi-Fi and SSH in its settings.
2. From this folder on the Mac: `./deploy.sh <user>@xgoduck.local`
3. On the Pi: `cd xgo-duck && ./install_pi.sh` (add `--service` to start at boot), then `sudo reboot`.

## Before assembly: servo IDs

Every new servo is ID 1. Connect **one servo at a time** to the URT-2, power the URT-2 (6–8.4 V on its terminal), and give each servo its ID per page 1 of the assembly guide:

```
python tools/bus_check.py setup 1 10     # ID 1 -> 10, center at 2047, store KP 5 / KD 20
python tools/bus_check.py scan           # list everything on the bus
```

This works on the Mac too (URT-2 on USB-C; add `--port /dev/cu.usbmodem...` if it isn't found). Upstream's **Servo setup** web page also works: `python run.py --no-imu`, then open `http://localhost:9527/servo.html`.

## IMU orientation

Mount the MPU6050 flat on the robot's back, components facing out, silkscreen Y arrow up and X arrow to the robot's right. That matches the default `axes = "-z,-x,+y"`. To check, or for any other mounting:

```
python tools/imu_check.py          # robot upright, then tilted nose-down: prints the axes line for config.toml
python tools/imu_check.py --live   # upright should read grav ~ (0, 0, -1)
```

## Run

```
.venv/bin/python run.py            # then open http://xgoduck.local:9527
```

It boots in **inference only**, with torque off. From there, follow upstream's README:
- **Calibration** page: set each joint's mechanical zero (saved in `upstream/data/zero_pos.json`).
- **Default pose** with the robot supported.
- **Walk** on a clear floor.

Closing the page turns torque off after 1 s.

`/api/status` has an extra `pi` block with bus connection, adapter echo and IMU errors.

## Without hardware

```
../.venv/bin/python run.py --sim   # full UI on http://localhost:9527 with simulated servos + IMU
../.venv/bin/python -m pytest tests/
```

The simulated robot doesn't fall or walk, but the servos follow the commanded targets.

## Differences from the UNO Q firmware

- The reply parser treats a run of `FF` bytes as one header. The firmware loses the next reply in that case.
- Sync reads stop waiting as soon as all 15 servos have answered, instead of always waiting 6 ms.
- Adapter echo is detected at start-up and stripped. Without that, an echoed ping would look like a reply.
- If the USB adapter disconnects, the loop keeps reporting (with no servos) and reconnects every second. Torque goes off as on any feedback loss.
- Angle-to-encoder conversion uses double precision instead of the STM32's float; the result can differ by at most 1 encoder step (0.09°).
- On exit, torque off is sent three times.
