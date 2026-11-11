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

**Status:** 48 tests on macOS (`python -m pytest tests/`), including the real upstream controller and policies driving simulated servos. **It has run on the Pi Zero 2 W** with the URT-2 and all 15 servos: 99.8 Hz servo feedback, 50 Hz policy, about 0 tick overruns. Not yet verified on the real robot: the battery/temperature reads (register addresses are taken from the Feetech SDK, see below), IMU brownout detection after a real power dip, and walking on battery.

## Guided setup (start here)

```
cd ~/Research/xgo-duck/pi_runtime
../.venv/bin/python setup_wizard.py        # opens http://localhost:8790
```

The wizard runs on the Mac and goes through every step below. It gives each servo its ID through the URT-2 on USB, checks the assembled robot, and checks the Pi connection over SSH. It also copies and installs the software, and finds the IMU and its mounting direction. Finally it starts the robot software with live status and links to the calibration and control pages. Progress is saved in `wizard_state.json`. Both "Simulator" adapters and the "Try it" button work without hardware. It listens on localhost only.

## Wiring

```
URT-2  USB-C ──── micro-USB OTG cable ──── Pi Zero "USB" port (not "PWR")
2S battery (7.4–8.4 V) ── XH plug ── URT-2 blue screw terminal V1/G
         (the XH plug is the on/off switch; there is no switch and no battery monitor)
UBEC 5 V ── from the same URT-2 terminal ── Pi pins 4 (5 V) and 14 (GND)
URT-2  TTL port 1 ─── legs
URT-2  TTL port 2 ─── neck chain (cut Waveshare 5264 cable, solder-seal joints)

MPU6050  VCC → pin 1 (3.3 V)   GND → pin 6   SDA → pin 3   SCL → pin 5
         AD0 → pin 17 (3.3 V)  → address 0x69 (kept from the PiSugar days; no PiSugar any more)
```

The PiSugar was removed: the Pi is powered by the UBEC. The Pi and the servos share ground through the USB cable and the UBEC.

Because the battery goes straight into the URT-2, the software watches the pack voltage (see Battery and temperature).

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

`/api/status` has an extra `pi` block with bus connection, adapter echo and IMU errors, plus: `bus_voltage` (V, or null), `servo_temps` (per ID, deg C), `servo_temp_max`, `low_battery`, `battery_warn`, `battery_warn_v`, `battery_cutoff_v`, `arm_refused_battery`.

## Battery and temperature

About once a second the control loop sweeps the 15 servos with one 2-byte read each (one small packet per 10 ms tick, only after all feedback replies are in). Register 0x3E is the present voltage in 0.1 V units and 0x3F the present temperature in deg C; both addresses come from the Feetech SDK (`SMS_STS_PRESENT_VOLTAGE = 62`, `SMS_STS_PRESENT_TEMPERATURE = 63`, identical in the SCSCL table) and sit right behind the 6-byte position/speed/load block the loop already reads. The HD-1910 is untested here. The upstream sketch never reads them. `bus_voltage` is the median of the last 5 s of servo readings.

```
[battery]
warn_v = 7.0     # status flag battery_warn
cutoff_v = 6.6   # 2S; below this for ~3 s: low_battery, new ARM commands refused
```

A walking robot is **not** switched limp when the battery drops (it would fall); it is only flagged, and the refusal applies to the next arm. Upstream sends ARM once per mode change, so after a refusal select shadow and then hold/policy again once the pack is charged. Length-4 replies (this read and the boot KP/KD read) are told apart by the pending request, not by length.

## IMU health and gyro bias

A power dip makes the MPU6050 reset into sleep and return zeros without an I2C error. About once a second the driver reads PWR_MGMT_1; if the sleep bit is set, or the accelerometer magnitude is under 0.3 g for 20 samples in a row, the IMU is marked lost (`imu.ok` false, so the robot disarms) and re-initialised by the existing 1 s re-probe.

Gyro zero-rate offset: with the robot still, `python tools/imu_check.py --bias` averages 5 s of raw gyro and prints `gyro_bias = [...]` (sensor frame, deg/s); `--write` puts it into config.toml. The driver subtracts it before the axis mapping. MPU6050 only.

## Network exposure

The web UI has no login and listens on every interface by default (`http_host = "0.0.0.0"` in config.toml), so anyone on the LAN can arm the robot. To restrict it set `http_host` to the Pi's Tailscale address (or `"127.0.0.1"` and use `ssh -L 9527:localhost:9527`), then restart the service.

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
- Added: servo voltage/temperature monitoring with ARM refusal on low battery, IMU brownout detection, gyro bias, and `http_host`.
- A `CAL_SET_ZEROS` packet is applied even while the servo adapter is closed (the firmware has no such state), so the controller's one-time zero push cannot be lost.
