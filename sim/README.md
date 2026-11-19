# Fixed-base robot leg simulation

The CAD MJCF and its referenced meshes are in `model/`. The runner converts the selected leg's six position actuators to torque motors in memory, preserves all four ankle rod constraints, and holds the other leg at its CAD targets with the original position servos.

From the repository root on Windows:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r sim/requirements.txt
.\.venv\Scripts\python.exe sim/run.py --motion joints --csv joints.csv
.\.venv\Scripts\python.exe sim/run.py --motion ankle-pitch --csv pitch.csv --viewer
.\.venv\Scripts\python.exe sim/run.py --leg right --motion ankle-roll --csv roll.csv
```

The default experiment holds the CAD reference for one second, then ramps toward a target change of 0.12 rad. The `joints` motion exercises the hip axes and knee together. `ankle-pitch` moves the two ankle cranks together, and `ankle-roll` moves them in opposite directions. The CSV records targets, ramped commands, measured joint angles and velocities, torque commands, the outer PID torque contribution, passive ankle angles, and the largest absolute rod closure residual component at each sample.

## Compare controller modes

```powershell
.\.venv\Scripts\python.exe sim/run.py --motion ankle-roll --mode mit --csv roll-mit.csv
.\.venv\Scripts\python.exe sim/run.py --motion ankle-roll --mode firmware --csv roll-firmware.csv
.\.venv\Scripts\python.exe sim/run.py --motion ankle-roll --mode anti-windup --csv roll-anti-windup.csv
```

`mit` uses the simulated CubeMars position and velocity feedback terms alone. `firmware` adds the STM32 outer PID with its default gains, integral clamp, and 1 rad error cutoff. `anti-windup` also pauses integration when the error would push the outer torque or combined motor torque farther into saturation. The default derivative gains of the outer PID are zero, matching the firmware initialization. The runner applies a fixed joint torque limit using the lower of the CAD force limit and firmware torque field range. Review the gain arrays in `run.py` before comparing alternative gains.

The physics step is 1 ms and the simulated controller period defaults to 5 ms. Use `--control-ms` to replay a measured period. The model uses ideal motor torque with sampled control. CAN timing and quantization, firmware enable and watchdog logic, the motor's internal update rate, and the torque-speed envelope are outside the model.

## Tune individual motors

```bat
.\.venv\Scripts\python.exe sim\run.py --motion m1 --kp 5 --ki 0 --kd 0 --seconds 8 --csv m1-baseline.csv
.\.venv\Scripts\python.exe sim\run.py --motion m4 --kp 25 --ki 0.3 --kd 0 --seconds 8 --csv m4-baseline.csv
```

`m1` moves hip pitch, `m2` moves hip roll, `m3` moves hip yaw, `m4` moves the knee, and `m5` and `m6` move the outer and inner ankle cranks respectively. The other joints keep their starting targets. The optional gain flags apply to the selected motor's outer PID for that run. With `--motion ankle-pitch` or `--motion ankle-roll`, the gain flags apply to both cranks. Omitted gains use the firmware defaults. Add `--viewer` to watch or `--amplitude -0.12` to test the opposite direction. Keep the amplitude, controller period, and duration the same when comparing gains.

### M1 sweep from +π to −π

```bat
.\.venv\Scripts\python.exe sim\run.py --motion m1 --full-rom --mode anti-windup --kp 20 --ki 32 --kd 0 --csv m1-full-rom.csv --viewer
```

The 44-second sweep holds the starting pose for one second, ramps to +π, and switches the target to −π at 17 seconds. The position ramp remains limited to 0.3 rad/s. This option temporarily widens only the selected hip-pitch joint from its CAD limits of ±1.57 rad to ±π. The model has no self-collision geometry to establish physical clearance across this range.

To test M2 holding zero roll during the sweep, add `--m2-hold KP KI KD` with the desired outer gains.
Add `--positive-only` to ramp from neutral to +π, hold there, and end after 17 seconds.

### M2 sweep from zero to +π and back

```bat
.\.venv\Scripts\python.exe sim\run.py --motion m2 --full-rom --mode anti-windup --kp 75 --ki 80 --kd 0 --csv m2-positive-pi-return.csv --viewer
```

The 33-second sweep switches M2's target from +π back to zero at 17 seconds. M1 holds its starting target with its tuned PID when the outer controller is active. This simulation temporarily widens the selected hip-roll joint's CAD limits from ±0.6 rad to ±π. Physical hip-roll travel and clearance over that range are unverified.

## Check against the bench

| Firmware channel | Simulated actuator on selected leg |
| --- | --- |
| M1 | Hip pitch |
| M2 | Hip roll |
| M3 | Hip yaw |
| M4 | Knee |
| M5 | Outer ankle crank |
| M6 | Inner ankle crank |

The channel-to-crank assignment and positive angle directions are simulation assumptions. Confirm the two crank assignments and encoder signs and zeros before transferring gains. The simulation adds a 0.621 kg AK70-10 at each hip-pitch pivot. It assumes both housings mount to the base and uses a cylindrical inertia estimate. Joint armature still uses the generic CAD-export value. The model uses estimated travel limits and visual meshes with contact disabled by default. This setup is for supported-leg tracking experiments. Standing and walking require a floating-base contact model and a balance controller.
