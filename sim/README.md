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

## Check against the bench

| Firmware channel | Simulated actuator on selected leg |
| --- | --- |
| M1 | Hip pitch |
| M2 | Hip roll |
| M3 | Hip yaw |
| M4 | Knee |
| M5 | Outer ankle crank |
| M6 | Inner ankle crank |

The channel-to-crank assignment and positive angle directions are simulation assumptions. Confirm the two crank assignments, encoder signs and zeros, and the actual hip pitch actuator before transferring gains. The CAD package uses a placeholder hip pitch motor, estimated inertias and travel limits, and visual meshes with contact disabled by default. This setup is for supported-leg tracking experiments. Standing and walking require a floating-base contact model and a balance controller.
