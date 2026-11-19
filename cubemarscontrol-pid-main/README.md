# Remote Control Code

## Adjustable control limits

Build and flash the updated STM32 firmware before using the new limits controls
in `motor_gui_qt.py`. After connecting, choose **Limits…** on a motor to set
its minimum and maximum position target (radians relative to the firmware zero),
torque feedforward cap (N·m), and integral torque cap (N·m). The global motion
row sets the position ramp's speed and acceleration. Apply a motor's position
bounds while it is passive if you want to avoid changing the target of an
enabled motor.

| UART command | Setting |
| --- | --- |
| `nB<min> <max> <feedforward> <integral>` | Set all four motor limits together, with `n` from 1 to 6. |
| `0L<speed>` | Set the global position ramp speed in rad/s. |
| `0U<acceleration>` | Set the global position ramp acceleration in rad/s². |

Terminate each command with a newline. For example, `4B-1.000 1.000 4.000 1.500`
bounds M4's target to ±1 rad, feedforward to ±4 N·m, and the integral
contribution to ±1.5 N·m. Setting the integral cap to zero suppresses integral
torque. The firmware also caps the integral contribution at the active
feedforward limit.

The feedforward cap covers the outer PID and the requested torque offset
together. The drive computes additional torque from its embedded MIT spring
and damper, so the feedforward setting does not bound total drive torque.
Position bounds apply to commanded targets. Physical motion can exceed those
bounds due to tracking error or external load.

Settings are held in STM32 RAM and return to firmware defaults after reset.
The GUI remembers values it sent during the current connection. Reapply your
settings after resetting the controller. The `--pid` flag only sends the
tuned PID gains.
