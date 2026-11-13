# Six-motor CAN leg controller

Fixed UART slots (no renumbering when M2 is disabled):

| UART | Joint | Model | CAN ID | Available |
|---|---|---|---|---|
| 1 | Hip Pitch | AK70-10 | 10 | Yes |
| 2 | Hip Roll | AK80-64 | 99 | No |
| 3 | Hip Yaw | AK70-10 | 12 | Yes |
| 4 | Knee | AK60-6 | 106 | Yes |
| 5 | Foot Diff 1 | AK60-6 | 107 | Yes |
| 6 | Foot Diff 2 | AK60-6 | 108 | Yes |

All use CAN1 PB8/PB9. USART2 is 115200 baud, 8N1. Existing CAN bitrate,
packet encodings and gains are preserved. No external E-stop is required. The AK70-10s use
the existing standard-ID MIT path; AK60-6s use the existing extended-ID
AK60 MIT path. Actual motor mode/firmware compatibility has not been
verified on hardware. Joint travel limits, direction signs, and foot
differential kinematics are not calibrated by this mapping update.
Foot differential motors remain independently commanded in motor radians.

M2 rejects every UART motor command and is excluded from all motor CAN
send helpers, including startup and E-stop zero-torque traffic. Commands
`I` (routing ID) and `A` (model) are ignored for every slot so older host
software cannot overwrite the fixed mapping. The IDs must already be
configured on the physical motors; this firmware does not program IDs.

Boot remains stop latched. `0Z` followed by newline resets the latch;
PA1/A1 wiring and a contact debounce delay are not required. Each active motor
requires its own fresh `E` command (for example `1E`). `0X` latches stop.
Position commands use radians, for example `4P0.1`. Existing `V`, `K`, `D`,
`T`, `G`, `H`, `J`, `R`, and `O` semantics remain. PWM slots are 7 and 9;
slot 8 remains disabled. UART `0X` and onboard B1 still latch stop.
See ESTOP.md for the current stop behavior.

Build with the STM32 toolchain, CMake and Ninja on PATH:

```
cmake --preset SixMotor
cmake --build --preset SixMotor
```

Use **Motor controller (ST-LINK)** in VS Code. It now selects
`build/SixMotor/CubeMars_Position_Control.elf`. Discovery and bridge launch
configurations remain separate. Old Debug artifacts are not the updated image.

Regression sources: tests/six_motor_config_test.c and
 tests/estop_commands_test.c, each linked with Core/Src/uart_cmd.c and
compiled with -ICore/Inc. Firmware built successfully with STM32 GCC 14.3.1.
Regression sources were ARM compile-checked; execution requires a host C
compiler, unavailable in the current environment. No hardware was flashed.
