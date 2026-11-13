# Listen-only CAN ID discovery

This is a separate diagnostic application for the NUCLEO-F446RE and Waveshare
RS485 CAN Shield. It reads CAN H/L via CAN1 PB8/PB9 and prints to the ST-LINK
USB serial port through USART2 at **115200 baud, 8N1**. It does not initialize
PWM, run the motor command parser, assign motor slots, or send CAN messages.
CAN is in hardware silent mode: it sends neither commands nor ACK/error bits.
RS485 A/B discovery is not included.

## Flash and use

1. Disconnect motor power. Keep the Nucleo connected by its ST-LINK USB port.
2. In VS Code's Run and Debug selector, choose **CAN discovery (listen only,
   ST-LINK)** and start debugging to program the diagnostic ELF. Continue
   execution if the debugger stops at `main`. Do not select Motor controller
   or UART bridge for this procedure.
3. Open the Nucleo/ST-LINK virtual serial port in a serial monitor at 115200,
   8N1, with flow control off. Reset the Nucleo if you missed the startup text.
   Confirm it says `CAN DISCOVERY ONLY` before applying motor power.
4. With motor power still disconnected, connect only one secured AK60 to
   CAN H/L and the appropriate signal ground. Verify the shield CAN jumpers
   route TX to D14/PB9 and RX to D15/PB8, and verify CAN termination. Leave the
   other motors disconnected from this test bus. Do not change wiring live.
5. Apply the motor's specified power. Observe the serial output. The initial
   listening bitrate is 1 Mbps. Recognized servo status/start frames print a
   **candidate** motor ID; raw frames are also printed for other formats.
6. If no frames appear, send `5` to try 500 kbps, `2` for 250 kbps, or `8` for
   125 kbps. `1` returns to 1 Mbps; `?` prints help. These are single-character
   serial keys; newline is optional. Reselecting a rate clears its counters
   and permits identifiers to print again. If needed, power-cycle the motor
   after selecting a bitrate to capture startup frames.
7. Record the motor label, candidate ID, bitrate, and raw frame. Power the
   motor off, change to the next AK60, and repeat. Nothing is saved or assigned
   automatically. Duplicate IDs on different motors can only be distinguished
   by testing them individually.

Example (illustrative, not a measurement of your motor):

```text
Listening at 1000000 bit/s; silent mode (no ACK or commands).
CAN EXT DATA id=0x00002968 dlc=8 data=00 00 00 00 00 00 19 00
Candidate motor ID: 104 (servo status; verify with one motor connected).
```

The decoder recognizes extended data frames with function `0x29` and eight
bytes (servo status), or `0x2C` with four bytes `FA FB FC FD` (servo startup).
The low byte of the identifier supplies the candidate ID. Other versions,
MIT frames, remote requests, and unknown functions are shown as raw traffic
without asserting a motor ID. These definitions come from the
[CubeMars AK driver manual](https://www.cubemars.com/images/file/20250522/1747899365958473.pdf).
A CAN identifier alone cannot establish the motor model or its mechanical role.

## Limits and interpreting output

A five-second summary shows total frames received, identifiers printed,
current physical E-stop contact state, CAN last-error code (LEC), and FIFO
overrun status. The first 32 distinct identifiers/types per selected bitrate
are printed; repeated frames are counted without flooding serial output.
Busy buses can overflow the receive FIFO and lose frames. An overrun flag
is sticky until CAN is reinitialized; the LEC is the hardware register value,
not a diagnosis. This is intended for one connected motor at a time.

A silent listener does **not acknowledge** frames. On an otherwise isolated
bus the motor may retry frames, accumulate transmit errors, or stop sending.
It may also be configured for a different bitrate/mode or have periodic
feedback disabled. Consequently, `no feedback detected` is inconclusive,
and repeated frames do not imply multiple motors. This build does not fall
back to an active scan, enable command, or normal CAN mode.

**The E-stop is reported only in this application. It cannot stop a motor
or cut its power.** Motor behavior at power-up depends on its existing
configuration; secure the motor and keep a means of removing motor power.
This diagnostic has been compiled and its decoder tested on the host, but
has not been tested on the physical Nucleo/shield/motors.

## Build and tests

With STM32 ARM GCC and Ninja on PATH:

```sh
cmake --preset CanDiscovery
cmake --build --preset CanDiscovery
```

Artifacts:

- `build/CanDiscovery/CubeMars_Position_Control.elf` for ST-LINK/debugging.
- `build/CanDiscovery/CanDiscovery.bin` for a programmer (flash at `0x08000000`).

Decoder test:

```sh
cc -std=c11 -Wall -Wextra -Werror -ICore/Inc tests/can_discovery_test.c Core/Src/can_discovery_decode.c -o /tmp/cubemars-discovery-test
/tmp/cubemars-discovery-test
```

To return to motor-control firmware later, explicitly program the Debug
motor-controller ELF. Power-cycle alone keeps the diagnostic application.
