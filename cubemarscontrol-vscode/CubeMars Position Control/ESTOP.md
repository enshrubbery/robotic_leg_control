# Software and onboard stop

This motor-controller build does not require an external E-stop switch.
PA1/A1 is an unused input; no jumper is required. Its EXTI interrupt is
not enabled and its level does not affect reset or CAN transmission.
Servo slot 8 remains disabled.

The controller boots stopped. Send `0Z` plus newline to reset the stop;
no contact or debounce delay is required. Reset clears motion and leaves
all motors disabled. Send a fresh E command for each active motor.

UART `0X` and the onboard B1 button still latch stop, abort queued CAN
messages, disable PWM, and send supported zero-gain/zero-feedforward
motor commands. A new stop cancels any queued reset. M2 remains excluded
from all motor commands and CAN transmissions.

These are software stops, not power disconnects. A mechanical travel
stop does not cut torque. CAN stop delivery depends on wiring, motor
protocol and a working controller. This build has not been hardware tested.

See SIX_MOTOR_SETUP.md for fixed CAN mapping and the SixMotor build.
