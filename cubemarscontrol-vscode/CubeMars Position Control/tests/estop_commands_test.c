#include "uart_cmd.h"
#include <assert.h>
#include <stdio.h>
static void command(const char *s) {
    while (*s) uart_cmd_feed((uint8_t)*s++);
    uart_cmd_feed('\n');
}
int main(void) {
    uart_cmd_init();
    assert(uart_cmd_e_stop_requested());
    command("1E"); assert(!uart_cmd_motor(0)->enable_pending);
    uart_cmd_estop_sample(0); // unwired PA1 must not prevent reset
    command("0Z"); assert(uart_cmd_e_stop_requested());
    assert(uart_cmd_service_reset());
    assert(!uart_cmd_e_stop_requested());
    uart_cmd_estop_sample(0); assert(!uart_cmd_e_stop_requested());
    command("1E"); command("1P1.5");
    assert(uart_cmd_motor(0)->enable_pending && uart_cmd_motor(0)->pid_active);
    command("0X"); assert(uart_cmd_e_stop_requested());
    command("3E"); assert(!uart_cmd_motor(2)->enable_pending);
    command("0Z"); assert(uart_cmd_service_reset());
    assert(!uart_cmd_motor(0)->enabled && !uart_cmd_motor(0)->enable_pending && !uart_cmd_motor(0)->pid_active);
    uart_cmd_latch_e_stop(); // same latch called by onboard B1
    assert(uart_cmd_e_stop_requested());
    command("0Z"); uart_cmd_latch_e_stop();
    assert(!uart_cmd_service_reset()); // new stop cancels queued reset
    command("0z"); assert(uart_cmd_service_reset());
    puts("Software/B1 stop and no-contact reset tests passed");
}
