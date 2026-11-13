#include "uart_cmd.h"
#include <assert.h>
#include <stdio.h>
static void command(const char *s) {
    while (*s) uart_cmd_feed((uint8_t)*s++);
    uart_cmd_feed('\n');
}
int main(void) {
    const unsigned ids[] = {10,99,12,106,107,108};
    uart_cmd_init();
    assert(NUM_MOTORS == 6);
    for (unsigned i=0;i<6;i++) {
        MotorState *m=uart_cmd_motor(i);
        assert(m->can_id==ids[i] && m->id_configured);
        assert(m->bus==MOTOR_BUS_CAN);
        assert(m->type==(i==1?MOTOR_TYPE_AK80_64:i<3?MOTOR_TYPE_AK70:MOTOR_TYPE_AK60));
        assert(uart_cmd_motor_can_ready(m)==(i!=1));
        assert(!m->enabled && !m->enable_pending);
    }
    for(unsigned i=0;i<100;i++) uart_cmd_estop_sample(1);
    command("0Z"); assert(uart_cmd_service_reset());
    const char *blocked[]={"2E","2R","2O","2P1","2V1","2T1","2K99","2D99","2G99","2H99","2J99","2I10","2A60","2e","2r","2o"};
    MotorState before=*uart_cmd_motor(1);
    for(unsigned i=0;i<sizeof(blocked)/sizeof(blocked[0]);i++) command(blocked[i]);
    MotorState *m=uart_cmd_motor(1);
    assert(m->can_id==99 && m->type==MOTOR_TYPE_AK80_64);
    assert(!m->enable_pending && !m->enabled && !m->fb_only && !m->set_origin_pending && !m->pid_active);
    assert(m->pos==0 && m->vel==0 && m->tff==0 && m->kp==before.kp && m->kd==before.kd);
    assert(m->pid_kp==before.pid_kp && m->pid_kd==before.pid_kd && m->pid_ki==before.pid_ki);
    command("1I99");command("1A60");
    assert(uart_cmd_motor(0)->can_id==10 && uart_cmd_motor(0)->type==MOTOR_TYPE_AK70);
    command("1E");command("3E");command("4E");command("5E");command("6E");
    for(unsigned i=0;i<6;i++) assert(uart_cmd_motor(i)->enable_pending==(i!=1));
    command("4P0.25");command("5P0.5");command("6P0.75");
    assert(uart_cmd_motor(3)->pos==0.25f && uart_cmd_motor(4)->pos==0.5f && uart_cmd_motor(5)->pos==0.75f);
    command("7P30");command("8P45");command("9P60");
    assert(uart_cmd_servo_pos()[0]==30 && uart_cmd_servo_pos()[1]==0 && uart_cmd_servo_pos()[2]==60);
    command("0X");command("6P1");assert(uart_cmd_motor(5)->pos==0.75f);
    command("0Z");assert(uart_cmd_service_reset());
    for(unsigned i=0;i<6;i++) {
        m=uart_cmd_motor(i);
        assert(!m->enabled && !m->enable_pending && !m->pid_active && !m->fb_only && !m->set_origin_pending);
        assert(m->can_id==ids[i]);
    }
    assert(!uart_cmd_motor_can_ready(uart_cmd_motor(1)));
    puts("Six-motor configuration tests passed");
}
