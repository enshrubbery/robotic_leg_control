#include "can_discovery.h"

uint8_t can_discovery_candidate(uint8_t extended, uint8_t remote,
                                uint32_t id, uint8_t dlc,
                                const uint8_t data[8], uint8_t *motor_id)
{
    if (!extended || remote || id > 0x1fffffffU || dlc > 8) return 0;
    uint32_t function = id >> 8;
    if ((function == 0x29 && dlc == 8) ||
        (function == 0x2c && dlc == 4 && data[0] == 0xfa &&
         data[1] == 0xfb && data[2] == 0xfc && data[3] == 0xfd)) {
        *motor_id = (uint8_t)id;
        return 1;
    }
    return 0;
}
