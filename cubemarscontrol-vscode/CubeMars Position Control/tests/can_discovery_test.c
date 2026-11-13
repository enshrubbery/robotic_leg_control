#include "can_discovery.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    uint8_t data[8] = {0};
    uint8_t id = 77;
    assert(can_discovery_candidate(1, 0, 0x2968, 8, data, &id) && id == 104);
    assert(can_discovery_candidate(1, 0, 0x2900, 8, data, &id) && id == 0);
    assert(can_discovery_candidate(1, 0, 0x29ff, 8, data, &id) && id == 255);
    assert(!can_discovery_candidate(0, 0, 0x2968, 8, data, &id));
    assert(!can_discovery_candidate(1, 1, 0x2968, 8, data, &id));
    assert(!can_discovery_candidate(1, 0, 0x2968, 7, data, &id));
    assert(!can_discovery_candidate(1, 0, 0x2968, 9, data, &id));
    assert(!can_discovery_candidate(1, 0, 0x12968, 8, data, &id));
    assert(!can_discovery_candidate(1, 0, 0x968, 8, data, &id));
    assert(!can_discovery_candidate(1, 0, 0x2c68, 4, data, &id));
    data[0] = 0xfa; data[1] = 0xfb; data[2] = 0xfc; data[3] = 0xfd;
    assert(can_discovery_candidate(1, 0, 0x2c68, 4, data, &id) && id == 104);
    assert(!can_discovery_candidate(1, 0, 0x2c68, 3, data, &id));
    data[3] = 0;
    assert(!can_discovery_candidate(1, 0, 0x2c68, 4, data, &id));
    puts("CAN discovery decoding tests passed");
}
