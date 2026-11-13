#ifndef CAN_DISCOVERY_H
#define CAN_DISCOVERY_H
#include <stdint.h>

/* A candidate from documented servo status/start frames, never auto-assigned. */
uint8_t can_discovery_candidate(uint8_t extended, uint8_t remote,
                                uint32_t id, uint8_t dlc,
                                const uint8_t data[8], uint8_t *motor_id);
#endif
