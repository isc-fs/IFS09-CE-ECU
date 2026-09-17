#ifndef ECU_TELEMETRY_H
#define ECU_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Telemetry_Send32(const uint8_t payload[32]);

#ifdef __cplusplus
}
#endif

#endif /* ECU_TELEMETRY_H */
