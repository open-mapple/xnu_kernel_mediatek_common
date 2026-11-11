/* Shared C/C++ interface. Init once, on the MapplePowerSource start thread. */
#ifndef MAPPLE_BATTERY_SPMI_H
#define MAPPLE_BATTERY_SPMI_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int mapple_spmi_init(void);
/* Thread context only; 0 success, -1 failure; *out unchanged on failure. */
int mapple_spmi_read(unsigned sid, unsigned addr, uint8_t *out);
#ifdef __cplusplus
}
#endif
#endif
