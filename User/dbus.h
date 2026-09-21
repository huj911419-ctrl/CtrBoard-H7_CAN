#ifndef DBUS_H
#define DBUS_H

#include <stdint.h>

/* DT7/DR16 protocol: 100 kbaud, 8 data bits, even parity, 1 stop bit.
 * The signal is inverted; this project uses UART7 RX hardware inversion on
 * PE7, so no external inverter is required. */
#define DBUS_FRAME_LENGTH       18U
#define DBUS_TIMEOUT_MS         100U
#define DBUS_CHANNEL_CENTER     1024
#define DBUS_CHANNEL_MIN        364
#define DBUS_CHANNEL_MAX        1684

/* Change these two values after looking at the UART log if your switch
 * direction is different. DT7/DR16 reports switch positions as 1..3. */
#define DBUS_ARM_S1_VALUE       1U
#define DBUS_FRICTION_S2_VALUE  1U

typedef struct {
    int16_t channel[4];
    uint8_t s1;
    uint8_t s2;
    uint8_t valid;
    uint32_t last_frame_ms;
    uint32_t frame_count;
} dbus_data_t;

void dbus_init(void);
void dbus_get_data(dbus_data_t *out);
uint8_t dbus_is_online(uint32_t now);
uint8_t dbus_is_armed(const dbus_data_t *data);
uint8_t dbus_friction_enabled(const dbus_data_t *data);
float dbus_channel_normalized(const dbus_data_t *data, unsigned index);

#endif
