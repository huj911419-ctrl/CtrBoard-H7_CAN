#include "dbus.h"
#include "usart.h"
#include "main.h"
#include <string.h>

static uint8_t rx_byte;
static uint8_t rx_frame[DBUS_FRAME_LENGTH];
static uint8_t rx_pos;
static uint32_t last_byte_ms;
static volatile dbus_data_t state;

static uint16_t unpack11(const uint8_t *p, uint8_t shift)
{
    uint32_t word = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16);
    return (uint16_t)((word >> shift) & 0x07FFU);
}

static uint8_t channel_valid(uint16_t value)
{
    return value >= DBUS_CHANNEL_MIN && value <= DBUS_CHANNEL_MAX;
}

static void accept_frame(const uint8_t *frame, uint32_t now)
{
    uint16_t raw[4];
    raw[0] = unpack11(&frame[0], 0);
    raw[1] = unpack11(&frame[1], 3);
    raw[2] = unpack11(&frame[2], 6);
    raw[3] = unpack11(&frame[4], 1);

    if (!channel_valid(raw[0]) || !channel_valid(raw[1]) ||
        !channel_valid(raw[2]) || !channel_valid(raw[3]) ||
        ((frame[5] >> 6) & 0x03U) < 1U || ((frame[5] >> 6) & 0x03U) > 3U ||
        ((frame[5] >> 4) & 0x03U) < 1U || ((frame[5] >> 4) & 0x03U) > 3U)
        return;

    state.channel[0] = (int16_t)raw[0];
    state.channel[1] = (int16_t)raw[1];
    state.channel[2] = (int16_t)raw[2];
    state.channel[3] = (int16_t)raw[3];
    state.s1 = (uint8_t)((frame[5] >> 6) & 0x03U);
    state.s2 = (uint8_t)((frame[5] >> 4) & 0x03U);
    state.valid = 1U;
    state.last_frame_ms = now;
    if (state.frame_count != UINT32_MAX)
        ++state.frame_count;
}

void dbus_init(void)
{
    memset((void *)&state, 0, sizeof(state));
    rx_pos = 0U;
    last_byte_ms = HAL_GetTick();
    (void)HAL_UART_Receive_IT(&huart7, &rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart7)
        return;

    uint32_t now = HAL_GetTick();
    if (now - last_byte_ms > 2U)
        rx_pos = 0U;
    last_byte_ms = now;

    rx_frame[rx_pos++] = rx_byte;
    if (rx_pos >= DBUS_FRAME_LENGTH) {
        accept_frame(rx_frame, now);
        rx_pos = 0U;
    }
    (void)HAL_UART_Receive_IT(&huart7, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        rx_pos = 0U;
        (void)HAL_UART_Receive_IT(&huart7, &rx_byte, 1U);
    }
}

void dbus_get_data(dbus_data_t *out)
{
    if (out == NULL)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out, (const void *)&state, sizeof(*out));
    __set_PRIMASK(primask);
}

uint8_t dbus_is_online(uint32_t now)
{
    return state.valid && (now - state.last_frame_ms <= DBUS_TIMEOUT_MS);
}

uint8_t dbus_is_armed(const dbus_data_t *data)
{
    return data != NULL && data->valid && data->s1 == DBUS_ARM_S1_VALUE;
}

uint8_t dbus_friction_enabled(const dbus_data_t *data)
{
    return data != NULL && data->valid && data->s2 == DBUS_FRICTION_S2_VALUE;
}

float dbus_channel_normalized(const dbus_data_t *data, unsigned index)
{
    if (data == NULL || index >= 4U)
        return 0.0f;
    float value = ((float)data->channel[index] - (float)DBUS_CHANNEL_CENTER) /
                  (float)(DBUS_CHANNEL_MAX - DBUS_CHANNEL_CENTER);
    if (value > 1.0f)
        value = 1.0f;
    if (value < -1.0f)
        value = -1.0f;
    if (value > -0.04f && value < 0.04f)
        value = 0.0f;
    return value;
}
