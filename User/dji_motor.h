#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>

/* CAN2 is dedicated to C620 + M3508. 0 = discover ONE motor; 1..8 = fixed ID. */
#ifndef DJI_MOTOR_ID
#define DJI_MOTOR_ID             0U
#endif
#define DJI_DEFAULT_OUTPUT_RPM   30.0f
#define DJI_GEAR_RATIO           (3591.0f / 187.0f)
#define DJI_CURRENT_LIMIT        2000   /* CAN units: 16384 = 20 A; here ~2.44 A */
#define DJI_SPEED_KP             4.0f   /* CAN current units / rotor rpm */
#define DJI_SPEED_KI             8.0f   /* CAN current units / (rotor rpm * s) */
#define DJI_RAMP_RPM_PER_S       30.0f   /* output shaft */
#define DJI_MAX_OUTPUT_RPM       120.0f
#define DJI_FEEDBACK_TIMEOUT_MS  100U
#define DJI_TEMPERATURE_LIMIT    70U    /* conservative bench-test threshold */

typedef enum {
    DJI_WAITING = 0,
    DJI_RUNNING,
    DJI_STOPPED,
    DJI_FAULT_MULTIPLE,
    DJI_FAULT_FEEDBACK,
    DJI_FAULT_TEMPERATURE,
    DJI_FAULT_TIMING,
    DJI_FAULT_TX,
    DJI_FAULT_STALL,
    DJI_FAULT_OVERSPEED,
    DJI_FAULT_TARGET
} dji_state_t;

typedef struct {
    dji_state_t state;
    uint8_t motor_id;
    uint8_t seen_mask;
    uint8_t temperature;
    uint16_t rotor_angle;
    int16_t rotor_rpm;
    int16_t torque_current;
    int16_t current_command;
    uint32_t feedback_age_ms;
    uint32_t feedback_count;
} dji_status_t;

/* Debugger controls: enable=0 sends zero current (coasts, does not hold position).
 * Faults latch until MCU reset. Target unit is OUTPUT SHAFT rpm, signed. */
extern volatile uint8_t dji_run_enable;
extern volatile float dji_target_output_rpm;

void dji_motor_init(uint32_t now); /* before starting CAN interrupts */
void dji_motor_on_feedback(uint16_t id, const uint8_t *data, uint8_t len, uint32_t now);
void dji_motor_task(uint32_t now); /* main loop, service at least every 10 ms */
const dji_status_t *dji_motor_status(void); /* main context only */
const char *dji_motor_state_name(dji_state_t state);

#endif
