#include "dji_motor.h"
#include "bsp_fdcan.h"
#include <string.h>

#if DJI_MOTOR_ID > 8
#error DJI_MOTOR_ID must be 0 (automatic) or 1..8
#endif

#define CONTROL_PERIOD_MS 10U
#define DISCOVERY_MS 1000U
#define MIN_FEEDBACK_FRAMES 10U
#define MAX_CONTROL_GAP_MS 50U
#define STALL_TIMEOUT_MS 2000U

typedef struct {
    int16_t rpm;
    uint8_t temperature;
    uint32_t count;
    uint32_t last_rx;
} feedback_t;

static volatile feedback_t feedback[8];
static volatile uint8_t seen_mask;
static dji_status_t status;
static uint32_t last_control, first_feedback, stall_ms;
static uint8_t discovery_started;
static float ramp_rpm, integral;

volatile uint8_t dji_run_enable = 0U;
volatile float dji_target_output_rpm = 0.0f;

static float clamp(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

static float magnitude(float x)
{
    return x < 0.0f ? -x : x;
}

void dji_motor_init(uint32_t now)
{
    for (unsigned i = 0; i < 8; ++i) {
        feedback[i].rpm = 0;
        feedback[i].temperature = 0;
        feedback[i].count = 0;
        feedback[i].last_rx = 0;
    }
    seen_mask = 0;
    memset(&status, 0, sizeof(status));
    last_control = now;
    first_feedback = now;
    discovery_started = 0;
    stall_ms = 0;
    ramp_rpm = integral = 0.0f;
}

/* Called only for CAN2. No UART or control calculations in the 1 kHz RX ISR. */
void dji_motor_on_feedback(uint16_t id, const uint8_t *data, uint8_t len, uint32_t now)
{
    if (id < 0x201U || id > 0x208U || len != 8U || data == NULL)
        return;
    if ((((uint32_t)data[0] << 8) | data[1]) > 8191U)
        return;
    unsigned slot = id - 0x201U;
    uint16_t raw = ((uint16_t)data[2] << 8) | data[3];
    feedback[slot].rpm = (int16_t)(raw < 0x8000U ? (int32_t)raw : (int32_t)raw - 65536);
    feedback[slot].temperature = data[6];
    feedback[slot].last_rx = now;
    if (feedback[slot].count != UINT32_MAX)
        ++feedback[slot].count;
    seen_mask |= (uint8_t)(1U << slot);
}

static void reset_control(void)
{
    status.current_command = 0;
    ramp_rpm = integral = 0.0f;
    stall_ms = 0;
}

/* Both groups are owned by this bench application; all other slots remain zero. */
static uint8_t send_current(void)
{
    uint8_t group1[8] = {0}, group2[8] = {0};
    if (status.motor_id != 0U) {
        uint8_t *group = status.motor_id <= 4U ? group1 : group2;
        unsigned slot = ((status.motor_id - 1U) % 4U) * 2U;
        uint16_t raw = (uint16_t)status.current_command;
        group[slot] = (uint8_t)(raw >> 8);
        group[slot + 1U] = (uint8_t)raw;
    }
    uint8_t failed = fdcanx_send_data(&hfdcan2, 0x200U, group1, 8U);
    failed |= fdcanx_send_data(&hfdcan2, 0x1FFU, group2, 8U);
    return failed;
}

void dji_motor_task(uint32_t now)
{
    uint32_t elapsed = now - last_control;
    if (elapsed < CONTROL_PERIOD_MS)
        return;
    last_control = now;

    /* Consistent ISR snapshot; preserve the caller's interrupt mask. */
    feedback_t samples[8];
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* An RX ISR may have run after the caller sampled 'now'. Use a timestamp
     * at least as recent as the snapshot, avoiding unsigned age underflow. */
    uint32_t snapshot_time = HAL_GetTick();
    status.seen_mask = seen_mask;
    for (unsigned i = 0; i < 8; ++i) {
        samples[i].rpm = feedback[i].rpm;
        samples[i].temperature = feedback[i].temperature;
        samples[i].count = feedback[i].count;
        samples[i].last_rx = feedback[i].last_rx;
    }
    __set_PRIMASK(primask);

    if (status.state < DJI_FAULT_MULTIPLE) {
#if DJI_MOTOR_ID == 0
        if ((status.seen_mask & (status.seen_mask - 1U)) != 0U)
            status.state = DJI_FAULT_MULTIPLE;
#endif
        if (!discovery_started && status.seen_mask != 0U) {
            discovery_started = 1;
            first_feedback = now;
        }
        if (status.state == DJI_WAITING && discovery_started &&
            now - first_feedback >= DISCOVERY_MS) {
            for (unsigned i = 0; i < 8; ++i) {
#if DJI_MOTOR_ID != 0
                if (i + 1U != DJI_MOTOR_ID)
                    continue;
#endif
                if (samples[i].count >= MIN_FEEDBACK_FRAMES &&
                    snapshot_time - samples[i].last_rx <= DJI_FEEDBACK_TIMEOUT_MS) {
                    status.motor_id = (uint8_t)(i + 1U);
                    status.state = DJI_STOPPED;
                    break;
                }
            }
        }
    }

    if (status.motor_id != 0U) {
        feedback_t *sample = &samples[status.motor_id - 1U];
        status.rotor_rpm = sample->rpm;
        status.temperature = sample->temperature;
        status.feedback_count = sample->count;
        if (status.state < DJI_FAULT_MULTIPLE) {
            if (snapshot_time - sample->last_rx > DJI_FEEDBACK_TIMEOUT_MS)
                status.state = DJI_FAULT_FEEDBACK;
            else if (sample->temperature >= DJI_TEMPERATURE_LIMIT)
                status.state = DJI_FAULT_TEMPERATURE;
            else if (status.state == DJI_RUNNING && elapsed > MAX_CONTROL_GAP_MS)
                status.state = DJI_FAULT_TIMING;
            else
                status.state = dji_run_enable ? DJI_RUNNING : DJI_STOPPED;
        }
    }

    if (status.state == DJI_RUNNING) {
        float target = dji_target_output_rpm;
        /* This comparison also rejects NaN and +/-infinity from debugger edits. */
        if (!(target >= -DJI_MAX_OUTPUT_RPM && target <= DJI_MAX_OUTPUT_RPM)) {
            status.state = DJI_FAULT_TARGET;
        } else if (magnitude((float)status.rotor_rpm / DJI_GEAR_RATIO) >
                   magnitude(target) + 60.0f) {
            status.state = DJI_FAULT_OVERSPEED;
        } else if (target == 0.0f) {
            /* Zero target means coast, not an active holding/braking command. */
            reset_control();
        } else {
            float dt = (elapsed <= MAX_CONTROL_GAP_MS ? elapsed : CONTROL_PERIOD_MS) * 0.001f;
            ramp_rpm += clamp(target - ramp_rpm, DJI_RAMP_RPM_PER_S * dt);
            float error = ramp_rpm * DJI_GEAR_RATIO - status.rotor_rpm;
            float candidate = clamp(integral + DJI_SPEED_KI * error * dt, DJI_CURRENT_LIMIT);
            float raw = DJI_SPEED_KP * error + candidate;
            /* Conditional integration prevents windup at either current limit. */
            if ((raw <= DJI_CURRENT_LIMIT && raw >= -DJI_CURRENT_LIMIT) ||
                (raw > DJI_CURRENT_LIMIT && error < 0.0f) ||
                (raw < -DJI_CURRENT_LIMIT && error > 0.0f))
                integral = candidate;
            status.current_command = (int16_t)clamp(DJI_SPEED_KP * error + integral,
                                                   DJI_CURRENT_LIMIT);
            if (magnitude(ramp_rpm) >= 10.0f && magnitude((float)status.rotor_rpm) < 50.0f &&
                magnitude((float)status.current_command) >= 0.9f * DJI_CURRENT_LIMIT)
                stall_ms += elapsed;
            else
                stall_ms = 0;
            if (stall_ms >= STALL_TIMEOUT_MS)
                status.state = DJI_FAULT_STALL;
        }
    }
    if (status.state != DJI_RUNNING)
        reset_control();

    /* With no C620 feedback, stay receive-only and avoid filling an unacked TX FIFO. */
    if (status.seen_mask != 0U && send_current()) {
        if (status.state < DJI_FAULT_MULTIPLE)
            status.state = DJI_FAULT_TX;
        reset_control();
        /* Cancel queued stale current commands; next service attempts zero again. */
        HAL_FDCAN_AbortTxRequest(&hfdcan2, 0xFFFFFFFFU);
        (void)send_current();
    }
}

const dji_status_t *dji_motor_status(void)
{
    return &status;
}

const char *dji_motor_state_name(dji_state_t state)
{
    static const char *const names[] = {
        "WAIT", "RUN", "STOP", "MULTI-ID", "RX-TIMEOUT", "HOT",
        "LOOP-LATE", "TX-FAIL", "STALL", "OVERSPEED", "BAD-TARGET"
    };
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}
