#include "bsp_fdcan.h"
#include "dji_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
static uint32_t tick, irq_mask, tx_count, abort_count;
static unsigned fail_tx, started_mask, notify_mask, log_count;
static uint8_t group_data[2][8];
static FDCAN_FilterTypeDef filters[3];
static struct {
    unsigned port, count;
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
} rx;

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
uint32_t HAL_GetTick(void) { return tick; }
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { started_mask |= 1U << h->Instance; return HAL_OK; }
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t flags, uint32_t buffers)
{ assert(flags == 1 && buffers == 0); notify_mask |= 1U << h->Instance; return HAL_OK; }
int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, FDCAN_FilterTypeDef *f)
{ filters[h->Instance - 1] = *f; return HAL_OK; }
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ assert(h->Instance < 3 && a == 2 && b == 2 && c == 1 && d == 1); return HAL_OK; }
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *head, uint8_t *data)
{
    assert(h->Instance != 3);
    assert(head->IdType == FDCAN_STANDARD_ID && head->FDFormat == FDCAN_CLASSIC_CAN);
    assert(head->DataLength <= 8);
    if (fail_tx) { --fail_tx; return 1; }
    ++tx_count;
    if (h == &hfdcan2) {
        assert(head->DataLength == 8 && (head->Identifier == 0x200 || head->Identifier == 0x1FF));
        memcpy(group_data[head->Identifier == 0x1FF], data, 8);
    }
    return HAL_OK;
}
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo, FDCAN_RxHeaderTypeDef *head, uint8_t *data)
{
    assert(fifo == 0 && rx.count && h->Instance == rx.port);
    *head = rx.header;
    memcpy(data, rx.data, head->DataLength <= 8 ? head->DataLength : 64);
    --rx.count;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{ assert(fifo == 0); return h->Instance == rx.port ? rx.count : 0; }
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h, uint32_t mask)
{ assert(h == &hfdcan2 && mask == UINT32_MAX); ++abort_count; return HAL_OK; }
void Error_Handler(void) { abort(); }
void log_print(const char *fmt, ...) { (void)fmt; ++log_count; }
void can_log(const char *dir, const char *port, uint16_t id, const uint8_t *data, uint8_t len)
{ (void)dir; (void)port; (void)id; (void)data; (void)len; ++log_count; }

static void inject(unsigned port, uint16_t id, const uint8_t *data, uint8_t len, unsigned copies)
{
    rx.port = port;
    rx.count = copies;
    rx.header = (FDCAN_RxHeaderTypeDef){id, 0, 0, len, 0};
    memcpy(rx.data, data, len);
    HAL_FDCAN_RxFifo0Callback(port == 1 ? &hfdcan1 : &hfdcan2, 1);
    assert(rx.count == 0);
}
static void sample(unsigned id, int rpm, unsigned temp)
{
    uint8_t data[8] = {0x10, 0, (uint8_t)((uint16_t)rpm >> 8), (uint8_t)rpm, 0, 0, (uint8_t)temp, 0};
    inject(2, (uint16_t)(0x200 + id), data, 8, 10); /* ten feedback frames per 10 ms */
}
static void reset_at(uint32_t now)
{
    tick = now;
    tx_count = abort_count = 0;
    fail_tx = 0;
    irq_mask = 0;
    memset(group_data, 0, sizeof(group_data));
    dji_run_enable = 1;
    dji_target_output_rpm = 30.0f;
    dji_motor_init(tick);
    can_tx_quiet = 1;
}
static void step(unsigned id, int rpm, unsigned temp)
{ tick += 10; sample(id, rpm, temp); dji_motor_task(tick); }
static void discover(unsigned id)
{
    for (unsigned n = 0; n < 101; ++n)
        step(id, 0, 25);
    assert(dji_motor_status()->state == DJI_RUNNING);
    assert(dji_motor_status()->motor_id == id);
}
static void assert_zero(void)
{
    assert(dji_motor_status()->current_command == 0);
    for (unsigned g = 0; g < 2; ++g)
        for (unsigned i = 0; i < 8; ++i)
            assert(group_data[g][i] == 0);
}
static void assert_payload(unsigned id)
{
    unsigned group = id > 4, slot = (id - 1) % 4 * 2;
    uint16_t command = (uint16_t)dji_motor_status()->current_command;
    for (unsigned g = 0; g < 2; ++g)
        for (unsigned i = 0; i < 8; ++i) {
            uint8_t expected = 0;
            if (g == group && i == slot) expected = (uint8_t)(command >> 8);
            if (g == group && i == slot + 1) expected = (uint8_t)command;
            assert(group_data[g][i] == expected);
        }
}

int main(void)
{
    bsp_can_init();
    assert(started_mask == 6 && notify_mask == 6); /* only CAN1 and CAN2 */
    assert(filters[0].FilterType == FDCAN_FILTER_MASK && filters[0].FilterID2 == 0);
    assert(filters[1].FilterType == FDCAN_FILTER_RANGE && filters[1].FilterID1 == 0x201 && filters[1].FilterID2 == 0x208);

    /* CAN2 traffic cannot complete a pending CAN1 DM transaction. */
    reset_at(0);
    uint8_t request[4] = {7, 0, 0x33, 0x3C};
    uint8_t response[8] = {7, 0, 0x33, 0x3C, 0, 0, 0xC0, 0x41};
    fdcanx_send_data(&hfdcan1, 0x7FF, request, 4);
    inject(2, 0x207, response, 8, 3);
    assert(dm_rx33_flag == 0 && dm_rx_count == 0 && dm_master_id == 0xFFFF);
    assert(log_count == 0); /* no UART from CAN2's ISR */
    inject(1, 0x10, response, 8, 1);
    assert(dm_rx33_flag && dm_reply_canid == 7 && dm_master_id == 0x10);
    assert(dm_rx_count == 1 && log_count == 0); /* CAN ISR must stay UART-free */
    dm_param_reply_t reply;
    assert(dm_take_param_reply(&reply));
    assert(reply.opcode == 0x33 && reply.rid == 0x3C && reply.can_id == 7 && reply.mst_id == 0x10);
    assert(reply.raw == 0x41C00000U);
    assert(!dm_take_param_reply(&reply));
    uint8_t dm_state[8] = {0x17, 0x80, 0, 0x80, 0, 0, 25, 26};
    inject(1, 0x10, dm_state, 8, 1);
    assert(dm_fb_count == 1 && dm_fb[7] == 26);
    assert(fdcanx_send_data(&hfdcan3, 1, dm_state, 8) == 1);
    assert(fdcanx_send_data(&hfdcan2, 1, dm_state, 12) == 1);

    reset_at(0);
    tick = 3000; dji_motor_task(tick);
    assert(dji_motor_status()->state == DJI_WAITING && tx_count == 0);
    uint8_t bad[8] = {0xFF, 0xFF};
    dji_motor_on_feedback(0x201, bad, 8, tick);
    dji_motor_on_feedback(0x201, bad, 7, tick);
    dji_motor_on_feedback(0x209, bad, 8, tick);
    tick += 10; dji_motor_task(tick);
    assert(tx_count == 0);

#if DJI_MOTOR_ID == 0
    for (unsigned id = 1; id <= 8; ++id) {
#else
    for (unsigned id = DJI_MOTOR_ID; id <= DJI_MOTOR_ID; ++id) {
#endif
        reset_at(0); discover(id);
        assert(dji_motor_status()->current_command > 0);
        assert(dji_motor_status()->current_command < 100); /* ramp starts gently */
        assert(dji_motor_status()->rotor_angle == 4096 && dji_motor_status()->torque_current == 0);
        assert_payload(id);
        for (unsigned n = 0; n < 100; ++n) {
            step(id, 0, 25);
            assert(abs(dji_motor_status()->current_command) <= DJI_CURRENT_LIMIT);
        }
        assert_payload(id);
        dji_run_enable = 0; step(id, 0, 25); assert_zero();
        assert(dji_motor_status()->state == DJI_STOPPED);
        dji_target_output_rpm = -30; dji_run_enable = 1;
        step(id, -10, 25);
        assert(dji_motor_status()->rotor_rpm == -10); /* signed big-endian feedback */
        step(id, 0, 25);
        assert(dji_motor_status()->current_command < 0); assert_payload(id);
        dji_target_output_rpm = 0; step(id, -100, 25); assert_zero();
    }
    unsigned id = DJI_MOTOR_ID == 0 ? 1 : DJI_MOTOR_ID;
    reset_at(0); discover(id);
    tick += 20; sample(id, 10, 25); dji_motor_task(tick - 1);
    assert(dji_motor_status()->state == DJI_RUNNING); /* RX newer than caller's tick */

    reset_at(0); discover(id);
    tick += 110; dji_motor_task(tick);
    assert(dji_motor_status()->state == DJI_FAULT_FEEDBACK); assert_zero();
    step(id, 0, 25); assert(dji_motor_status()->state == DJI_FAULT_FEEDBACK); assert_zero();

    reset_at(0); discover(id); step(id, 0, 70);
    assert(dji_motor_status()->state == DJI_FAULT_TEMPERATURE); assert_zero();

    reset_at(0); discover(id); tick += 60; sample(id, 0, 25); dji_motor_task(tick);
    assert(dji_motor_status()->state == DJI_FAULT_TIMING); assert_zero();

    reset_at(0); discover(id); fail_tx = 1; step(id, 0, 25);
    assert(dji_motor_status()->state == DJI_FAULT_TX && abort_count == 1); assert_zero();

    reset_at(0); discover(id);
    for (unsigned n = 0; n < 500; ++n) step(id, 0, 25);
    assert(dji_motor_status()->state == DJI_FAULT_STALL); assert_zero();

    reset_at(0); discover(id); step(id, 2000, 25);
    assert(dji_motor_status()->state == DJI_FAULT_OVERSPEED); assert_zero();

    reset_at(0); discover(id); dji_target_output_rpm = NAN; step(id, 0, 25);
    assert(dji_motor_status()->state == DJI_FAULT_TARGET); assert_zero();

    reset_at(UINT32_MAX - 500U); discover(id); /* SysTick wrap-around */
    irq_mask = 1; step(id, 50, 25); assert(irq_mask == 1);
    irq_mask = 0;
#if DJI_MOTOR_ID == 0
    reset_at(0); step(1, 0, 25); step(2, 0, 25);
    assert(dji_motor_status()->state == DJI_FAULT_MULTIPLE); assert_zero();
    reset_at(0); discover(1); step(2, 0, 25);
    assert(dji_motor_status()->state == DJI_FAULT_MULTIPLE); assert_zero();
#else
    reset_at(0);
    for (unsigned n = 0; n < 110; ++n) {
        tick += 10; sample(1, 0, 25); sample(DJI_MOTOR_ID, 0, 25); dji_motor_task(tick);
    }
    assert(dji_motor_status()->state == DJI_RUNNING && dji_motor_status()->motor_id == DJI_MOTOR_ID);
    assert_payload(DJI_MOTOR_ID);
#endif
    puts("PASS: CAN isolation, discovery, slots/endian, ramp/limit, stop, faults, wrap, fixed/auto ID");
    return 0;
}
