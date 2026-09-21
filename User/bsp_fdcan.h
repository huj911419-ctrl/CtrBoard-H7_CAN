#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);

extern volatile uint8_t dm_rx33_flag;      /* 收到0x33应答置1 */
extern volatile uint16_t dm_reply_canid;   /* 应答中回显的电机CAN ID */
extern volatile uint8_t dm_reply_rid;      /* 应答的寄存器地址 */
extern volatile uint16_t dm_master_id;     /* 电机反馈ID(MST_ID)，由参数应答自动学习 */
extern volatile uint8_t can_tx_quiet;      /* =1时发送不打印TX日志 */
extern volatile uint32_t dm_rx_count;      /* CAN1 收到的总帧数 */
extern volatile uint8_t dm_fb[8];          /* 最近一帧MIT反馈报文 */
extern volatile uint8_t dm_fb_new;         /* =1有未处理反馈 */
extern volatile uint32_t dm_fb_count;      /* MST_ID反馈帧计数 */

#endif /* __BSP_FDCAN_H_ */

