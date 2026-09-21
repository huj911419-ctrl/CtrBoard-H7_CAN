#include "bsp_fdcan.h"
#include "dji_motor.h"
#include <string.h>

/* 只在确实发出0x33读参数请求时，才把匹配的8字节帧当成参数应答。
 * 避免普通状态反馈的D2偶然等于0x33时被误判。 */
static volatile uint8_t dm_param_waiting = 0;
static volatile uint16_t dm_expected_canid = 0xFFFF;
static volatile uint8_t dm_expected_opcode = 0;
static volatile uint8_t dm_expected_rid = 0;

/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void bsp_can_init(void)
{
    can_filter_init();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan2) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        Error_Handler();
    /* CAN3 is reserved: no start, notifications or motor protocol. */
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    /* CAN1 keeps accepting every standard ID for the existing DM ID scan. */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        Error_Handler();

    /* CAN2 only accepts C620 feedback IDs (motors 1..8). */
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterID1 = 0x201U;
    filter.FilterID2 = 0x208U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        Error_Handler();
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan��FDCAN���
* @param:       id��CAN�豸ID
* @param:       data�����͵�����
* @param:       len�����͵����ݳ���
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    /* This BSP is classic CAN only, with 8-byte message RAM elements. */
    if (data == NULL || len > 8U || id > 0x7FFU || hfdcan == &hfdcan3)
        return 1;
    FDCAN_TxHeaderTypeDef pTxHeader = {0};
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = len; /* bundled HAL uses unshifted DLC values */
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;   /* ����CAN���رձ��٣�BRS�� */
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;    /* ����CAN֡�������ֻ�Ͼ���֡��ԭ���̵�FD֡�ᱻ������� */
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	/* ���۳ɰܶ���ӡ���������� = CAN1 ������
	 * ע�����д�ӡ������֡"���"ʱ���������ķ�����Ӳ���첽��ɣ�
	 * ��������û�нڵ�Ӧ��(ACK)��֡�ᱻӲ�������ط���ѧϰҪ�㣩 */
	uint8_t is_param_request =
		(hfdcan == &hfdcan1) && (id == 0x7FFU) &&
		((len == 4U && data[2] == 0x33U) ||
		 (len == 8U && data[2] == 0x55U));
	if (is_param_request)
	{
		dm_expected_canid = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
		dm_expected_opcode = data[2];
		dm_expected_rid = data[3];
		dm_param_waiting = 1;
	}

	uint8_t ok = (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) == HAL_OK);
	if (!ok && is_param_request)
		dm_param_waiting = 0;

	if (!can_tx_quiet && hfdcan == &hfdcan1)
	{
		const char *port = (hfdcan->Instance == FDCAN1) ? "CAN1" :
		                   (hfdcan->Instance == FDCAN2) ? "CAN2" : "CAN3";
		can_log(ok ? "TX" : "TX-FAIL", port, (uint16_t)pTxHeader.Identifier, data, (uint8_t)len);
	}
	return ok ? 0 : 1;
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan��FDCAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        return 0;
    if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
        header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength > 8U)
        return 0;
    *rec_id = (uint16_t)header.Identifier;
    memcpy(buf, data, header.DataLength);
    return (uint8_t)header.DataLength;
}

/* ���0x33Ӧ���"�����ź�"����ѭ��ɨ��IDʱ�����ж���̽�Ƿ����� */
volatile uint8_t dm_rx33_flag = 0;
volatile uint16_t dm_reply_canid = 0xFFFF;
volatile uint8_t dm_reply_rid = 0;
volatile uint16_t dm_master_id = 0xFFFF;  /* 参数应答/状态反馈使用的MST_ID */
volatile uint8_t can_tx_quiet = 0;        /* =1ʱfdcanx_send_data����ӡTX��־(��ɨ��) */
volatile uint32_t dm_rx_count = 0;        /* �յ�����֡����������ID�����"�з�Ӧ"�ã� */
volatile uint8_t dm_fb[8] = {0};          /* ���һ֡��������ԭ�� */
volatile uint8_t dm_fb_new = 0;           /* =1��ʾ��δ�����ķ���֡ */
volatile uint32_t dm_fb_count = 0;        /* MST_ID反馈帧计数 */

/* 达妙参数应答与状态反馈共用MST_ID。
 * 参数应答必须结合当前请求的ESC_ID/opcode/RID匹配，剩余MST_ID帧才按状态反馈处理。 */
static void motor_frame_print(const char *port, uint16_t id, const uint8_t *d, uint8_t len)
{
	/* 参数应答的CAN帧ID是MST_ID，不应写死成0x10。
	 * D0/D1回显被访问的ESC_ID，同时匹配当前请求的opcode与RID。 */
	uint16_t echoed_canid = (len >= 2) ? (uint16_t)(d[0] | ((uint16_t)d[1] << 8)) : (uint16_t)0xFFFF;
	if (dm_param_waiting && len >= 8 &&
	    echoed_canid == dm_expected_canid &&
	    d[2] == dm_expected_opcode && d[3] == dm_expected_rid)
	{
		dm_param_waiting = 0;
		dm_reply_canid = echoed_canid;
		dm_reply_rid = d[3];
		dm_master_id = id;  /* 参数应答本身的帧ID就是MST_ID */

		uint32_t u = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
		           | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);

		if (d[2] == 0x33U)
		{
			dm_rx33_flag = 1;
			float f;
			memcpy(&f, &u, 4);

			if (d[3] == 0x07)          /* MST_ID 反馈ID */
			{
				dm_master_id = (uint16_t)u;
				log_print("[PARAM|%s] MST_ID = 0x%03X\r\n", port, dm_master_id);
			}
			else if (d[3] == 0x3C)     /* VBus 电源电压 */
			{
				uint32_t mv = (uint32_t)(f * 100.0f + 0.5f);
				log_print("[PARAM|%s] VBus = %lu.%02lu V\r\n", port,
				          (unsigned long)(mv / 100), (unsigned long)(mv % 100));
			}
			else if (d[3] == 0x50)     /* p_m 电机当前位置(rad) */
			{
				float a = (f < 0) ? -f : f;
				uint32_t cr = (uint32_t)(a * 100.0f + 0.5f);
				log_print("[PARAM|%s] p_m = %s%lu.%02lu rad\r\n", port, (f < 0) ? "-" : "",
				          (unsigned long)(cr / 100), (unsigned long)(cr % 100));
			}
			else
				log_print("[PARAM|%s] RID=0x%02X raw=0x%08lX\r\n",
				          port, d[3], (unsigned long)u);
		}
		else /* 0x55 写参数应答 */
		{
			log_print("[PARAM-WR|%s] RID=0x%02X raw=0x%08lX\r\n",
			          port, d[3], (unsigned long)u);
		}
	}
	else if (id == dm_master_id && len >= 8)
	{
		dm_fb_count++;                            /* 统计MST_ID反馈帧频率 */
		/* MITģʽ����֡��D0=ERR<<4|ID��D1~D5=λ��/�ٶ�/Ť�ض�������
		 * D6=T_MOS��D7=T_Rotor������������ѭ�������ӡ */
		for (uint8_t i = 0; i < 8 && i < len; i++)
			dm_fb[i] = d[i];
		dm_fb_new = 1;
	}
	else
	{
		can_log("RX", port, id, d, len);
	}
}

void fdcan1_rx_callback(void)
{
    uint8_t data[8];
    uint16_t id;
    uint8_t len = fdcanx_receive(&hfdcan1, &id, data);
    if (len != 0U) {
        ++dm_rx_count;
        motor_frame_print("CAN1", id, data, len);
    }
}

void fdcan2_rx_callback(void)
{
    uint8_t data[8];
    uint16_t id;
    uint8_t len = fdcanx_receive(&hfdcan2, &id, data);
    if (len != 0U)
        dji_motor_on_feedback(id, data, len, HAL_GetTick());
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
        return;
    if (hfdcan != &hfdcan1 && hfdcan != &hfdcan2)
        return;
    /* Drain FIFO: C620 sends feedback at 1 kHz by default. */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U) {
        if (hfdcan == &hfdcan1)
            fdcan1_rx_callback();
        else
            fdcan2_rx_callback();
    }
}
