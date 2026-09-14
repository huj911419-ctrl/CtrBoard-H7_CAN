#include "bsp_fdcan.h"
#include <string.h>
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;   /* 经典CAN：关闭变速（BRS） */
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;    /* 经典CAN帧——电机只认经典帧，原例程的FD帧会被电机无视 */
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	/* 无论成败都打印：串口所见 = CAN1 所发。
	 * 注意这行打印发生在帧"入队"时——真正的发送由硬件异步完成，
	 * 若总线上没有节点应答(ACK)，帧会被硬件反复重发（学习要点） */
	uint8_t ok = (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) == HAL_OK);
	if (!can_tx_quiet)
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
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;

	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		/* 本HAL的DataLength约定（已核对源码 stm32h7xx_hal_fdcan.c:3074）：
		 * TX/RX均为"DLC代码"原值——代码0~8=0~8字节，9~15=12/16/20/24/32/48/64字节
		 * （注意FDCAN_DLC_BYTES_12=0x09，不是12！之前的 n<<16 映射是错的，
		 *  曾导致8字节反馈帧被算成len=0、dm_fb永远全零） */
		uint32_t dlc = pRxHeader.DataLength;
		if      (dlc <= 8U)  len = (uint8_t)dlc;
		else if (dlc == 9U)  len = 12;
		else if (dlc == 10U) len = 16;
		else if (dlc == 11U) len = 20;
		else if (dlc == 12U) len = 24;
		else if (dlc == 13U) len = 32;
		else if (dlc == 14U) len = 48;
		else                 len = 64;

		return len;//接收数据
	}
	return 0;
}



/* 电机0x33应答的"捕获信号"：主循环扫描ID时靠它判断试探是否命中 */
volatile uint8_t dm_rx33_flag = 0;
volatile uint16_t dm_reply_canid = 0xFFFF;
volatile uint8_t dm_reply_rid = 0;
volatile uint8_t can_tx_quiet = 0;        /* =1时fdcanx_send_data不打印TX日志(快扫用) */
volatile uint32_t dm_rx_count = 0;        /* 收到的总帧数（含任意ID，检测"有反应"用） */
volatile uint8_t dm_fb[8] = {0};          /* 最近一帧反馈报文原文 */
volatile uint8_t dm_fb_new = 0;           /* =1表示有未处理的反馈帧 */
volatile uint32_t dm_fb_count = 0;        /* ID=0x10帧计数 */

/* 电机应答统一解码：读参数应答(帧ID=Master ID 0x10, D2==0x33)解出数值，
 * 其余帧打印原始内容。三个CAN口的回调都走这里，电机接哪路都能看到。 */
static void motor_frame_print(const char *port, uint16_t id, const uint8_t *d, uint8_t len)
{
	if (id == 0x10 && len >= 8 && d[2] == 0x33)
	{
		dm_rx33_flag = 1;                              /* 通知扫描器：命中了 */
		dm_reply_canid = d[0] | ((uint16_t)d[1] << 8); /* 应答里回显电机自己的ID */
		dm_reply_rid = d[3];

		uint32_t u = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
		           | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
		float f;
		memcpy(&f, &u, 4);

		if (d[3] == 0x3C)          /* VBus 电源电压 */
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
	else if (id == 0x10)
	{
		dm_fb_count++;                            /* 统计ID=0x10帧频率 */
		/* MIT模式反馈帧：D0=ERR<<4|ID，D1~D5=位置/速度/扭矩定点数，
		 * D6=T_MOS、D7=T_Rotor。存起来给主循环解码打印 */
		for (uint8_t i = 0; i < 8 && i < len; i++)
			dm_fb[i] = d[i];
		dm_fb_new = 1;
	}
	else
	{
		can_log("RX", port, id, d, len);
	}
}

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
	uint8_t len = fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
	motor_frame_print("CAN1", rec_id1, rx_data1, len);
}
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
	uint8_t len = fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
	motor_frame_print("CAN2", rec_id2, rx_data2, len);
}
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
	uint8_t len = fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
	motor_frame_print("CAN3", rec_id3, rx_data3, len);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	dm_rx_count++;    /* 任何一路收到任何帧都计数：电机"开口说话"的信号 */
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
	if(hfdcan == &hfdcan3)
	{
		fdcan3_rx_callback();
	}
}











