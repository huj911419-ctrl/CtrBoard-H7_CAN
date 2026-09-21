/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "dbus.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};

/* MIT协议定点映射（PMAX/VMAX/TMAX=12.5/30/10，与上位机读到的保持一致） */
static uint16_t dm_motor_id = 0x000U;  /* 运行时使用扫描到的ESC_ID */
#define PMAX_F     12.5f
#define VMAX_F     30.0f
#define TMAX_F     10.0f
static float u2f(uint32_t x, float lo, float hi, int bits)
{
	return (float)x * (hi - lo) / (float)((1 << bits) - 1) + lo;
}

/* 通过 CAN 写参数寄存器：0x7FF + CANID + 0x55 + RID + 32位数据（低字节在前） */
static void dm_write_u32_param(uint32_t value, uint8_t rid)
{
	uint8_t wr[8] = {0};
	wr[0] = (uint8_t)(dm_motor_id & 0xFF);
	wr[1] = (uint8_t)((dm_motor_id >> 8) & 0xFF);
	wr[2] = 0x55;
	wr[3] = rid;
	wr[4] = (uint8_t)(value & 0xFF);
	wr[5] = (uint8_t)((value >> 8) & 0xFF);
	wr[6] = (uint8_t)((value >> 16) & 0xFF);
	wr[7] = (uint8_t)((value >> 24) & 0xFF);
	fdcanx_send_data(&hfdcan1, 0x7FF, wr, 8);
}

/* CTRL_MODE寄存器(0x0A)：1=MIT，2=位置速度，3=速度，4=力位混控 */
static void dm_set_mode(uint32_t mode)
{
	dm_write_u32_param(mode, 0x0A);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* CAN3 reserved: leave the peripheral and its pins uninitialized. */
  MX_USART10_UART_Init();
  MX_UART7_Init();       /* DT7/DR16 DBUS input: PE7, 100 kbaud, inverted */
  /* USER CODE BEGIN 2 */
	/* Board02 V1.1: PC14=CAN1 power, PC13=CAN2 power, PC15=external 5V.
	 * Keep the working DM power setup. C620 uses a separate 24V power branch. */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef pwr_gpio = {0};
	pwr_gpio.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	pwr_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	pwr_gpio.Pull  = GPIO_NOPULL;
	pwr_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &pwr_gpio);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

	HAL_Delay(2000);            /* 等电机上电自检完成（红灯常亮=失能，正常） */
	dji_motor_init(HAL_GetTick());
	dbus_init();
	bsp_can_init();             /* CAN1=DM, CAN2=C620, CAN3 reserved */
	HAL_Delay(100);

	log_print("\r\n=== DM-J4310-2EC ID scan & enable (CAN1, 1Mbps classic) ===\r\n");
	log_print("Flow: scan ESC_ID -> learn MST_ID -> speed mode -> wait for DBUS arm\r\n");
	log_print("DM LED: red=disabled, green=enabled, flashing red=fault\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ===== J4310-2EC 速度模式台架测试 =====
   * 先扫描ESC_ID并从参数应答学习MST_ID，再切换CTRL_MODE=3。
   * 使能后每10ms发送一次0x200+ESC_ID的4字节float速度命令，
   * 同时解析MST_ID反馈中的位置、速度、扭矩和双温度。 */
	/* 速度模式控制量：v_des单位rad/s，float小端发送。 */
	uint8_t speed_cmd[4];
	float speed_target = 2.0f;    /* 约19rpm，首次台架测试先用低速 */
	memcpy(speed_cmd, &speed_target, sizeof(speed_cmd));
	/* Preserve the existing DM parameter scan on CAN1 (0x00..0x7F). */
	can_tx_quiet = 1;
	uint8_t scanrd[4] = {0, 0, 0x33, 0x3C};
	uint16_t found_id = 0xFFFF;
	for (uint16_t cand = 0; cand <= 0x7F; cand++)
	{
		dm_rx33_flag = 0;
		scanrd[0] = (uint8_t)(cand & 0xFF);
		scanrd[1] = (uint8_t)(cand >> 8);
		fdcanx_send_data(&hfdcan1, 0x7FF, scanrd, 4);
		HAL_Delay(10);
		/* 0x33应答的CAN帧ID是MST_ID；真正的ESC_ID在D0/D1回显。 */
		if (dm_rx33_flag && dm_reply_rid == 0x3C && dm_reply_canid == cand)
		{
			found_id = cand;
			break;
		}
	}
	can_tx_quiet = 0;
	if (found_id != 0xFFFF)
	{
		dm_motor_id = found_id;
		log_print("[SCAN] ESC_ID=0x%03X MST_ID=0x%03X\r\n",
		          dm_motor_id, dm_master_id);
	}
	else
	{
		log_print("[SCAN] No valid DM reply in 0x00..0x7F; skip DM enable.\r\n");
		log_print("[CAN1] DM absent; CAN2 C620 test remains available.\r\n");
	}

	if (found_id != 0xFFFF)
	{
		/* 上电顺序：先失能 -> 写CTRL_MODE=3（速度模式） -> 再使能 */
		uint8_t dis[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
		fdcanx_send_data(&hfdcan1, dm_motor_id, dis, 8);
		HAL_Delay(100);

		/* 0x0A = CTRL_MODE，写入3表示速度模式。该操作立即生效，但不保存到Flash。 */
		dm_set_mode(3);
		log_print("[MODE] Requested CTRL_MODE=3 (speed).\r\n");
		HAL_Delay(100);

		log_print("[EN] Waiting for DBUS S1=%u before enabling ESC_ID=0x%03X.\r\n",
		          (unsigned)DBUS_ARM_S1_VALUE, dm_motor_id);

		/* 读参数诊断：读必非0的寄存器，客观验证通信+供电+存活（不依赖灯色）。
		 * 手册读参数帧只有4字节(D0=CANID_L,D1=CANID_H,D2=0x33,D3=RID)，发8字节DLC不匹配电机不响应 */
		uint8_t rd[4] = {
			(uint8_t)(dm_motor_id & 0xFF),
			(uint8_t)(dm_motor_id >> 8),
			0x33,
			0
		};
		uint8_t rids[] = {0x07, 0x10, 0x3C, 0x3E};  /* MST_ID/NPP/VBus/Tmtr */
		for (uint8_t i = 0; i < sizeof(rids); i++)
		{
			rd[3] = rids[i];
			fdcanx_send_data(&hfdcan1, 0x7FF, rd, 4);
			HAL_Delay(80);   /* 等应答帧(问询式，发一帧回一帧) */
		}

	} /* DM startup only when CAN1 scan succeeded. */

	log_print("[DJI|CAN2] ID config=%u (0=auto), DBUS-gated target=%ldrpm, current limit=%d.\r\n",
	          (unsigned)DJI_MOTOR_ID, (long)DJI_DEFAULT_OUTPUT_RPM, DJI_CURRENT_LIMIT);
	log_print("[DJI|CAN2] Listen 1s for C620 ID; CAN3 reserved.\r\n");
	can_tx_quiet = 1; /* periodic control must not print a UART line per frame */
	uint32_t last_dm_command = HAL_GetTick();
	uint32_t last_dji_log = HAL_GetTick();
	uint8_t dm_enabled = 0U;
	while (1)
	{
		uint32_t now = HAL_GetTick();
		dbus_data_t rc;
		dbus_get_data(&rc);
		uint8_t rc_online = dbus_is_online(now);
		uint8_t rc_armed = rc_online && dbus_is_armed(&rc);
		uint8_t friction_on = rc_armed && dbus_friction_enabled(&rc);
		/* The C620 loop remains closed on CAN2, but cannot arm without DBUS. */
		dji_run_enable = friction_on;
		dji_target_output_rpm = friction_on ? DJI_DEFAULT_OUTPUT_RPM : 0.0f;
		/* One DM motor is currently installed on CAN1. Stick CH1 commands speed;
		 * center stops it. This is a speed-mode gimbal command, not position mode. */
		speed_target = rc_armed ? dbus_channel_normalized(&rc, 0U) * 2.0f : 0.0f;
		memcpy(speed_cmd, &speed_target, sizeof(speed_cmd));

		if (found_id != 0xFFFF && dm_enabled != rc_armed)
		{
			dm_enabled = rc_armed;
			uint8_t command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			                     rc_armed ? 0xFC : 0xFD};
			fdcanx_send_data(&hfdcan1, dm_motor_id, command, 8);
		}
		dji_motor_task(now);
		/* 100Hz速度模式命令：ID=0x200+ESC_ID，4字节float，小端 */
		if (found_id != 0xFFFF && now - last_dm_command >= 10U)
		{
			last_dm_command = now;
			fdcanx_send_data(&hfdcan1, (uint16_t)(0x200 + dm_motor_id), speed_cmd, 4);
		}
		if (now - last_dji_log >= 1000U)
		{
			last_dji_log = now;
			const dji_status_t *dji = dji_motor_status();
			log_print("[DJI] %s id=%u seen=%02X out=%ldrpm cmd=%d T=%u rx=%lu\r\n",
			          dji_motor_state_name(dji->state), dji->motor_id, dji->seen_mask,
			          (long)((float)dji->rotor_rpm / DJI_GEAR_RATIO), dji->current_command,
			          dji->temperature, (unsigned long)dji->feedback_count);
			log_print("[RC] %s arm=%u friction=%u S1=%u S2=%u CH0=%d CH1=%d CH2=%d CH3=%d frames=%lu\r\n",
			          rc_online ? "ONLINE" : "WAIT", rc_armed, friction_on,
			          rc.s1, rc.s2, rc.channel[0], rc.channel[1], rc.channel[2], rc.channel[3],
			          (unsigned long)rc.frame_count);
		}

		/* 每秒打印帧计数：反馈帧ID使用运行时学习到的MST_ID */
		static uint32_t last_cnt = 0, prev_rx = 0, prev_fb = 0;
		if ((HAL_GetTick() - last_cnt) >= 1000)
		{
			uint32_t drx = dm_rx_count - prev_rx;
			uint32_t dfb = dm_fb_count - prev_fb;
			prev_rx = dm_rx_count;
			prev_fb = dm_fb_count;
			last_cnt = HAL_GetTick();
			log_print("[CNT] rx_total=%lu(+%lu/s) fb_mst(0x%03X)=%lu(+%lu/s)\r\n",
			          (unsigned long)dm_rx_count, (unsigned long)drx, dm_master_id,
			          (unsigned long)dm_fb_count, (unsigned long)dfb);
		}

		/* 解码最新反馈帧（ERR/位置/速度/扭矩/双温度），2Hz打印 */
		static uint32_t last_print = 0;
		if (dm_fb_new && (HAL_GetTick() - last_print) >= 500)
		{
			uint8_t fb[8];
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			for (unsigned i = 0; i < 8; ++i)
				fb[i] = dm_fb[i];
			dm_fb_new = 0;
			__set_PRIMASK(primask);
			last_print = HAL_GetTick();
			log_print("[RAW FB] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
			          fb[0], fb[1], fb[2], fb[3],
			          fb[4], fb[5], fb[6], fb[7]);
			uint8_t err = fb[0] >> 4;
			int  pos_raw = (fb[1] << 8) | fb[2];
			int  vel_raw = ((fb[3] & 0xFF) << 4) | (fb[4] >> 4);
			int  tau_raw = ((fb[4] & 0x0F) << 8) | fb[5];
			float pos = u2f(pos_raw, -PMAX_F, PMAX_F, 16);
			float vel = u2f(vel_raw, -VMAX_F, VMAX_F, 12);
			float tau = u2f(tau_raw, -TMAX_F, TMAX_F, 12);
			const char *err_s =
				(err == 0) ? "OFF" : (err == 1) ? "ON" :
				(err == 3) ? "OUT-ENC-CAL" : (err == 4) ? "SENSOR" :
				(err == 5) ? "MOTOR-ENC-CAL" : (err == 8) ? "OV" :
				(err == 9) ? "UV" : (err == 0xA) ? "OC" :
				(err == 0xB) ? "MOS-HOT" : (err == 0xC) ? "COIL-HOT" :
				(err == 0xD) ? "CAN-TIMEOUT" : (err == 0xE) ? "OVERLOAD" : "UNKNOWN";
			float ap = (pos < 0) ? -pos : pos;
			float av = (vel < 0) ? -vel : vel;
			float at = (tau < 0) ? -tau : tau;
			log_print("[FB] ERR=%X(%s) pos=%s%lu.%02lurad spd=%s%lu.%02lurad/s "
			          "tau=%s%lu.%02luNm T_MOS=%uC T_Rotor=%uC\r\n",
			          err, err_s,
			          (pos < 0) ? "-" : "", (unsigned long)ap, (unsigned long)((ap - (int)ap) * 100),
			          (vel < 0) ? "-" : "", (unsigned long)av, (unsigned long)((av - (int)av) * 100),
			          (tau < 0) ? "-" : "", (unsigned long)at, (unsigned long)((at - (int)at) * 100),
			          fb[6], fb[7]);
		}
		HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 极简串口日志：vsnprintf 格式化后阻塞发送到 USART10（PE3，115200）。
 * 学习阶段够用；注意两点（以后进阶再改）：
 * 1) 阻塞发送会占住CPU（一行约40字符≈3.5ms）；
 * 2) 在中断回调里打印属于学习用法，正式工程应改为中断/DMA发送。 */
void log_print(const char *fmt, ...)
{
	char buf[96];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;
	if (n >= (int)sizeof(buf))
		n = (int)sizeof(buf) - 1;
	HAL_UART_Transmit(&huart10, (uint8_t *)buf, (uint16_t)n, 50);
}

/* 把一帧 CAN 打印成一行，例如：
 * [TX|CAN1] ID=0x000  FF FC 00 00 00 00 00 00
 * len 为实际字节数（DLC），发几字节打几字节 */
void can_log(const char *dir, const char *port, uint16_t id, const uint8_t *data, uint8_t len)
{
	log_print("[%s|%s] ID=0x%03X ", dir, port, id);
	for (uint8_t i = 0; i < len && i < 8; i++)
		log_print("%02X ", data[i]);
	log_print("\r\n");
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
