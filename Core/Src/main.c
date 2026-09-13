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
#include "bsp_fdcan.h"//
#include <stdio.h>
#include <stdarg.h>
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
#define MOTOR_ID   0x000
#define PMAX_F     12.5f
#define VMAX_F     30.0f
#define TMAX_F     10.0f
static int f2u(float x, float lo, float hi, int bits)
{
	return (int)((x - lo) * (float)((1 << bits) - 1) / (hi - lo));
}
static float u2f(uint32_t x, float lo, float hi, int bits)
{
	return (float)x * (hi - lo) / (float)((1 << bits) - 1) + lo;
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
  MX_FDCAN3_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
	/* ===== 普通模式：给电机口上电并发使能帧 =====
	 * 电机口电源由PMOS控制：上口OUT1(FDCAN1)=PC14，下口OUT2(FDCAN2)=PC13，
	 * 对外5V=PC15。V1.0原理图与V1.1管脚标注图对PC13/PC15标注有修订，
	 * 三个都拉高最稳妥：电机口必然有电，5V轨开启也无副作用。 */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef pwr_gpio = {0};
	pwr_gpio.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	pwr_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	pwr_gpio.Pull  = GPIO_NOPULL;
	pwr_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &pwr_gpio);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

	HAL_Delay(2000);            /* 等电机上电自检完成（红灯常亮=失能，正常） */
	bsp_can_init();             /* 三路FDCAN启动，1Mbps经典CAN */
	HAL_Delay(100);

	log_print("\r\n=== DM-J4310-2EC ID scan & enable (CAN1, 1Mbps classic) ===\r\n");
	log_print("流程: 扫描0x00~0x7F找电机真实ID -> 清错误 -> 使能 -> 轮询\r\n");
	log_print("观察电机灯: 常亮红=失能正常 绿=使能成功 闪烁红=有故障码(看[PARAM])\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ===== 安全保持模式：使能 + 100Hz零扭矩 + 遥测解码 =====
   * 已破案：电机ID=0x000，曾被故障闩死；FF×7+FB/FC 全填充格式有效。
   * 本循环每10ms发一帧MIT零扭矩（kp=kd=0、t_ff=0且按映射编码到0x7FF，
   * 注意MIT帧全零字节≠零扭矩，t_ff原始0会映射成-10Nm！），这同时
   * 触发电机每帧回反馈 → 解码出位置/速度/扭矩/双温度打印。 */
	/* MIT零扭矩帧：p=0 v=0 kp=0 kd=0 t_ff=0（t_ff编码值2047=0x7FF） */
	uint8_t mit0[8];
	int pi = f2u(0, -PMAX_F, PMAX_F, 16);   /* 32768 */
	int vi = f2u(0, -VMAX_F, VMAX_F, 12);   /* 2048  */
	int ti = f2u(0, -TMAX_F, TMAX_F, 12);   /* 2047  */
	mit0[0] = pi >> 8;  mit0[1] = pi & 0xFF;
	mit0[2] = vi >> 4;
	mit0[3] = (vi & 0xF) << 4;              /* kp高4位=0 */
	mit0[4] = 0;                            /* kp低8位   */
	mit0[5] = 0;                            /* kd[11:4]  */
	mit0[6] = (uint8_t)((0 & 0xF) << 4 | (ti >> 8));  /* kd[3:0]=0, t_ff[11:8] */
	mit0[7] = (uint8_t)(ti & 0xFF);         /* t_ff[7:0]  */
	/* 上电顺序：先失能(停掉一切残余动作) -> 再使能 */
	uint8_t dis[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	fdcanx_send_data(&hfdcan1, MOTOR_ID, dis, 8);
	log_print("[SAFE] 已发失能帧，清除残余动作\r\n");
	HAL_Delay(200);
	uint8_t en[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	fdcanx_send_data(&hfdcan1, MOTOR_ID, en, 8);
	log_print("[EN] 电机ID=0x%03X 已使能，进入零扭矩保持（绿灯应常亮，轴可用手转动）\r\n",
	          MOTOR_ID);

	while (1)
	{
		/* 100Hz零扭矩命令：维持使能状态 + 触发反馈帧 */
		fdcanx_send_data(&hfdcan1, MOTOR_ID, mit0, 8);

		/* 解码最新反馈帧（ERR/位置/速度/扭矩/双温度），2Hz打印 */
		static uint32_t last_print = 0;
		if (dm_fb_new && (HAL_GetTick() - last_print) >= 500)
		{
			dm_fb_new = 0;
			last_print = HAL_GetTick();
			uint8_t err = dm_fb[0] >> 4;
			int  pos_raw = (dm_fb[1] << 8) | dm_fb[2];
			int  vel_raw = ((dm_fb[3] & 0xFF) << 4) | (dm_fb[4] >> 4);
			int  tau_raw = ((dm_fb[4] & 0x0F) << 8) | dm_fb[5];
			float pos = u2f(pos_raw, -PMAX_F, PMAX_F, 16);
			float vel = u2f(vel_raw, -VMAX_F, VMAX_F, 12);
			float tau = u2f(tau_raw, -TMAX_F, TMAX_F, 12);
			const char *err_s =
				(err == 0) ? "失能" : (err == 1) ? "使能" :
				(err == 3) ? "输出轴校准异常" : (err == 4) ? "传感器输出异常" :
				(err == 5) ? "电机编码器校准异常" : (err == 8) ? "超压" :
				(err == 9) ? "欠压" : (err == 0xA) ? "过流" :
				(err == 0xB) ? "MOS过温" : (err == 0xC) ? "线圈过温" :
				(err == 0xD) ? "通讯丢失" : (err == 0xE) ? "过载" : "未知";
			float ap = (pos < 0) ? -pos : pos;
			float av = (vel < 0) ? -vel : vel;
			float at = (tau < 0) ? -tau : tau;
			log_print("[FB] ERR=%X(%s) pos=%s%lu.%02lurad spd=%s%lu.%02lurad/s "
			          "tau=%s%lu.%02luNm T_MOS=%uC T_Rotor=%uC\r\n",
			          err, err_s,
			          (pos < 0) ? "-" : "", (unsigned long)ap, (unsigned long)((ap - (int)ap) * 100),
			          (vel < 0) ? "-" : "", (unsigned long)av, (unsigned long)((av - (int)av) * 100),
			          (tau < 0) ? "-" : "", (unsigned long)at, (unsigned long)((at - (int)at) * 100),
			          dm_fb[6], dm_fb[7]);
		}
		HAL_Delay(10);
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
