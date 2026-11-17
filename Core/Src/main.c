/***
	*************************************************************************************************
	*	@file  	main.c
	*	@brief  8 路灰度传感器测试（YAHBOOM YB-MYX06-V1.0）
	*
	*   接线：
	*     传感器 5V  -> D24A 的 5V 输出
	*     传感器 GND -> GND
	*     传感器 AD0 -> PC0   （地址最低位）
	*     传感器 AD1 -> PC1   （地址中位）
	*     传感器 AD2 -> PC2   （地址最高位）
	*     传感器 OUT -> PA3   （ADC1_INP15，模拟输入，直连不用电阻）
	*
	*   串口：USART1 PA9/PA10，115200 8N1，接 CH340
	*
	*   上电后 1.5 秒自动标定白值（此时把传感器放在白纸或浅色桌面上）
	*   之后每 100ms 打印一行：8 路原始值、黑线中心位置、强反射（手指）中心位置
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include <stdio.h>
#include <string.h>


/********************************************** 函数声明 *******************************************/
void SystemClock_Config(void);

static void     DWT_Init(void);
static void     DelayUs(uint32_t us);
static void     UART1_Init(void);
static void     UART_Send(const char *s);
static void     Gray_GPIO_Init(void);
static void     ADC1_Init(void);
static uint16_t ADC_Read(void);
static void     Gray_Scan(void);
static void     Gray_Calibrate(void);
static void     Gray_CalcDark(float *center, float *weight, uint8_t *hit, uint8_t *valid);
static void     Gray_CalcBright(float *center, float *weight, uint8_t *hit, uint8_t *valid);
static void     FmtF(float v, int dec, char *out);


/********************************************** 可调参数 *******************************************/
#define GRAY_CH_NUM            8U
#define GRAY_ADC_FULL          4095.0f     /* 12 位 ADC 满量程 */
#define GRAY_SETTLE_US         100U        /* 通道切换后的稳定延时 */
#define GRAY_SCAN_PERIOD_MS    10U         /* 扫描周期 */
#define GRAY_PRINT_PERIOD_MS   100U        /* 打印周期 */
#define GRAY_STAT_PERIOD_MS    2000U       /* min/max 统计打印周期 */
#define GRAY_CALIB_MS          1500U       /* 上电标定时长 */
#define GRAY_PITCH_MM          12.0f       /* 相邻探头中心距，用于把中心换算成毫米 */

/* 判黑：原始值比白值低多少比例才算“黑线”。越小越灵敏，噪声也越容易被当成黑 */
#define GRAY_DARK_RATIO        0.35f

/* 判白：原始值比白值高多少比例才算“强反射物”（用手指遮挡测试用） */
#define GRAY_BRIGHT_RATIO      0.25f

/* 总权重低于此值视为丢线，不输出中心 */
#define GRAY_MIN_WEIGHT        0.30f

/* 传感器装反了（X1 实际在最右边）就把这里改成 1 */
#define GRAY_REVERSE           0


/********************************************** 地址线宏 *******************************************/
#define AD0_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define AD0_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define AD1_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define AD1_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define AD2_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define AD2_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)


/********************************************** 全局变量 *******************************************/
static ADC_HandleTypeDef  s_hadc1;
static UART_HandleTypeDef s_huart1;

static uint16_t s_raw[GRAY_CH_NUM];      /* 本次扫描的原始值 */
static uint16_t s_white[GRAY_CH_NUM];    /* 上电标定得到的白值 */
static uint16_t s_min[GRAY_CH_NUM];      /* 运行中见过的最小值（找阈值用） */
static uint16_t s_max[GRAY_CH_NUM];      /* 运行中见过的最大值（找阈值用） */


/***************************************************************************************************
*	函 数 名: main
*	函数功能: 上电标定白值，之后循环扫描 8 路灰度并输出中心位置
****************************************************************************************************/
int main(void)
{
	uint32_t scan_tick;
	uint32_t print_tick;
	uint32_t stat_tick;
	uint32_t led_tick;
	char     buf[180];
	char     f1[16];
	char     f2[16];
	int      i;

	SCB_EnableICache();
	SCB_EnableDCache();
	HAL_Init();
	SystemClock_Config();		/* 480MHz */

	LED_Init();
	DWT_Init();
	UART1_Init();
	Gray_GPIO_Init();
	ADC1_Init();

	UART_Send("\r\n=== YB-MYX06 8 路灰度测试 ===\r\n");
	UART_Send("正在标定白值，请把传感器放在白纸/浅色桌面上，保持 1.5 秒...\r\n");

	Gray_Calibrate();

	/* 打印标定得到的白值和对应阈值，这就是“好的固定阈值”的参考 */
	UART_Send("标定完成。白值 W 与判黑阈值 TH（低于 TH 判为黑线）：\r\n");
	snprintf(buf, sizeof(buf), "  W :%4u %4u %4u %4u %4u %4u %4u %4u\r\n",
	         s_white[0], s_white[1], s_white[2], s_white[3],
	         s_white[4], s_white[5], s_white[6], s_white[7]);
	UART_Send(buf);
	snprintf(buf, sizeof(buf), "  TH:%4u %4u %4u %4u %4u %4u %4u %4u\r\n",
	         (unsigned)(s_white[0] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[1] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[2] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[3] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[4] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[5] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[6] * (1.0f - GRAY_DARK_RATIO)),
	         (unsigned)(s_white[7] * (1.0f - GRAY_DARK_RATIO)));
	UART_Send(buf);
	UART_Send("\r\n用手指遮住一个或多个探头，看 BRIGHT 行的 E 是否指向挡住的位置\r\n");
	UART_Send("用黑胶带/黑纸盖住探头，看 DARK 行的 E 是否正确\r\n\r\n");

	scan_tick  = HAL_GetTick();
	print_tick = scan_tick;
	stat_tick  = scan_tick;
	led_tick   = scan_tick;

	while (1)
	{
		/* 10ms 扫描一轮 */
		if ((HAL_GetTick() - scan_tick) >= GRAY_SCAN_PERIOD_MS)
		{
			scan_tick += GRAY_SCAN_PERIOD_MS;
			Gray_Scan();

			/* 统计历史最值，帮助确定固定阈值 */
			for (i = 0; i < (int)GRAY_CH_NUM; i++)
			{
				if (s_raw[i] < s_min[i]) s_min[i] = s_raw[i];
				if (s_raw[i] > s_max[i]) s_max[i] = s_raw[i];
			}
		}

		/* 100ms 打印一次 */
		if ((HAL_GetTick() - print_tick) >= GRAY_PRINT_PERIOD_MS)
		{
			float   e_dark;
			float   w_dark;
			float   e_bright;
			float   w_bright;
			uint8_t hit_dark;
			uint8_t hit_bright;
			uint8_t ok_dark;
			uint8_t ok_bright;

			print_tick += GRAY_PRINT_PERIOD_MS;

			Gray_CalcDark(&e_dark, &w_dark, &hit_dark, &ok_dark);
			Gray_CalcBright(&e_bright, &w_bright, &hit_bright, &ok_bright);

			snprintf(buf, sizeof(buf), "R: %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
			         s_raw[0], s_raw[1], s_raw[2], s_raw[3],
			         s_raw[4], s_raw[5], s_raw[6], s_raw[7]);
			UART_Send(buf);

			/* 黑线中心：E 为探头序号单位（-3.5~+3.5），括号内为毫米 */
			FmtF(e_dark, 2, f1);
			FmtF(e_dark * GRAY_PITCH_MM, 1, f2);
			if (ok_dark)
			{
				snprintf(buf, sizeof(buf), "  DARK  E:%s (%s mm)  W:%u.%02u  HIT:0x%02X\r\n",
				         f1, f2,
				         (unsigned)w_dark, (unsigned)((w_dark - (unsigned)w_dark) * 100.0f + 0.5f),
				         hit_dark);
			}
			else
			{
				snprintf(buf, sizeof(buf), "  DARK  ---- 丢线（黑线未覆盖到探头）----\r\n");
			}
			UART_Send(buf);

			/* 强反射中心：手指遮挡时看这一行 */
			FmtF(e_bright, 2, f1);
			FmtF(e_bright * GRAY_PITCH_MM, 1, f2);
			if (ok_bright)
			{
				snprintf(buf, sizeof(buf), "  BRIGHT E:%s (%s mm)  W:%u.%02u  HIT:0x%02X\r\n",
				         f1, f2,
				         (unsigned)w_bright, (unsigned)((w_bright - (unsigned)w_bright) * 100.0f + 0.5f),
				         hit_bright);
			}
			else
			{
				snprintf(buf, sizeof(buf), "  BRIGHT ---- 无强反射（手指没挡住或挡得不够近）----\r\n");
			}
			UART_Send(buf);
		}

		/* 2 秒打印一次历史最值 */
		if ((HAL_GetTick() - stat_tick) >= GRAY_STAT_PERIOD_MS)
		{
			stat_tick += GRAY_STAT_PERIOD_MS;
			snprintf(buf, sizeof(buf), "  MIN:%4u %4u %4u %4u %4u %4u %4u %4u\r\n",
			         s_min[0], s_min[1], s_min[2], s_min[3],
			         s_min[4], s_min[5], s_min[6], s_min[7]);
			UART_Send(buf);
			snprintf(buf, sizeof(buf), "  MAX:%4u %4u %4u %4u %4u %4u %4u %4u\r\n\r\n",
			         s_max[0], s_max[1], s_max[2], s_max[3],
			         s_max[4], s_max[5], s_max[6], s_max[7]);
			UART_Send(buf);
		}

		/* 500ms 心跳 */
		if ((HAL_GetTick() - led_tick) >= 500U)
		{
			led_tick += 500U;
			LED1_Toggle;
		}
	}
}

/***************************************************************************************************
*	函 数 名: Gray_GPIO_Init
*	函数功能: AD0/AD1/AD2 三根地址线配成推挽输出
****************************************************************************************************/
static void Gray_GPIO_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &gpio);

	AD0_L();
	AD1_L();
	AD2_L();
}

/***************************************************************************************************
*	函 数 名: ADC1_Init
*	函数功能: PA3 = ADC1_INP15，12 位，异步时钟 CLKP/2 = 32MHz
****************************************************************************************************/
static void ADC1_Init(void)
{
	GPIO_InitTypeDef      gpio = {0};
	ADC_ChannelConfTypeDef cfg = {0};
	RCC_PeriphCLKInitTypeDef pclk = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC12_CLK_ENABLE();

	/* ADC 内核时钟：CLKP（默认 HSI 64MHz）/ 2 = 32MHz，VOS0 下安全 */
	pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	pclk.AdcClockSelection    = RCC_ADCCLKSOURCE_CLKP;
	if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
	{
		Error_Handler();
	}

	/* PA3 模拟模式：不能配上拉/下拉，否则内部分压会导致读数偏低 */
	gpio.Pin  = GPIO_PIN_3;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	s_hadc1.Instance                      = ADC1;
	s_hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;
	s_hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
	s_hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
	s_hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
	s_hadc1.Init.LowPowerAutoWait         = DISABLE;
	s_hadc1.Init.ContinuousConvMode       = DISABLE;
	s_hadc1.Init.NbrOfConversion          = 1;
	s_hadc1.Init.DiscontinuousConvMode    = DISABLE;
	s_hadc1.Init.NbrOfDiscConversion      = 0;
	s_hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
	s_hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
	s_hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	s_hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
	s_hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
	s_hadc1.Init.OversamplingMode         = DISABLE;

	if (HAL_ADC_Init(&s_hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/* 每次上电必须校准一次，否则前几十次读数会飘 */
	if (HAL_ADCEx_Calibration_Start(&s_hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	cfg.Channel      = ADC_CHANNEL_15;
	cfg.Rank         = ADC_REGULAR_RANK_1;
	cfg.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	cfg.SingleDiff   = ADC_SINGLE_ENDED;
	cfg.OffsetNumber = ADC_OFFSET_NONE;
	cfg.Offset       = 0;
	if (HAL_ADC_ConfigChannel(&s_hadc1, &cfg) != HAL_OK)
	{
		Error_Handler();
	}
}

/***************************************************************************************************
*	函 数 名: ADC_Read
*	函数功能: 软件触发一次转换，返回 0~4095
****************************************************************************************************/
static uint16_t ADC_Read(void)
{
	uint16_t v = 0;

	HAL_ADC_Start(&s_hadc1);
	if (HAL_ADC_PollForConversion(&s_hadc1, 5) == HAL_OK)
	{
		v = (uint16_t)HAL_ADC_GetValue(&s_hadc1);
	}
	HAL_ADC_Stop(&s_hadc1);

	return v;
}

/***************************************************************************************************
*	函 数 名: Gray_Scan
*	函数功能: 依次选中 0~7 号探头，各读一次 OUT
****************************************************************************************************/
static void Gray_Scan(void)
{
	uint8_t i;

	for (i = 0; i < (uint8_t)GRAY_CH_NUM; i++)
	{
		if (i & 0x01U) { AD0_H(); } else { AD0_L(); }
		if (i & 0x02U) { AD1_H(); } else { AD1_L(); }
		if (i & 0x04U) { AD2_H(); } else { AD2_L(); }

		DelayUs(GRAY_SETTLE_US);      /* 等八选一开关切换稳定 */

		s_raw[i] = ADC_Read();
	}
}

/***************************************************************************************************
*	函 数 名: Gray_Calibrate
*	函数功能: 1.5 秒内反复扫描，取每路平均值作为白值
****************************************************************************************************/
static void Gray_Calibrate(void)
{
	uint32_t t0;
	uint32_t sum[GRAY_CH_NUM] = {0};
	uint32_t cnt = 0;
	uint8_t  i;

	t0 = HAL_GetTick();

	while ((HAL_GetTick() - t0) < GRAY_CALIB_MS)
	{
		Gray_Scan();

		for (i = 0; i < (uint8_t)GRAY_CH_NUM; i++)
		{
			sum[i] += s_raw[i];
		}
		cnt++;

		LED1_Toggle;
		HAL_Delay(5);
	}

	if (cnt == 0U)
	{
		cnt = 1U;
	}

	for (i = 0; i < (uint8_t)GRAY_CH_NUM; i++)
	{
		s_white[i] = (uint16_t)(sum[i] / cnt);
		s_min[i]   = s_white[i];
		s_max[i]   = s_white[i];
	}
}

/***************************************************************************************************
*	函 数 名: Gray_CalcDark
*	函数功能: 按“比白值暗”的权重算中心位置（黑线）
*	形    参: center 中心（探头序号单位），weight 总权重，hit 命中位图，valid 是否有效
****************************************************************************************************/
static void Gray_CalcDark(float *center, float *weight, uint8_t *hit, uint8_t *valid)
{
	static const float pos[GRAY_CH_NUM] = {-3.5f, -2.5f, -1.5f, -0.5f,
	                                       0.5f,  1.5f,  2.5f,  3.5f};
	float   sw = 0.0f;
	float   se = 0.0f;
	uint8_t i;

	*hit = 0;

	for (i = 0; i < (uint8_t)GRAY_CH_NUM; i++)
	{
		float w = 0.0f;

		if (s_white[i] > 200U)
		{
			/* 相对变暗比例：0 = 和白值一样亮，1 = 全黑 */
			w = ((float)s_white[i] - (float)s_raw[i]) / (float)s_white[i];
		}

		if (w > GRAY_DARK_RATIO)
		{
			uint8_t idx = GRAY_REVERSE ? (uint8_t)(7U - i) : i;

			*hit |= (uint8_t)(1U << idx);
			sw   += w;
			se   += w * pos[idx];
		}
	}

	*weight = sw;

	if (sw < GRAY_MIN_WEIGHT)
	{
		*center = 0.0f;
		*valid  = 0U;
	}
	else
	{
		*center = se / sw;
		*valid  = 1U;
	}
}

/***************************************************************************************************
*	函 数 名: Gray_CalcBright
*	函数功能: 按“比白值更亮”的权重算中心位置（手指/白物遮挡）
****************************************************************************************************/
static void Gray_CalcBright(float *center, float *weight, uint8_t *hit, uint8_t *valid)
{
	static const float pos[GRAY_CH_NUM] = {-3.5f, -2.5f, -1.5f, -0.5f,
	                                       0.5f,  1.5f,  2.5f,  3.5f};
	float   sw = 0.0f;
	float   se = 0.0f;
	uint8_t i;

	*hit = 0;

	for (i = 0; i < (uint8_t)GRAY_CH_NUM; i++)
	{
		float w = 0.0f;
		float room = GRAY_ADC_FULL - (float)s_white[i];

		if (room > 100.0f)
		{
			/* 相对变亮比例：0 = 和白值一样，1 = 冲到满量程 */
			w = ((float)s_raw[i] - (float)s_white[i]) / room;
		}

		if (w > GRAY_BRIGHT_RATIO)
		{
			uint8_t idx = GRAY_REVERSE ? (uint8_t)(7U - i) : i;

			*hit |= (uint8_t)(1U << idx);
			sw   += w;
			se   += w * pos[idx];
		}
	}

	*weight = sw;

	if (sw < GRAY_MIN_WEIGHT)
	{
		*center = 0.0f;
		*valid  = 0U;
	}
	else
	{
		*center = se / sw;
		*valid  = 1U;
	}
}

/***************************************************************************************************
*	函 数 名: FmtF
*	函数功能: 把浮点数格式化成字符串，避开 Keil MicroLIB 对 %f 的支持问题
*	形    参: v 待格式化数值，dec 小数位数（1 或 2），out 输出缓冲（至少 16 字节）
****************************************************************************************************/
static void FmtF(float v, int dec, char *out)
{
	int   neg = (v < 0.0f) ? 1 : 0;
	float a   = neg ? -v : v;
	int   mul = (dec == 2) ? 100 : 10;
	int   x   = (int)(a * (float)mul + 0.5f);

	if (neg)
	{
		sprintf(out, "-%d.%0*d", x / mul, dec, x % mul);
	}
	else
	{
		sprintf(out, "%d.%0*d", x / mul, dec, x % mul);
	}
}

/***************************************************************************************************
*	函 数 名: DWT_Init
*	函数功能: 使能内核周期计数器，用于微秒级延时
****************************************************************************************************/
static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/***************************************************************************************************
*	函 数 名: DelayUs
*	函数功能: 忙等延时（基于 DWT，480MHz 下误差很小）
****************************************************************************************************/
static void DelayUs(uint32_t us)
{
	uint32_t start  = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000U);

	while ((DWT->CYCCNT - start) < cycles)
	{
		/* busy wait */
	}
}

/***************************************************************************************************
*	函 数 名: UART1_Init
*	函数功能: USART1 PA9/PA10，115200 8N1
****************************************************************************************************/
static void UART1_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_PULLUP;
	gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &gpio);

	s_huart1.Instance          = USART1;
	s_huart1.Init.BaudRate     = 115200;
	s_huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	s_huart1.Init.StopBits     = UART_STOPBITS_1;
	s_huart1.Init.Parity       = UART_PARITY_NONE;
	s_huart1.Init.Mode         = UART_MODE_TX_RX;
	s_huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	s_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&s_huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

/***************************************************************************************************
*	函 数 名: UART_Send
*	函数功能: 阻塞发送一个字符串
****************************************************************************************************/
static void UART_Send(const char *s)
{
	HAL_UART_Transmit(&s_huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/****************************************************************************************************/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 480000000 (CPU Clock)
  *            HCLK(Hz)                       = 240000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  120MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  120MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  120MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  120MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 192
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
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
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
