/***
	*************************************************************************************************
	*	@file  	main.c
	*	@brief  模式一：8 路灰度循线（串级控制：位置环 PD + 速度环 PI）
	*
	*   传感器：YAHBOOM YB-MYX06-V1.0（74HC4051 八选一，单 OUT 模拟输出）
	*     5V->5V  GND->GND  AD0->PC0  AD1->PC1  AD2->PC2  OUT->PA3(ADC1_INP15)
	*   电机接线见 motor.h（TIM1 PWM + 编码器 TIM2/3/4/8）
	*   轮子位置：A=右后  B=右前  C=左前  D=左后
	*   串口：USART1 PA9/PA10，115200 8N1，调试打印用
	*
	*   上电流程：
	*     1. 初始化 + 灰度标定 1.5 秒（此时传感器放在白纸/浅色桌面上）
	*     2. ST_IDLE：四轮不动，等黑线（连续 3 次有效才算，防反光误触发）
	*     3. ST_WAIT：检测到黑线后再停 5 秒，四轮仍不动（E 持续更新）
	*     4. ST_RUN ：开始循线
	*     5. 丢线超 1 秒或全黑异常持续 300ms -> ST_STOP 永久停车（需复位重启）
	*
	*   控制结构：
	*     每 10ms：扫 8 路 -> 算重心 E -> 一阶低通滤波
	*     每 20ms：位置环 PD -> 左右目标速度(mm/s) -> 速度环 PI -> PWM
	*     每 200ms：串口打印一行调试信息（正式比赛可注释掉）
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


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
static void     Gray_TrackWhite(void);
static void     Gray_CalcDark(float *center, float *weight, uint8_t *hit, uint8_t *valid);
static void     FmtF(float v, int dec, char *out);

static void     Motors_Stop(void);
static float    SpeedPI(uint8_t idx, float target, float actual);
static void     SpeedLoop(void);
static void     Control_20ms(void);
static void     Print_Debug(void);


/********************************************** 灰度参数 *******************************************/
#define GRAY_CH_NUM            8U
#define GRAY_SETTLE_US         100U        /* 通道切换后的稳定延时 */
#define GRAY_SCAN_PERIOD_MS    10U         /* 扫描周期 */
#define GRAY_CALIB_MS          1500U       /* 上电标定时长 */
#define GRAY_PITCH_MM          12.0f       /* 相邻探头中心距 */

/* 判黑：比白值暗多少比例才算黑线。越小越灵敏，也越容易误判 */
#define GRAY_DARK_RATIO        0.35f
/* 总权重低于此值视为丢线 */
#define GRAY_MIN_WEIGHT        0.30f
/* 白值慢跟踪：只学习明显偏白的通道，每次向当前读数靠 1% */
#define GRAY_TRACK_WHITE_W     0.15f
#define GRAY_TRACK_RATE        0.01f
/* 至少 7 路命中且平均权重很大，判为悬空/极端暗区，不当成黑线 */
#define GRAY_FULL_BLACK_CNT    7U
#define GRAY_FULL_BLACK_W      0.80f
#define FULL_BLACK_TIMEOUT_MS  300U
/* 传感器装反了（X1 实际在最右边）改成 1 */
#define GRAY_REVERSE           0


/********************************************** 节拍参数 *******************************************/
#define CTRL_PERIOD_MS         20U         /* 位置环 + 速度环周期 */
#define PRINT_PERIOD_MS        200U        /* 调试打印周期（阻塞约 6ms） */


/********************************************** 位置环参数（外环 PD） *****************************/
#define V_BASE                 200.0f      /* 基础前进速度 mm/s */
#define V_MIN_RATIO            0.4f        /* 偏差最大时速度降到基础的 40% */
#define KP_POS                 60.0f       /* 比例：每 1 个探头偏差 -> 60mm/s 差速 */
#define KD_POS                 6.0f        /* 微分：抑制来回摇摆，从 KP 的 0.1 倍起步 */
#define DV_LIMIT               200.0f      /* 差速限幅 mm/s */
#define V_TARGET_MAX           350.0f      /* 单轮目标速度上限 mm/s */
#define E_FILTER_NEW           0.3f        /* 一阶低通：新值占 30% */
#define E_FILTER_OLD           0.7f        /*             历史占 70% */


/********************************************** 速度环参数（内环 PI） *****************************/
/* 空载实测值，带载后若稳态误差偏大，优先加大 KI_SPD */
#define KP_SPD                 0.8f
#define KI_SPD                 13.0f
#define SPD_DT                 0.02f       /* = CTRL_PERIOD_MS / 1000 */
#define INTEGRAL_MAX           46.0f       /* 积分限幅：KI*46 = 600 = PWM_LIMIT */


/********************************************** 丢线参数 *******************************************/
#define LOST_SPEED             80.0f       /* 丢线后低速找线 mm/s */
#define LOST_TURN              80.0f       /* 叠加的转向差速：内侧轮停、外侧轮 160 */
#define LOST_TIMEOUT_MS        1000U       /* 丢线超过 1 秒停车 */


/********************************************** 启动参数 *******************************************/
#define START_WAIT_MS          5000U       /* 检测到黑线后再停 5 秒 */
#define DETECT_CONFIRM         3U          /* 连续 3 次有效才确认有黑线（30ms 防抖） */


/********************************************** 地址线宏 *******************************************/
#define AD0_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define AD0_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define AD1_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define AD1_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define AD2_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define AD2_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)


/********************************************** 运行状态 *******************************************/
typedef enum
{
	ST_IDLE = 0,   /* 等黑线，四轮不动 */
	ST_WAIT,       /* 已检测到，倒计时 5 秒，四轮不动 */
	ST_RUN,        /* 正常循线 */
	ST_STOP        /* 丢线超时或全黑异常，永久停车 */
} RunState;


/********************************************** 全局变量 *******************************************/
static ADC_HandleTypeDef  s_hadc1;
static UART_HandleTypeDef s_huart1;

static uint16_t s_raw[GRAY_CH_NUM];      /* 本次扫描原始值 */
static float    s_white[GRAY_CH_NUM];    /* 标定白值，运行中对偏白通道慢跟踪 */

static RunState s_state = ST_IDLE;

static float   s_e_raw = 0.0f;           /* 本次算出的重心（未滤波） */
static float   s_e     = 0.0f;           /* 滤波后的重心，控制用 */
static float   s_e_last = 0.0f;          /* 上一次重心，微分项用 */
static float   s_weight = 0.0f;
static uint8_t s_hit = 0;
static uint8_t s_valid = 0;              /* 0=丢线 1=正常黑线 2=全黑异常 */

static float   s_last_sign = 1.0f;       /* 丢线前最后一次偏差符号：+1 线在右，-1 线在左 */
static uint32_t s_lost_ms = 0;
static uint32_t s_dark_ms = 0;           /* 全黑异常持续时间 */

/* 轮子顺序固定：0=A右后 1=B右前 2=C左前 3=D左后 */
static float   s_target[4]  = {0};       /* 目标速度 mm/s */
static float   s_actual[4]  = {0};       /* 实际速度 mm/s */
static float   s_integral[4] = {0};      /* 速度环积分项 */

static float   s_v_base = 0.0f;
static float   s_dv     = 0.0f;
static uint32_t s_wait_start = 0;
static uint8_t  s_detect_cnt = 0;


/***************************************************************************************************
*	函 数 名: main
*	函数功能: 初始化后进入主循环，按 10/20/200/500ms 四个节拍调度
****************************************************************************************************/
int main(void)
{
	uint32_t scan_tick;
	uint32_t ctrl_tick;
	uint32_t print_tick;
	uint32_t led_tick;

	SCB_EnableICache();
	SCB_EnableDCache();
	HAL_Init();
	SystemClock_Config();		/* SYSCLK 480MHz，HCLK(CPU) 240MHz */

	LED_Init();
	DWT_Init();
	UART1_Init();
	Gray_GPIO_Init();
	ADC1_Init();

	MotorA_Init();
	MotorB_Init();
	MotorC_Init();
	MotorD_Init();

	UART_Send("\r\n=== 模式一：8 路灰度循线 ===\r\n");
	UART_Send("正在标定白值，请把传感器放在白纸/浅色桌面上，保持 1.5 秒...\r\n");

	Gray_Calibrate();

	UART_Send("标定完成。四轮保持不动，把黑线放到传感器下方即开始 5 秒倒计时。\r\n");

	scan_tick  = HAL_GetTick();
	ctrl_tick  = scan_tick;
	print_tick = scan_tick;
	led_tick   = scan_tick;

	while (1)
	{
		/* ---------------- 10ms：扫描 + 算重心 + 滤波 ---------------- */
		if ((HAL_GetTick() - scan_tick) >= GRAY_SCAN_PERIOD_MS)
		{
			scan_tick += GRAY_SCAN_PERIOD_MS;

			Gray_Scan();
			Gray_CalcDark(&s_e_raw, &s_weight, &s_hit, &s_valid);
			Gray_TrackWhite();

			if (s_valid == 1U)
			{
				/* 一阶低通：新值 30% + 历史 70%，压掉单帧毛刺 */
				s_e = E_FILTER_OLD * s_e + E_FILTER_NEW * s_e_raw;

				/* 记下偏差符号，丢线后要靠它决定往哪边找 */
				s_last_sign = (s_e >= 0.0f) ? 1.0f : -1.0f;
				s_lost_ms   = 0;
			}
		}

		/* ---------------- 20ms：状态机 + 位置环 + 速度环 ---------------- */
		if ((HAL_GetTick() - ctrl_tick) >= CTRL_PERIOD_MS)
		{
			ctrl_tick += CTRL_PERIOD_MS;
			Control_20ms();
		}

		/* ---------------- 200ms：调试打印 ---------------- */
		if ((HAL_GetTick() - print_tick) >= PRINT_PERIOD_MS)
		{
			print_tick += PRINT_PERIOD_MS;
			Print_Debug();
		}

		/* ---------------- LED：正常 500ms 心跳，停车 100ms 快闪 ---------------- */
		if ((HAL_GetTick() - led_tick) >= ((s_state == ST_STOP) ? 100U : 500U))
		{
			led_tick = HAL_GetTick();
			LED1_Toggle;
		}
	}
}

/***************************************************************************************************
*	函 数 名: Control_20ms
*	函数功能: 运行状态机。RUN 态下先位置环算出左右目标速度，再速度环输出 PWM
****************************************************************************************************/
static void Control_20ms(void)
{
	float vL;
	float vR;

	switch (s_state)
	{
	case ST_IDLE:
		Motors_Stop();
		if (s_valid == 1U)
		{
			s_detect_cnt++;
			if (s_detect_cnt >= DETECT_CONFIRM)
			{
				s_detect_cnt  = 0;
				s_wait_start  = HAL_GetTick();
				s_state       = ST_WAIT;
				UART_Send("检测到黑线，5 秒后启动...\r\n");
			}
		}
		else
		{
			s_detect_cnt = 0;
		}
		break;

	case ST_WAIT:
		Motors_Stop();       /* 等待期间四轮始终不动，但扫描仍在继续 */
		if ((HAL_GetTick() - s_wait_start) >= START_WAIT_MS)
		{
			s_e_last  = s_e;     /* 防止启动瞬间微分项爆出一个大值 */
			s_lost_ms = 0;
			s_state   = ST_RUN;
			UART_Send("启动！\r\n");
		}
		break;

	case ST_RUN:
		if (s_valid == 2U)
		{
			/* 全黑时先立即松油门；持续 300ms 认定异常，永久停车 */
			Motors_Stop();
			s_dark_ms += CTRL_PERIOD_MS;
			if (s_dark_ms >= FULL_BLACK_TIMEOUT_MS)
			{
				s_state = ST_STOP;
				UART_Send("全黑异常持续 300ms，停车。\r\n");
			}
			break;
		}
		s_dark_ms = 0;

		if (s_valid == 1U)
		{
			/* ===== 位置环（外环 PD）：输出左右目标速度，不直接碰 PWM ===== */

			/* ① 偏差越大基础速度越低，过弯不丢线 */
			s_v_base = V_BASE * (1.0f - (1.0f - V_MIN_RATIO) * fabsf(s_e) / 3.5f);

			/* ② PD 算差速：E>0 线在右 -> 左轮快右轮慢 -> 向右转 */
			s_dv = KP_POS * s_e + KD_POS * (s_e - s_e_last);
			if (s_dv >  DV_LIMIT) s_dv =  DV_LIMIT;
			if (s_dv < -DV_LIMIT) s_dv = -DV_LIMIT;

			vL = s_v_base + s_dv;
			vR = s_v_base - s_dv;
		}
		else
		{
			/* ===== 丢线：按最后一次偏差符号，低速往那边转着找 ===== */
			s_lost_ms += CTRL_PERIOD_MS;
			if (s_lost_ms >= LOST_TIMEOUT_MS)
			{
				s_state = ST_STOP;
				Motors_Stop();
				UART_Send("丢线超过 1 秒，停车。\r\n");
				break;
			}
			vL = LOST_SPEED + s_last_sign * LOST_TURN;
			vR = LOST_SPEED - s_last_sign * LOST_TURN;
		}

		/* 目标速度限幅：不允许反转，防止过度甩尾 */
		if (vL > V_TARGET_MAX) vL = V_TARGET_MAX;
		if (vR > V_TARGET_MAX) vR = V_TARGET_MAX;
		if (vL < 0.0f) vL = 0.0f;
		if (vR < 0.0f) vR = 0.0f;

		s_target[2] = vL;      /* C 左前 */
		s_target[3] = vL;      /* D 左后 */
		s_target[0] = vR;      /* A 右后 */
		s_target[1] = vR;      /* B 右前 */

		s_e_last = s_e;
		SpeedLoop();           /* ===== 速度环（内环 PI）===== */
		break;

	case ST_STOP:
	default:
		Motors_Stop();
		break;
	}
}

/***************************************************************************************************
*	函 数 名: SpeedLoop
*	函数功能: 四个轮子各跑一次 PI，输出 PWM
****************************************************************************************************/
static void SpeedLoop(void)
{
	s_actual[0] = MotorA_UpdateSpeed();
	s_actual[1] = MotorB_UpdateSpeed();
	s_actual[2] = MotorC_UpdateSpeed();
	s_actual[3] = MotorD_UpdateSpeed();

	MotorA_SetPWM((int16_t)SpeedPI(0, s_target[0], s_actual[0]));
	MotorB_SetPWM((int16_t)SpeedPI(1, s_target[1], s_actual[1]));
	MotorC_SetPWM((int16_t)SpeedPI(2, s_target[2], s_actual[2]));
	MotorD_SetPWM((int16_t)SpeedPI(3, s_target[3], s_actual[3]));
}

/***************************************************************************************************
*	函 数 名: SpeedPI
*	函数功能: 单轮速度 PI。P 管当前误差，I 管长期静差（克服摩擦和负载）
*	备    注: 积分必须限幅，否则车被卡住时积分无限涨，脱困瞬间会猛冲
****************************************************************************************************/
static float SpeedPI(uint8_t idx, float target, float actual)
{
	float err = target - actual;
	float out;

	s_integral[idx] += err * SPD_DT;
	if (s_integral[idx] >  INTEGRAL_MAX) s_integral[idx] =  INTEGRAL_MAX;
	if (s_integral[idx] < -INTEGRAL_MAX) s_integral[idx] = -INTEGRAL_MAX;

	out = KP_SPD * err + KI_SPD * s_integral[idx];

	if (out >  (float)PWM_LIMIT) out =  (float)PWM_LIMIT;
	if (out < -(float)PWM_LIMIT) out = -(float)PWM_LIMIT;

	return out;
}

/***************************************************************************************************
*	函 数 名: Motors_Stop
*	函数功能: 四轮 PWM 归零、清积分；编码器照常读数，保持计数器同步
****************************************************************************************************/
static void Motors_Stop(void)
{
	s_target[0] = s_target[1] = s_target[2] = s_target[3] = 0.0f;

	s_actual[0] = MotorA_UpdateSpeed();
	s_actual[1] = MotorB_UpdateSpeed();
	s_actual[2] = MotorC_UpdateSpeed();
	s_actual[3] = MotorD_UpdateSpeed();

	MotorA_SetPWM(0);
	MotorB_SetPWM(0);
	MotorC_SetPWM(0);
	MotorD_SetPWM(0);

	s_integral[0] = s_integral[1] = s_integral[2] = s_integral[3] = 0.0f;
}

/***************************************************************************************************
*	函 数 名: Print_Debug
*	函数功能: 一行打印状态、偏差、左右目标速度和四轮实际速度
*	输出示例: RUN E:+0.52 DV:+31.0 T:170/202 A:168/171/199/205 W:0 V:1 H:18
****************************************************************************************************/
static void Print_Debug(void)
{
	static const char *name[4] = {"IDLE", "WAIT", "RUN ", "STOP"};
	static uint8_t diag_cnt = 0;
	char buf[128];
	char fe[12];
	char fdv[12];
	uint32_t wait_left = 0;

	if (s_state == ST_WAIT)
	{
		uint32_t el = HAL_GetTick() - s_wait_start;
		wait_left = (el >= START_WAIT_MS) ? 0U : (START_WAIT_MS - el) / 1000U + 1U;
	}

	FmtF(s_e, 2, fe);
	FmtF(s_dv, 1, fdv);

	snprintf(buf, sizeof(buf), "%s E:%s DV:%s T:%d/%d A:%d/%d/%d/%d W:%u V:%u H:%02X\r\n",
	         name[(int)s_state], fe, fdv,
	         (int)s_target[2], (int)s_target[0],
	         (int)s_actual[0], (int)s_actual[1], (int)s_actual[2], (int)s_actual[3],
	         (unsigned)wait_left, (unsigned)s_valid, (unsigned)s_hit);
	UART_Send(buf);

	/* 每 1 秒打印一次 8 路原始值和当前白值，用于核对代码判定与板载 LED */
	diag_cnt++;
	if (diag_cnt >= 5U)
	{
		diag_cnt = 0;
		snprintf(buf, sizeof(buf), "R:%u/%u/%u/%u/%u/%u/%u/%u\r\n",
		         (unsigned)s_raw[0], (unsigned)s_raw[1],
		         (unsigned)s_raw[2], (unsigned)s_raw[3],
		         (unsigned)s_raw[4], (unsigned)s_raw[5],
		         (unsigned)s_raw[6], (unsigned)s_raw[7]);
		UART_Send(buf);
		snprintf(buf, sizeof(buf), "W:%u/%u/%u/%u/%u/%u/%u/%u\r\n",
		         (unsigned)s_white[0], (unsigned)s_white[1],
		         (unsigned)s_white[2], (unsigned)s_white[3],
		         (unsigned)s_white[4], (unsigned)s_white[5],
		         (unsigned)s_white[6], (unsigned)s_white[7]);
		UART_Send(buf);
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

	pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	pclk.AdcClockSelection    = RCC_ADCCLKSOURCE_CLKP;
	if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
	{
		Error_Handler();
	}

	/* PA3 模拟模式：不能配上拉/下拉，否则内部分压导致读数偏低 */
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
*	函数功能: 地址线二进制 0~7 依次选通 X1~X8，各读一次 OUT
****************************************************************************************************/
static void Gray_Scan(void)
{
	int i;

	for (i = 0; i < (int)GRAY_CH_NUM; i++)
	{
		if (i & 0x01U) { AD0_H(); } else { AD0_L(); }
		if (i & 0x02U) { AD1_H(); } else { AD1_L(); }
		if (i & 0x04U) { AD2_H(); } else { AD2_L(); }

		DelayUs(GRAY_SETTLE_US);   /* 等八选一开关切换稳定 */
		s_raw[i] = ADC_Read();
	}
}

/***************************************************************************************************
*	函 数 名: Gray_Calibrate
*	函数功能: 上电后在白纸上采约 300 轮，求平均作为每路白值
****************************************************************************************************/
static void Gray_Calibrate(void)
{
	uint32_t sum[GRAY_CH_NUM] = {0};
	uint32_t cnt = 0;
	uint32_t t0  = HAL_GetTick();
	int      i;

	while ((HAL_GetTick() - t0) < GRAY_CALIB_MS)
	{
		Gray_Scan();
		for (i = 0; i < (int)GRAY_CH_NUM; i++)
		{
			sum[i] += s_raw[i];
		}
		cnt++;
		LED1_Toggle;
		HAL_Delay(5);
	}

	if (cnt == 0U) cnt = 1U;

	for (i = 0; i < (int)GRAY_CH_NUM; i++)
	{
		s_white[i] = (float)sum[i] / (float)cnt;
	}
}

/***************************************************************************************************
*	函 数 名: Gray_TrackWhite
*	函数功能: 白值慢跟踪。只学习 w<0.15 的偏白通道，避免把黑线学成白值
*	备    注: 每 10ms 更新 1%，约 1 秒跟踪 63%，用于补偿光照和慢漂移
****************************************************************************************************/
static void Gray_TrackWhite(void)
{
	int i;

	for (i = 0; i < (int)GRAY_CH_NUM; i++)
	{
		if (s_white[i] > 200.0f)
		{
			float w = (s_white[i] - (float)s_raw[i]) / s_white[i];

			if (w < GRAY_TRACK_WHITE_W)
			{
				s_white[i] += ((float)s_raw[i] - s_white[i]) * GRAY_TRACK_RATE;
			}
		}
	}
}

/***************************************************************************************************
*	函 数 名: Gray_CalcDark
*	函数功能: 加权平均求黑线重心。E<0 线在左，E>0 线在右，0 为正中
*	备    注: valid：0=丢线，1=正常黑线，2=全黑异常（悬空/极端暗区）
****************************************************************************************************/
static void Gray_CalcDark(float *center, float *weight, uint8_t *hit, uint8_t *valid)
{
	static const float pos[GRAY_CH_NUM] = {-3.5f, -2.5f, -1.5f, -0.5f,
	                                        0.5f,  1.5f,  2.5f,  3.5f};
	float   sw = 0.0f;
	float   se = 0.0f;
	uint8_t cnt = 0;
	int     i;

	*hit = 0;

	for (i = 0; i < (int)GRAY_CH_NUM; i++)
	{
		float w = 0.0f;

		if (s_white[i] > 200.0f)
		{
			w = (s_white[i] - (float)s_raw[i]) / s_white[i];
		}

		if (w > GRAY_DARK_RATIO)
		{
			uint8_t idx = GRAY_REVERSE ? (uint8_t)(GRAY_CH_NUM - 1U - i) : (uint8_t)i;
			*hit |= (uint8_t)(1U << idx);
			sw   += w;
			se   += w * pos[idx];
			cnt++;
		}
	}

	*weight = sw;

	if ((cnt >= GRAY_FULL_BLACK_CNT) && ((sw / (float)cnt) > GRAY_FULL_BLACK_W))
	{
		*center = 0.0f;
		*valid  = 2U;      /* 近乎全部通道都很暗：不是可循的黑线 */
	}
	else if (sw < GRAY_MIN_WEIGHT)
	{
		*center = 0.0f;
		*valid  = 0U;      /* 丢线 */
	}
	else
	{
		*center = se / sw;
		*valid  = 1U;
	}
}

/***************************************************************************************************
*	函 数 名: FmtF
*	函数功能: 浮点转字符串（MicroLIB 的 printf 不支持 %f，只能先转整数再拼）
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
		sprintf(out, "+%d.%0*d", x / mul, dec, x % mul);
	}
}

/***************************************************************************************************
*	函 数 名: DWT_Init
*	函数功能: 打开内核周期计数器（240MHz 下 1us = 240 个计数）
****************************************************************************************************/
static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/***************************************************************************************************
*	函 数 名: DelayUs
*	函数功能: 微秒级忙等延时（无符号差值，计数器回绕也安全）
****************************************************************************************************/
static void DelayUs(uint32_t us)
{
	uint32_t start  = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000U);

	while ((DWT->CYCCNT - start) < cycles) { }
}

/***************************************************************************************************
*	函 数 名: UART1_Init
*	函数功能: USART1 PA9(TX)/PA10(RX)，115200 8N1
****************************************************************************************************/
static void UART1_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_PULLUP;
	gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
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
*	函数功能: 阻塞发送字符串。约 87us/字节，调试期 200ms 一次可接受
****************************************************************************************************/
static void UART_Send(const char *s)
{
	HAL_UART_Transmit(&s_huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/***************************************************************************************************
*	函 数 名: SystemClock_Config
*	函数功能: HSE 25M -> PLL 480M -> AHB/2 = HCLK 240M（CPU）-> APB/2 = 120M
****************************************************************************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
