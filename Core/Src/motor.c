#include "motor.h"

static TIM_HandleTypeDef s_htim_pwm;     /* TIM1，A/B/C/D 四个通道共用 */

static TIM_HandleTypeDef s_htim_enc_a;   /* TIM2，32 位 */
static TIM_HandleTypeDef s_htim_enc_b;   /* TIM3，16 位 */
static TIM_HandleTypeDef s_htim_enc_c;   /* TIM4，16 位 */
static TIM_HandleTypeDef s_htim_enc_d;   /* TIM8，16 位（H743 的 32 位定时器只有 TIM2/TIM5） */

static uint32_t s_last_cnt_a;
static uint32_t s_last_cnt_b;
static uint32_t s_last_cnt_c;
static uint32_t s_last_cnt_d;
static float    s_speed_a;
static float    s_speed_b;
static float    s_speed_c;
static float    s_speed_d;

/* ---------------- MSP 回调：时钟与引脚 ---------------- */

/* TIM1 PWM：PE9(CH1)+PE11(CH2)+PE13(CH3)+PE14(CH4)，四个电机共用 TIM1 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};

  if (htim->Instance == MOTOR_A_PWM_TIM)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio.Pin       = MOTOR_A_PWM_PIN | MOTOR_B_PWM_PIN |
                     MOTOR_C_PWM_PIN | MOTOR_D_PWM_PIN;   /* PE9|PE11|PE13|PE14 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = MOTOR_A_PWM_AF;                      /* AF1 = TIM1 */
    HAL_GPIO_Init(GPIOE, &gpio);
  }
}

/* 编码器 1：TIM2（PA0/PA1）；编码器 2：TIM3（PB4/PB5）
 * 编码器 3：TIM4（PB6/PB7）；编码器 4：TIM8（PC6/PC7） */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};

  if (htim->Instance == MOTOR_A_ENC_TIM)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = MOTOR_A_ENC_A_PIN | MOTOR_A_ENC_B_PIN;  /* PA0 | PA1 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = MOTOR_A_ENC_AF;                         /* AF1 = TIM2 */
    HAL_GPIO_Init(MOTOR_A_ENC_PORT, &gpio);
  }
  else if (htim->Instance == MOTOR_B_ENC_TIM)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = MOTOR_B_ENC_A_PIN | MOTOR_B_ENC_B_PIN;  /* PB4 | PB5 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = MOTOR_B_ENC_AF;                         /* AF2 = TIM3 */
    HAL_GPIO_Init(MOTOR_B_ENC_PORT, &gpio);
  }
  else if (htim->Instance == MOTOR_C_ENC_TIM)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = MOTOR_C_ENC_A_PIN | MOTOR_C_ENC_B_PIN;  /* PB6 | PB7 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = MOTOR_C_ENC_AF;                         /* AF2 = TIM4 */
    HAL_GPIO_Init(MOTOR_C_ENC_PORT, &gpio);
  }
  else if (htim->Instance == MOTOR_D_ENC_TIM)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin       = MOTOR_D_ENC_A_PIN | MOTOR_D_ENC_B_PIN;  /* PC6 | PC7 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = MOTOR_D_ENC_AF;                         /* AF3 = TIM8 */
    HAL_GPIO_Init(MOTOR_D_ENC_PORT, &gpio);
  }
}

/* ---------------- 方向脚 ---------------- */

static void MotorA_InitDirGpio(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin   = MOTOR_A_AIN1_PIN | MOTOR_A_AIN2_PIN;   /* PD8 | PD9 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &gpio);

  HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_RESET);
}

static void MotorB_InitDirGpio(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin   = MOTOR_B_BIN1_PIN | MOTOR_B_BIN2_PIN;   /* PD10 | PD11 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &gpio);

  HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_RESET);
}

static void MotorC_InitDirGpio(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin   = MOTOR_C_CIN1_PIN | MOTOR_C_CIN2_PIN;   /* PD12 | PD13 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &gpio);

  HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_RESET);
}

static void MotorD_InitDirGpio(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin   = MOTOR_D_DIN1_PIN | MOTOR_D_DIN2_PIN;   /* PD14 | PD15 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &gpio);

  HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_RESET);
}

/* ---------------- PWM ---------------- */

/* TIM1_CH1（PE9），10kHz */
static void MotorA_InitPwm(void)
{
  TIM_OC_InitTypeDef oc = {0};

  s_htim_pwm.Instance               = MOTOR_A_PWM_TIM;
  s_htim_pwm.Init.Prescaler         = 23;        /* 240MHz / 24 = 10MHz */
  s_htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim_pwm.Init.Period            = PWM_ARR;   /* 10MHz / 1000 = 10kHz */
  s_htim_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim_pwm.Init.RepetitionCounter = 0;
  s_htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&s_htim_pwm);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&s_htim_pwm, &oc, MOTOR_A_PWM_CH);
  HAL_TIM_PWM_Start(&s_htim_pwm, MOTOR_A_PWM_CH);
}

/* TIM1_CH2（PE11），与 A 共用同一个定时器和频率 */
static void MotorB_InitPwm(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};

  /* MSP 里已经配过，这里再配一次，保证单独调用 B 也不会漏 */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  gpio.Pin       = MOTOR_B_PWM_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = MOTOR_B_PWM_AF;
  HAL_GPIO_Init(MOTOR_B_PWM_PORT, &gpio);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&s_htim_pwm, &oc, MOTOR_B_PWM_CH);
  HAL_TIM_PWM_Start(&s_htim_pwm, MOTOR_B_PWM_CH);
}

/* TIM1_CH3（PE13），共用 TIM1 时基 */
static void MotorC_InitPwm(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  gpio.Pin       = MOTOR_C_PWM_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = MOTOR_C_PWM_AF;
  HAL_GPIO_Init(MOTOR_C_PWM_PORT, &gpio);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&s_htim_pwm, &oc, MOTOR_C_PWM_CH);
  HAL_TIM_PWM_Start(&s_htim_pwm, MOTOR_C_PWM_CH);
}

/* TIM1_CH4（PE14），共用 TIM1 时基 */
static void MotorD_InitPwm(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  gpio.Pin       = MOTOR_D_PWM_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = MOTOR_D_PWM_AF;
  HAL_GPIO_Init(MOTOR_D_PWM_PORT, &gpio);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&s_htim_pwm, &oc, MOTOR_D_PWM_CH);
  HAL_TIM_PWM_Start(&s_htim_pwm, MOTOR_D_PWM_CH);
}

/* ---------------- 编码器 ---------------- */

/* TIM2：32 位计数 */
static void MotorA_InitEncoder(void)
{
  TIM_Encoder_InitTypeDef enc = {0};

  s_htim_enc_a.Instance               = MOTOR_A_ENC_TIM;
  s_htim_enc_a.Init.Prescaler         = 0;
  s_htim_enc_a.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim_enc_a.Init.Period            = 0xFFFFFFFFUL;
  s_htim_enc_a.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim_enc_a.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  enc.EncoderMode  = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter    = 6;
  enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter    = 6;
  HAL_TIM_Encoder_Init(&s_htim_enc_a, &enc);
  HAL_TIM_Encoder_Start(&s_htim_enc_a, TIM_CHANNEL_ALL);

  s_last_cnt_a = __HAL_TIM_GET_COUNTER(&s_htim_enc_a);
}

/* TIM3：16 位计数 */
static void MotorB_InitEncoder(void)
{
  TIM_Encoder_InitTypeDef enc = {0};

  s_htim_enc_b.Instance               = MOTOR_B_ENC_TIM;
  s_htim_enc_b.Init.Prescaler         = 0;
  s_htim_enc_b.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim_enc_b.Init.Period            = 0xFFFFUL;
  s_htim_enc_b.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim_enc_b.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  enc.EncoderMode  = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter    = 6;
  enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter    = 6;
  HAL_TIM_Encoder_Init(&s_htim_enc_b, &enc);
  HAL_TIM_Encoder_Start(&s_htim_enc_b, TIM_CHANNEL_ALL);

  s_last_cnt_b = __HAL_TIM_GET_COUNTER(&s_htim_enc_b);
}

/* TIM4：16 位计数 */
static void MotorC_InitEncoder(void)
{
  TIM_Encoder_InitTypeDef enc = {0};

  s_htim_enc_c.Instance               = MOTOR_C_ENC_TIM;
  s_htim_enc_c.Init.Prescaler         = 0;
  s_htim_enc_c.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim_enc_c.Init.Period            = 0xFFFFUL;
  s_htim_enc_c.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim_enc_c.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  enc.EncoderMode  = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter    = 6;
  enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter    = 6;
  HAL_TIM_Encoder_Init(&s_htim_enc_c, &enc);
  HAL_TIM_Encoder_Start(&s_htim_enc_c, TIM_CHANNEL_ALL);

  s_last_cnt_c = __HAL_TIM_GET_COUNTER(&s_htim_enc_c);
}

/* TIM8：16 位计数（注意：TIM8 不是 32 位，H743 只有 TIM2/TIM5 是 32 位） */
static void MotorD_InitEncoder(void)
{
  TIM_Encoder_InitTypeDef enc = {0};

  s_htim_enc_d.Instance               = MOTOR_D_ENC_TIM;
  s_htim_enc_d.Init.Prescaler         = 0;
  s_htim_enc_d.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim_enc_d.Init.Period            = 0xFFFFUL;
  s_htim_enc_d.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim_enc_d.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  enc.EncoderMode  = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter    = 6;
  enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter    = 6;
  HAL_TIM_Encoder_Init(&s_htim_enc_d, &enc);
  HAL_TIM_Encoder_Start(&s_htim_enc_d, TIM_CHANNEL_ALL);

  s_last_cnt_d = __HAL_TIM_GET_COUNTER(&s_htim_enc_d);
}

/* ---------------- 对外接口 ---------------- */

void MotorA_Init(void)
{
  MotorA_InitDirGpio();
  MotorA_InitPwm();       /* 首次初始化 TIM1，会调用 MSP，同时把 PE9/PE11 都配好 */
  MotorA_InitEncoder();
  s_speed_a = 0.0f;
}

void MotorB_Init(void)
{
  MotorB_InitDirGpio();
  MotorB_InitPwm();       /* 共用已初始化的 TIM1，只配置 CH2 */
  MotorB_InitEncoder();
  s_speed_b = 0.0f;
}

void MotorC_Init(void)
{
  MotorC_InitDirGpio();
  MotorC_InitPwm();       /* 共用已初始化的 TIM1，只配置 CH3 */
  MotorC_InitEncoder();
  s_speed_c = 0.0f;
}

void MotorD_Init(void)
{
  MotorD_InitDirGpio();
  MotorD_InitPwm();       /* 共用已初始化的 TIM1，只配置 CH4 */
  MotorD_InitEncoder();
  s_speed_d = 0.0f;
}

/* pwm: -PWM_LIMIT ~ +PWM_LIMIT，正数前进，负数后退 */
void MotorA_SetPWM(int16_t pwm)
{
  if (pwm > PWM_LIMIT)
  {
    pwm = PWM_LIMIT;
  }
  if (pwm < -PWM_LIMIT)
  {
    pwm = -PWM_LIMIT;
  }

  if (pwm > 0)
  {
#if MOTOR_A_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_RESET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_A_PWM_CH, (uint32_t)pwm);
  }
  else if (pwm < 0)
  {
#if MOTOR_A_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_SET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_A_PWM_CH, (uint32_t)(-pwm));
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_A_AIN1_PORT, MOTOR_A_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_AIN2_PORT, MOTOR_A_AIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_A_PWM_CH, 0U);
  }
}

void MotorB_SetPWM(int16_t pwm)
{
  if (pwm > PWM_LIMIT)
  {
    pwm = PWM_LIMIT;
  }
  if (pwm < -PWM_LIMIT)
  {
    pwm = -PWM_LIMIT;
  }

  if (pwm > 0)
  {
#if MOTOR_B_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_RESET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_B_PWM_CH, (uint32_t)pwm);
  }
  else if (pwm < 0)
  {
#if MOTOR_B_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_SET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_B_PWM_CH, (uint32_t)(-pwm));
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_B_BIN1_PORT, MOTOR_B_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_BIN2_PORT, MOTOR_B_BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_B_PWM_CH, 0U);
  }
}

void MotorC_SetPWM(int16_t pwm)
{
  if (pwm > PWM_LIMIT)
  {
    pwm = PWM_LIMIT;
  }
  if (pwm < -PWM_LIMIT)
  {
    pwm = -PWM_LIMIT;
  }

  if (pwm > 0)
  {
#if MOTOR_C_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_RESET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_C_PWM_CH, (uint32_t)pwm);
  }
  else if (pwm < 0)
  {
#if MOTOR_C_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_SET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_C_PWM_CH, (uint32_t)(-pwm));
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_C_CIN1_PORT, MOTOR_C_CIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_C_CIN2_PORT, MOTOR_C_CIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_C_PWM_CH, 0U);
  }
}

void MotorD_SetPWM(int16_t pwm)
{
  if (pwm > PWM_LIMIT)
  {
    pwm = PWM_LIMIT;
  }
  if (pwm < -PWM_LIMIT)
  {
    pwm = -PWM_LIMIT;
  }

  if (pwm > 0)
  {
#if MOTOR_D_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_RESET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_D_PWM_CH, (uint32_t)pwm);
  }
  else if (pwm < 0)
  {
#if MOTOR_D_DIR_INVERT
    HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_SET);
#endif
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_D_PWM_CH, (uint32_t)(-pwm));
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_D_DIN1_PORT, MOTOR_D_DIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_D_DIN2_PORT, MOTOR_D_DIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&s_htim_pwm, MOTOR_D_PWM_CH, 0U);
  }
}

/* 每 50ms 调用一次：32 位计数器，直接相减 */
float MotorA_UpdateSpeed(void)
{
  uint32_t now   = __HAL_TIM_GET_COUNTER(&s_htim_enc_a);
  int32_t  delta = (int32_t)(now - s_last_cnt_a);
  float    mm_per_count = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

  s_last_cnt_a = now;
  s_speed_a = ((float)delta) * (1000.0f / (float)SPEED_LOOP_PERIOD_MS)
              * mm_per_count * ENC_DIR_SIGN_A;
  return s_speed_a;
}

/* TIM3 是 16 位计数器，必须按 16 位回绕算差值 */
float MotorB_UpdateSpeed(void)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&s_htim_enc_b);
  int16_t  delta = (int16_t)((uint16_t)now - (uint16_t)s_last_cnt_b);
  float    mm_per_count = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

  s_last_cnt_b = now;
  s_speed_b = ((float)delta) * (1000.0f / (float)SPEED_LOOP_PERIOD_MS)
              * mm_per_count * ENC_DIR_SIGN_B;
  return s_speed_b;
}

/* TIM4 是 16 位计数器，同 B 的算法 */
float MotorC_UpdateSpeed(void)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&s_htim_enc_c);
  int16_t  delta = (int16_t)((uint16_t)now - (uint16_t)s_last_cnt_c);
  float    mm_per_count = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

  s_last_cnt_c = now;
  s_speed_c = ((float)delta) * (1000.0f / (float)SPEED_LOOP_PERIOD_MS)
              * mm_per_count * ENC_DIR_SIGN_C;
  return s_speed_c;
}

/* TIM8 是 16 位计数器，同 B 的算法 */
float MotorD_UpdateSpeed(void)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&s_htim_enc_d);
  int16_t  delta = (int16_t)((uint16_t)now - (uint16_t)s_last_cnt_d);
  float    mm_per_count = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

  s_last_cnt_d = now;
  s_speed_d = ((float)delta) * (1000.0f / (float)SPEED_LOOP_PERIOD_MS)
              * mm_per_count * ENC_DIR_SIGN_D;
  return s_speed_d;
}

float MotorA_GetSpeed(void)
{
  return s_speed_a;
}

float MotorB_GetSpeed(void)
{
  return s_speed_b;
}

float MotorC_GetSpeed(void)
{
  return s_speed_c;
}

float MotorD_GetSpeed(void)
{
  return s_speed_d;
}
