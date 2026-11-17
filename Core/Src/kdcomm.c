#include "kdcomm.h"
#include "motor.h"
#include <string.h>

#define KD_RX_BUF_SIZE   64

static UART_HandleTypeDef s_huart1;
static uint8_t  s_rx_byte;
static volatile uint8_t s_ring[KD_RX_BUF_SIZE];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;

static uint8_t s_frame[10];
static uint8_t s_idx;

/* 每个通道一份 PID 参数；默认值先统一用 Motor A 调好的值，机械差异再单独微调 */
KD_Cmd_t g_kd_cmd[4] =
{
  { 0.8f, 13.0f, 0.0f, 0.0f, 0 },   /* 通道 1：Motor A */
  { 0.8f, 13.0f, 0.0f, 0.0f, 0 },   /* 通道 2：Motor B */
  { 0.8f, 13.0f, 0.0f, 0.0f, 0 },   /* 通道 3：Motor C */
  { 0.8f, 13.0f, 0.0f, 0.0f, 0 }    /* 通道 4：Motor D */
};

/* ------------------------- 帧打包 -------------------------
 * 指令帧：'C' | 指令ID | 参数1 | 参数2 | 参数3 | float(4字节) | '\n'
 * 数据帧：'0'~'3' | float 实际值 | float 目标值 | '\n'
 * --------------------------------------------------------- */
static void KD_PackCmd(uint8_t *f, uint8_t cmd, uint8_t p1, uint8_t p2,
                       uint8_t p3, float data)
{
  f[0] = 'C';
  f[1] = cmd;
  f[2] = p1;
  f[3] = p2;
  f[4] = p3;
  memcpy(&f[5], &data, 4);
  f[9] = '\n';
}

static void KD_PackData(uint8_t *f, uint8_t ch, float actual, float target)
{
  f[0] = (uint8_t)('0' + ch);
  memcpy(&f[1], &actual, 4);
  memcpy(&f[5], &target, 4);
  f[9] = '\n';
}

static void KD_Send(uint8_t *f)
{
  HAL_UART_Transmit(&s_huart1, f, 10, 20);
}

/* ------------------------- 初始化 ------------------------- */
void KD_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA9 = USART1_TX, PA10 = USART1_RX, AF7 */
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
  HAL_UART_Init(&s_huart1);

  s_head = 0;
  s_tail = 0;
  s_idx  = 0;

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&s_huart1, &s_rx_byte, 1);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&s_huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t next = (uint8_t)((s_head + 1U) % KD_RX_BUF_SIZE);

    if (next != s_tail)
    {
      s_ring[s_head] = s_rx_byte;
      s_head = next;
    }
    HAL_UART_Receive_IT(&s_huart1, &s_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(&s_huart1, &s_rx_byte, 1);
  }
}

/* ------------------------- 指令处理 ------------------------- */
static void KD_HandleFrame(uint8_t *f, const float actual[4])
{
  uint8_t out[10];
  uint8_t cmd;
  uint8_t p1;
  uint8_t p2;
  uint8_t ch;
  float   data;

  if (f[0] != 'C')
  {
    return;   /* 只处理指令帧 */
  }

  cmd = f[1];
  p1  = f[2];
  p2  = f[3];                       /* 0~3 对应上位机通道 1~4 */
  ch  = (p2 < 4U) ? p2 : 0U;
  memcpy(&data, &f[5], 4);

  switch (cmd)
  {
  case 1:   /* P */
    if (p1)
    {
      if (p2 < 4U)
      {
        g_kd_cmd[ch].kp = data;
      }
    }
    else
    {
      KD_PackCmd(out, 1, 1, ch, 0, g_kd_cmd[ch].kp);
      KD_Send(out);
    }
    break;

  case 2:   /* I */
    if (p1)
    {
      if (p2 < 4U)
      {
        g_kd_cmd[ch].ki = data;
      }
    }
    else
    {
      KD_PackCmd(out, 2, 1, ch, 0, g_kd_cmd[ch].ki);
      KD_Send(out);
    }
    break;

  case 3:   /* D */
    if (p1)
    {
      if (p2 < 4U)
      {
        g_kd_cmd[ch].kd = data;
      }
    }
    else
    {
      KD_PackCmd(out, 3, 1, ch, 0, g_kd_cmd[ch].kd);
      KD_Send(out);
    }
    break;

  case 4:   /* 目标值 */
    if (p1)
    {
      if (p2 < 4U)
      {
        if (data > SPEED_TARGET_MAX)
        {
          data = SPEED_TARGET_MAX;
        }
        if (data < -SPEED_TARGET_MAX)
        {
          data = -SPEED_TARGET_MAX;
        }
        g_kd_cmd[ch].target = data;
      }
    }
    else
    {
      KD_PackCmd(out, 4, 1, ch, 0, g_kd_cmd[ch].target);
      KD_Send(out);
    }
    break;

  case 6:   /* 读实际值 */
    if (!p1)
    {
      KD_PackCmd(out, 6, 1, ch, 0, actual[ch]);
      KD_Send(out);
    }
    break;

  case 7:   /* 开始运行：所有通道放开，各自按自己的目标跑 */
    for (ch = 0; ch < 4U; ch++)
    {
      g_kd_cmd[ch].run = 1;
    }
    break;

  case 8:   /* 停止运行：所有通道停车并清目标 */
    for (ch = 0; ch < 4U; ch++)
    {
      g_kd_cmd[ch].run    = 0;
      g_kd_cmd[ch].target = 0.0f;
    }
    break;

  default:
    break;
  }
}

/* 从环形缓冲区取字节，按 10 字节一帧解析 */
void KD_Poll(const float actual[4])
{
  while (s_tail != s_head)
  {
    uint8_t b = s_ring[s_tail];

    s_tail = (uint8_t)((s_tail + 1U) % KD_RX_BUF_SIZE);

    if (s_idx == 0)
    {
      if ((b == 'C') || ((b >= '0') && (b <= '3')))
      {
        s_frame[s_idx] = b;
        s_idx++;
      }
    }
    else
    {
      s_frame[s_idx] = b;
      s_idx++;

      if (s_idx >= 10U)
      {
        if (s_frame[9] == '\n')
        {
          KD_HandleFrame(s_frame, actual);
        }
        s_idx = 0;
      }
    }
  }
}

/* 每个控制周期把 4 个通道的数据帧发给上位机 */
void KD_SendAll(const float actual[4], const float target[4])
{
  uint8_t f[10];
  uint8_t ch;

  for (ch = 0; ch < 4U; ch++)
  {
    KD_PackData(f, ch, actual[ch], target[ch]);
    KD_Send(f);
  }
}
