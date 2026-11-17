#ifndef __KDCOMM_H
#define __KDCOMM_H

#include "stm32h7xx_hal.h"

/* 酷点 PID 调试助手（KD 协议）参数，每个通道一份 */
typedef struct
{
  float   kp;
  float   ki;
  float   kd;
  float   target;   /* 目标速度，单位 mm/s */
  uint8_t run;      /* 1: 收到“开始运行”，0: 收到“停止运行” */
} KD_Cmd_t;

extern KD_Cmd_t g_kd_cmd[4];

void KD_Init(void);
void KD_Poll(const float actual[4]);
void KD_SendAll(const float actual[4], const float target[4]);

#endif /* __KDCOMM_H */
