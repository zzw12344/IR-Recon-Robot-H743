#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

/* ================= Motor A ���� =================
 * PWMA -> PE9   TIM1_CH1, AF1
 * AIN1 -> PD8   GPIO �������
 * AIN2 -> PD9   GPIO �������
 * E1A  -> PA0   TIM2_CH1, AF1
 * E1B  -> PA1   TIM2_CH2, AF1
 * ================================================ */

#define MOTOR_A_PWM_TIM        TIM1
#define MOTOR_A_PWM_CH         TIM_CHANNEL_1
#define MOTOR_A_PWM_PORT       GPIOE
#define MOTOR_A_PWM_PIN        GPIO_PIN_9
#define MOTOR_A_PWM_AF         GPIO_AF1_TIM1

#define MOTOR_A_AIN1_PORT      GPIOD
#define MOTOR_A_AIN1_PIN       GPIO_PIN_8
#define MOTOR_A_AIN2_PORT      GPIOD
#define MOTOR_A_AIN2_PIN       GPIO_PIN_9

#define MOTOR_A_ENC_TIM        TIM2
#define MOTOR_A_ENC_PORT       GPIOA
#define MOTOR_A_ENC_A_PIN      GPIO_PIN_0
#define MOTOR_A_ENC_B_PIN      GPIO_PIN_1
#define MOTOR_A_ENC_AF         GPIO_AF1_TIM2

/* ================= Motor B ���� =================
 * PWMB -> PE11  TIM1_CH2, AF1
 * BIN1 -> PD10  GPIO �������
 * BIN2 -> PD11  GPIO �������
 * E2A  -> PB4   TIM3_CH1, AF2
 * E2B  -> PB5   TIM3_CH2, AF2
 * ================================================ */

#define MOTOR_B_PWM_TIM        TIM1
#define MOTOR_B_PWM_CH         TIM_CHANNEL_2
#define MOTOR_B_PWM_PORT       GPIOE
#define MOTOR_B_PWM_PIN        GPIO_PIN_11
#define MOTOR_B_PWM_AF         GPIO_AF1_TIM1

#define MOTOR_B_BIN1_PORT      GPIOD
#define MOTOR_B_BIN1_PIN       GPIO_PIN_10
#define MOTOR_B_BIN2_PORT      GPIOD
#define MOTOR_B_BIN2_PIN       GPIO_PIN_11

#define MOTOR_B_ENC_TIM        TIM3
#define MOTOR_B_ENC_PORT       GPIOB
#define MOTOR_B_ENC_A_PIN      GPIO_PIN_4
#define MOTOR_B_ENC_B_PIN      GPIO_PIN_5
#define MOTOR_B_ENC_AF         GPIO_AF2_TIM3

/* ================= Motor C ���ߣ��ڶ��� TB6612 �� A ͨ���� =================
 * PWMC -> PE13  TIM1_CH3, AF1
 * CIN1 -> PD12  GPIO �������
 * CIN2 -> PD13  GPIO �������
 * E3A  -> PB6   TIM4_CH1, AF2
 * E3B  -> PB7   TIM4_CH2, AF2
 * ================================================ */

#define MOTOR_C_PWM_TIM        TIM1
#define MOTOR_C_PWM_CH         TIM_CHANNEL_3
#define MOTOR_C_PWM_PORT       GPIOE
#define MOTOR_C_PWM_PIN        GPIO_PIN_13
#define MOTOR_C_PWM_AF         GPIO_AF1_TIM1

#define MOTOR_C_CIN1_PORT      GPIOD
#define MOTOR_C_CIN1_PIN       GPIO_PIN_12
#define MOTOR_C_CIN2_PORT      GPIOD
#define MOTOR_C_CIN2_PIN       GPIO_PIN_13

#define MOTOR_C_ENC_TIM        TIM4
#define MOTOR_C_ENC_PORT       GPIOB
#define MOTOR_C_ENC_A_PIN      GPIO_PIN_6
#define MOTOR_C_ENC_B_PIN      GPIO_PIN_7
#define MOTOR_C_ENC_AF         GPIO_AF2_TIM4

/* ================= Motor D ���ߣ��ڶ��� TB6612 �� B ͨ���� =================
 * PWMD -> PE14  TIM1_CH4, AF1
 * DIN1 -> PD14  GPIO �������
 * DIN2 -> PD15  GPIO �������
 * E4A  -> PC6   TIM8_CH1, AF3
 * E4B  -> PC7   TIM8_CH2, AF3
 * ================================================ */

#define MOTOR_D_PWM_TIM        TIM1
#define MOTOR_D_PWM_CH         TIM_CHANNEL_4
#define MOTOR_D_PWM_PORT       GPIOE
#define MOTOR_D_PWM_PIN        GPIO_PIN_14
#define MOTOR_D_PWM_AF         GPIO_AF1_TIM1

#define MOTOR_D_DIN1_PORT      GPIOD
#define MOTOR_D_DIN1_PIN       GPIO_PIN_14
#define MOTOR_D_DIN2_PORT      GPIOD
#define MOTOR_D_DIN2_PIN       GPIO_PIN_15

#define MOTOR_D_ENC_TIM        TIM8
#define MOTOR_D_ENC_PORT       GPIOC
#define MOTOR_D_ENC_A_PIN      GPIO_PIN_6
#define MOTOR_D_ENC_B_PIN      GPIO_PIN_7
#define MOTOR_D_ENC_AF         GPIO_AF3_TIM8

/* ================= ��е���� =================
 * 520 ���ٵ�������ٱ� 30�������� 11 �ߣ������һȦ 11 ��������
 * �����һȦ = 11 * 30 = 330 ��������AB �ı�Ƶ = 1320 ������
 * �־� 65mm
 * ========================================== */
#define ENCODER_LINES          11.0f
#define GEAR_RATIO             30.0f
#define ENCODER_COUNTS_PER_REV (ENCODER_LINES * GEAR_RATIO * 4.0f)   /* 1320 */
#define WHEEL_DIAMETER_MM      65.0f

/* ������������ţ��ֶ���ת����ʱ����λ��ʵ��ֵӦΪ������
 * �����ʾ�������ĳ� -1.0f */
#define ENC_DIR_SIGN_A         (1.0f)
#define ENC_DIR_SIGN_B         (1.0f)      /* �ֶ���ת��������ȷ��û���� */
#define ENC_DIR_SIGN_C         (-1.0f)     /* �ֶ���ת�������������� */
#define ENC_DIR_SIGN_D         (-1.0f)     /* �ֶ���ת�������������� */

/* �������ת��PID ���� PWM �������תʱ�ĳ� 1��
 * Motor B ��Ϊ BIN1/BIN2 ���߷��ˣ������� 1 �Ե� */
#define MOTOR_A_DIR_INVERT     (0)
#define MOTOR_B_DIR_INVERT     (1)
#define MOTOR_C_DIR_INVERT     (1)         /* Ŀ��+100 ʱ��ת���Ե������ƽ */
#define MOTOR_D_DIR_INVERT     (0)

/* ================= ���Ʋ��� ================= */
#define SPEED_LOOP_PERIOD_MS   20       /* 50Hz��ѭ����Ҫ������Ӧ��20ms �� 200mm/s ����Լ 26 ������������ */
#define PWM_ARR                999      /* 240MHz/(23+1)/(999+1) = 10kHz */
#define PWM_LIMIT              600      /* ռ�ձ����� 60%���ٶȲ���̫�� */
#define SPEED_TARGET_MAX       300.0f   /* Ŀ���ٶ����� mm/s */

void  MotorA_Init(void);
void  MotorA_SetPWM(int16_t pwm);
float MotorA_UpdateSpeed(void);
float MotorA_GetSpeed(void);

void  MotorB_Init(void);
void  MotorB_SetPWM(int16_t pwm);
float MotorB_UpdateSpeed(void);
float MotorB_GetSpeed(void);

void  MotorC_Init(void);
void  MotorC_SetPWM(int16_t pwm);
float MotorC_UpdateSpeed(void);
float MotorC_GetSpeed(void);

void  MotorD_Init(void);
void  MotorD_SetPWM(int16_t pwm);
float MotorD_UpdateSpeed(void);
float MotorD_GetSpeed(void);

#endif /* __MOTOR_H */
