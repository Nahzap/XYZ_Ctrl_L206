/**
  ******************************************************************************
  * @file    app_motor.c
  * @brief   L298N motor actuation over TIM1/TIM8 PWM + direction GPIO.
  ******************************************************************************
  */

#include "app_motor.h"
#include "main.h"
#include "tim.h"

/* Direction pin roles (see main.h). '+' pin HIGH => forward (power > 0). */
#define MOTOR_A_PLUS_PORT   DIRECTION_MOTOR_A_E11_GPIO_Port   /* PE11 */
#define MOTOR_A_PLUS_PIN    DIRECTION_MOTOR_A_E11_Pin
#define MOTOR_A_MINUS_PORT  DIRECTION_MOTOR_A__GPIO_Port      /* PE9  */
#define MOTOR_A_MINUS_PIN   DIRECTION_MOTOR_A__Pin

#define MOTOR_B_PLUS_PORT   DIRECTION_MOTOR_B_A6_GPIO_Port    /* PA6  */
#define MOTOR_B_PLUS_PIN    DIRECTION_MOTOR_B_A6_Pin
#define MOTOR_B_MINUS_PORT  DIRECTION_MOTOR_B__GPIO_Port      /* PA5  */
#define MOTOR_B_MINUS_PIN   DIRECTION_MOTOR_B__Pin

static int clamp_power(int power)
{
  if (power > MOTOR_POWER_MAX)
  {
    return MOTOR_POWER_MAX;
  }
  if (power < -MOTOR_POWER_MAX)
  {
    return -MOTOR_POWER_MAX;
  }
  return power;
}

static int clamp_duty(int duty)
{
  if (duty > MOTOR_DUTY_MAX)
  {
    return MOTOR_DUTY_MAX;
  }
  if (duty < -MOTOR_DUTY_MAX)
  {
    return -MOTOR_DUTY_MAX;
  }
  return duty;
}

/* Escala Arduino 0..255 → CCR 0..ARR (cuantiza ~ARR/255 ≈ 17 ticks). */
static uint32_t power_to_ccr(int magnitude)
{
  return ((uint32_t)magnitude * MOTOR_PWM_ARR) / (uint32_t)MOTOR_POWER_MAX;
}

static void apply_signed_ccr(MotorAxis axis, int signed_level, uint32_t ccr)
{
  GPIO_TypeDef *plus_port  = (axis == MOTOR_A) ? MOTOR_A_PLUS_PORT  : MOTOR_B_PLUS_PORT;
  uint32_t      plus_pin   = (axis == MOTOR_A) ? MOTOR_A_PLUS_PIN   : MOTOR_B_PLUS_PIN;
  GPIO_TypeDef *minus_port = (axis == MOTOR_A) ? MOTOR_A_MINUS_PORT : MOTOR_B_MINUS_PORT;
  uint32_t      minus_pin  = (axis == MOTOR_A) ? MOTOR_A_MINUS_PIN  : MOTOR_B_MINUS_PIN;

  if (signed_level > 0)
  {
    LL_GPIO_SetOutputPin(plus_port, plus_pin);
    LL_GPIO_ResetOutputPin(minus_port, minus_pin);
  }
  else if (signed_level < 0)
  {
    LL_GPIO_ResetOutputPin(plus_port, plus_pin);
    LL_GPIO_SetOutputPin(minus_port, minus_pin);
  }
  else
  {
    LL_GPIO_ResetOutputPin(plus_port, plus_pin);
    LL_GPIO_ResetOutputPin(minus_port, minus_pin);
    ccr = 0U;
  }

  if (axis == MOTOR_A)
  {
    LL_TIM_OC_SetCompareCH3(TIM1, ccr);
  }
  else
  {
    LL_TIM_OC_SetCompareCH1(TIM8, ccr);
  }
}

void app_motor_init(void)
{
  /* Motor A -> TIM1 CH3 (advanced timer: needs main output enable). */
  LL_TIM_OC_SetCompareCH3(TIM1, 0);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  /* Motor B -> TIM8 CH1 (advanced timer: needs main output enable). */
  LL_TIM_OC_SetCompareCH1(TIM8, 0);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIM8);
  LL_TIM_EnableCounter(TIM8);

  app_motor_set(MOTOR_A, 0);
  app_motor_set(MOTOR_B, 0);
}

void app_motor_set(MotorAxis axis, int power)
{
  power = clamp_power(power);
  uint32_t ccr = (power == 0) ? 0U : power_to_ccr((power > 0) ? power : -power);
  apply_signed_ccr(axis, power, ccr);
}

void app_motor_set_duty(MotorAxis axis, int duty)
{
  duty = clamp_duty(duty);
  uint32_t ccr = (duty == 0) ? 0U : (uint32_t)((duty > 0) ? duty : -duty);
  if (ccr > MOTOR_PWM_ARR)
  {
    ccr = MOTOR_PWM_ARR;
  }
  apply_signed_ccr(axis, duty, ccr);
}

void app_motor_brake(MotorAxis axis)
{
  /* L298N active brake: both inputs at the same level with the enable driven,
     short-circuits the motor for a fast stop instead of coasting. */
  if (axis == MOTOR_A)
  {
    LL_GPIO_SetOutputPin(MOTOR_A_PLUS_PORT, MOTOR_A_PLUS_PIN);
    LL_GPIO_SetOutputPin(MOTOR_A_MINUS_PORT, MOTOR_A_MINUS_PIN);
    LL_TIM_OC_SetCompareCH3(TIM1, MOTOR_PWM_ARR);
  }
  else
  {
    LL_GPIO_SetOutputPin(MOTOR_B_PLUS_PORT, MOTOR_B_PLUS_PIN);
    LL_GPIO_SetOutputPin(MOTOR_B_MINUS_PORT, MOTOR_B_MINUS_PIN);
    LL_TIM_OC_SetCompareCH1(TIM8, MOTOR_PWM_ARR);
  }
}
