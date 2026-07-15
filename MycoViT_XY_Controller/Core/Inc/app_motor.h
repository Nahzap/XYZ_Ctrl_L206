/**
  ******************************************************************************
  * @file    app_motor.h
  * @brief   L298N motor actuation over TIM1/TIM8 PWM + direction GPIO.
  *          Port of the Arduino setMotorPower() semantics.
  ******************************************************************************
  */

#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PWM timers: ARR = 4319 → 4320 niveles de duty (~12 bit), ~50 kHz (tim.c). */
#define MOTOR_PWM_ARR   4319U
/* Escala coarse (comando A / UI Arduino): -255..255 → mapea a 0..ARR. */
#define MOTOR_POWER_MAX 255
/* Escala fine nativa (átomos / LUT): -ARR..+ARR, 1 LSB = 1 tick CCR. */
#define MOTOR_DUTY_MAX  ((int)MOTOR_PWM_ARR)
/* Escala intermedia documentada (2^9-1): útil si se quiere API 9-bit sin ARR. */
#define MOTOR_FINE9_MAX 511

typedef enum
{
  /* Laboratorio MycoViT / GUI: A = eje X (Sensor 2), B = eje Y (Sensor 1). */
  MOTOR_A = 0,   /* TIM1 CH3 / PE13, dir PE11(+) PE9(-)  -> eje X */
  MOTOR_B = 1    /* TIM8 CH1 / PC6,  dir PA6(+) PA5(-)  -> eje Y */
} MotorAxis;

/* Enable PWM outputs (MOE), CC channels and counters; force both motors to 0. */
void app_motor_init(void);

/* Apply a signed power [-255..255] to one axis (constrained internally). */
void app_motor_set(MotorAxis axis, int power);

/* Duty nativo firmado [-MOTOR_DUTY_MAX..+MOTOR_DUTY_MAX] (sin cuantizar a 8 bit). */
void app_motor_set_duty(MotorAxis axis, int duty);

/* Active brake (both L298N inputs high, enable full): fast stop / hold. */
void app_motor_brake(MotorAxis axis);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTOR_H */
