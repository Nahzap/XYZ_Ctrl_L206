/**
  ******************************************************************************
  * @file    app_atom.h
  * @brief   Mini-pulso micrométrico: LUT en Flash + FSM en DTCM @ 1 µs.
  *
  * Protocolo serial: P,<axis>,<sign>,<idx>
  *   axis: A|X|0 = MOTOR_A (X), B|Y|1 = MOTOR_B (Y)
  *   sign: +1 / -1
  *   idx:  índice LUT (duty CCR, t_on_us)
  *
  * Timing: app_atom_tick_1us() desde ISR 1 MHz (no SysTick).
  ******************************************************************************
  */

#ifndef __APP_ATOM_H
#define __APP_ATOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_motor.h"

#define APP_ATOM_LUT_LEN  8U

typedef struct
{
  uint16_t duty;     /* 1..MOTOR_DUTY_MAX (=ARR): CCR nativo */
  uint16_t t_on_us;  /* duración ON en microsegundos (tick RT 1 MHz) */
} AtomLutEntry;

void app_atom_init(void);

/* Dispara un mini-pulso. Retorna 0 si OK, -1 si idx inválido o busy. */
int app_atom_fire(MotorAxis axis, int sign, unsigned idx);

void app_atom_abort(void);

/* Avance 1 µs desde ISR RT. Reafirma duty durante ON / cierra pulso. */
void app_atom_tick_1us(void);

/* Compat: no-op (timing ya no vive en SysTick). */
void app_atom_tick_1ms(void);

int app_atom_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ATOM_H */
