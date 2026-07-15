/**
  ******************************************************************************
  * @file    app_cz.h
  * @brief   C(z) fine: estimación + decisión + disparo en el mismo tick @ 1 MHz.
  *
  * Activa: F,<ref_x_adc>,<ref_y_adc>
  * Polaridad: I,<inv_x>,<inv_y>
  * Desactiva: M / A,*,* (pwm≠0) / B
  *
  * Sincronía: ADC EOC → EMA e → ley → atom_fire (si idle). Un solo cuanto = 1 µs.
  * COOLDOWN no es otra tasa de C(z): solo anti-apilado mecánico entre átomos.
  ******************************************************************************
  */

#ifndef __APP_CZ_H
#define __APP_CZ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* |e_adc| bajo esto → hold. ≥ paso típico átomo (log: idx0 spoil si gate=2–3). */
#define APP_CZ_GATE_ADC       8
/* Anti-apilado entre átomos (µs). La ley sigue evaluándose cada tick. */
#define APP_CZ_COOLDOWN_US    200000U

void app_cz_init(void);
void app_cz_enable(uint16_t ref_x_adc, uint16_t ref_y_adc);
void app_cz_disable(void);
int  app_cz_enabled(void);
void app_cz_set_invert(int inv_x, int inv_y);

/* Path caliente @ 1 MHz: estima, decide y dispara átomo (si idle). */
void app_cz_on_sample(uint16_t x_adc, uint16_t y_adc);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CZ_H */
