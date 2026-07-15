/**
  ******************************************************************************
  * @file    app_cz.h
  * @brief   C(z) deadband-settle: no pulsar cerca del target.
  *
  * Política (LSB~12 µm):
  *   ae <= QUIET  → motors 0, acumular SETTLED (sin átomos)
  *   ae >= FIRE   → un micro-pulso y observar
  *   entre medias → esperar (ruido / creep), no bang-bang
  ******************************************************************************
  */

#ifndef __APP_CZ_H
#define __APP_CZ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_CZ_GATE_DEFAULT       1
/* Quiet: ≤2 LSB (~24 µm) → NUNCA disparar. */
#define APP_CZ_QUIET_ADD          1
/* Solo disparar si error ≥3 LSB (~36 µm). */
#define APP_CZ_FIRE_MIN_ADD       2
#define APP_CZ_SETTLE_US     300000U
#define APP_CZ_MAX_FIRES         16U

#define APP_CZ_STEP_NUM           1
#define APP_CZ_STEP_DEN          16

#define APP_CZ_SENS_Q12_INIT   768U
#define APP_CZ_SENS_Q12_MIN    128U
#define APP_CZ_SENS_Q12_MAX   4000U

#define APP_CZ_DUTY_MIN         35U
#define APP_CZ_DUTY_MAX        160U
#define APP_CZ_TON_MIN_US      150U
#define APP_CZ_TON_MAX_US      800U
#define APP_CZ_OBSERVE_US    350000U
#define APP_CZ_INIT_COOL_US   60000U

void app_cz_init(void);
void app_cz_enable(uint16_t ref_x_adc, uint16_t ref_y_adc);
void app_cz_enable_gate(uint16_t ref_x_adc, uint16_t ref_y_adc, uint16_t gate_adc);
void app_cz_disable(void);
int  app_cz_enabled(void);
int  app_cz_hold_active(void);
int  app_cz_settled(void);
void app_cz_set_invert(int inv_x, int inv_y);
void app_cz_tick_fast(uint16_t x_adc, uint16_t y_adc, uint16_t dt_us);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CZ_H */
