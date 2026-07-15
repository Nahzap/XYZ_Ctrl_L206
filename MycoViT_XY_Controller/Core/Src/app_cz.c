/**
  ******************************************************************************
  * @file    app_cz.c
  * @brief   C(z) @ 1 MHz síncrono: mismo tick mide, estima, decide y actúa.
  ******************************************************************************
  */

#include "app_cz.h"
#include "app_atom.h"
#include "app_rt.h"

typedef struct
{
  uint8_t  enabled;
  uint8_t  inv_x;
  uint8_t  inv_y;
  uint16_t ref_x;
  uint16_t ref_y;
  uint32_t cool_us;
  int32_t  e_ema_x;
  int32_t  e_ema_y;
  uint8_t  e_init;
} CzState;

static CzState s_cz __attribute__((section(".dtcm"), used));

static unsigned idx_from_abs_err(int ae)
{
  if (ae <= APP_CZ_GATE_ADC)
  {
    return 99U;
  }
  if (ae <= 6)
  {
    return 0U;
  }
  if (ae <= 12)
  {
    return 1U;
  }
  if (ae <= 24)
  {
    return 2U;
  }
  return 3U;
}

void app_cz_init(void)
{
  s_cz.enabled = 0U;
  s_cz.inv_x = 0U;
  s_cz.inv_y = 0U;
  s_cz.ref_x = 0U;
  s_cz.ref_y = 0U;
  s_cz.cool_us = 0U;
  s_cz.e_ema_x = 0;
  s_cz.e_ema_y = 0;
  s_cz.e_init = 0U;
}

void app_cz_enable(uint16_t ref_x_adc, uint16_t ref_y_adc)
{
  s_cz.ref_x = (ref_x_adc > 4095U) ? 4095U : ref_x_adc;
  s_cz.ref_y = (ref_y_adc > 4095U) ? 4095U : ref_y_adc;
  if (s_cz.enabled == 0U)
  {
    s_cz.cool_us = 0U;
    s_cz.e_init = 0U;
    s_cz.enabled = 1U;
  }
}

void app_cz_disable(void)
{
  s_cz.enabled = 0U;
  s_cz.cool_us = 0U;
}

int app_cz_enabled(void)
{
  return (s_cz.enabled != 0U) ? 1 : 0;
}

void app_cz_set_invert(int inv_x, int inv_y)
{
  s_cz.inv_x = (inv_x != 0) ? 1U : 0U;
  s_cz.inv_y = (inv_y != 0) ? 1U : 0U;
}

void app_cz_on_sample(uint16_t x_adc, uint16_t y_adc)
{
  if (s_cz.enabled == 0U)
  {
    return;
  }

  if (s_cz.cool_us > 0U)
  {
    s_cz.cool_us--;
  }

  /* Muestra del mismo tick (ISR ya actualizó EMA backbone; usar filtrado). */
  uint16_t xf = app_rt_get_x_filtered();
  uint16_t yf = app_rt_get_y_filtered();
  (void)x_adc;
  (void)y_adc;

  int ex = (int)s_cz.ref_x - (int)xf;
  int ey = (int)s_cz.ref_y - (int)yf;
  if (s_cz.inv_x)
  {
    ex = -ex;
  }
  if (s_cz.inv_y)
  {
    ey = -ey;
  }

  if (s_cz.e_init == 0U)
  {
    s_cz.e_ema_x = ex;
    s_cz.e_ema_y = ey;
    s_cz.e_init = 1U;
  }
  else
  {
    s_cz.e_ema_x += (ex - s_cz.e_ema_x) >> 5;
    s_cz.e_ema_y += (ey - s_cz.e_ema_y) >> 5;
  }

  /* Ley cada tick @ 1 MHz — no decimar. Si no puede actuar aún, igual actualiza e. */
  if (s_cz.cool_us > 0U || app_atom_busy())
  {
    return;
  }

  int aex = (s_cz.e_ema_x >= 0) ? (int)s_cz.e_ema_x : (int)(-s_cz.e_ema_x);
  int aey = (s_cz.e_ema_y >= 0) ? (int)s_cz.e_ema_y : (int)(-s_cz.e_ema_y);
  int err;
  MotorAxis axis;
  if (aex >= aey)
  {
    err = (int)s_cz.e_ema_x;
    axis = MOTOR_A;
  }
  else
  {
    err = (int)s_cz.e_ema_y;
    axis = MOTOR_B;
  }

  unsigned idx = idx_from_abs_err((err >= 0) ? err : -err);
  if (idx >= APP_ATOM_LUT_LEN)
  {
    return; /* hold: |e| ≤ gate */
  }

  int sign = (err > 0) ? 1 : -1;
  if (app_atom_fire(axis, sign, idx) == 0)
  {
    s_cz.cool_us = APP_CZ_COOLDOWN_US;
  }
}

void app_rt_control_hook(uint16_t x, uint16_t y)
{
  app_cz_on_sample(x, y);
}
