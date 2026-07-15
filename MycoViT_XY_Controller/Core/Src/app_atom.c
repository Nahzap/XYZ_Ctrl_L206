/**
  ******************************************************************************
  * @file    app_atom.c
  * @brief   Ejecutor de mini-pulso + LUT — duty nativo, tiempo en µs @ 1 MHz.
  ******************************************************************************
  */

#include "app_atom.h"

typedef enum
{
  ATOM_IDLE = 0,
  ATOM_ON,
  ATOM_OFF
} AtomState;

/* LUT v15: anti bang-bang. C(z) solo usa idx≤2; energía ~½–⅓ de v14.
 * Objetivo cerca de gate: pasos ~1–4 LSB (~3–12 µm), no spoil de 30–70 µm. */
static const AtomLutEntry s_lut_a[APP_ATOM_LUT_LEN]
  __attribute__((section(".atom_lut"), used)) = {
    {  220U,   1200U }, {  320U,   1800U }, {  450U,   2500U }, {  650U,   3500U },
    {  900U,   5000U }, { 1300U,   7000U }, { 1800U,   9000U }, { 2500U,  12000U },
};

static const AtomLutEntry s_lut_b[APP_ATOM_LUT_LEN]
  __attribute__((section(".atom_lut"), used)) = {
    {  240U,   1200U }, {  350U,   1800U }, {  480U,   2500U }, {  700U,   3500U },
    {  950U,   5000U }, { 1350U,   7000U }, { 1900U,   9000U }, { 2600U,  12000U },
};

typedef struct
{
  AtomState  state;
  MotorAxis  axis;
  int16_t    duty;
  uint32_t   remain_us;
  uint16_t   off_us;
} AtomFsm;

static AtomFsm s_fsm __attribute__((section(".dtcm"), used));

void app_atom_init(void)
{
  s_fsm.state = ATOM_IDLE;
  s_fsm.remain_us = 0U;
  s_fsm.off_us = 0U;
  s_fsm.duty = 0;
}

int app_atom_busy(void)
{
  return (s_fsm.state != ATOM_IDLE) ? 1 : 0;
}

void app_atom_abort(void)
{
  if (s_fsm.state == ATOM_IDLE)
  {
    return;
  }
  app_motor_set_duty(s_fsm.axis, 0);
  s_fsm.state = ATOM_IDLE;
  s_fsm.remain_us = 0U;
  s_fsm.duty = 0;
}

int app_atom_fire(MotorAxis axis, int sign, unsigned idx)
{
  if (idx >= APP_ATOM_LUT_LEN)
  {
    return -1;
  }
  const AtomLutEntry *lut = (axis == MOTOR_A) ? s_lut_a : s_lut_b;
  const AtomLutEntry *e = &lut[idx];
  return app_atom_fire_duty(axis, sign, e->duty, e->t_on_us);
}

int app_atom_fire_duty(MotorAxis axis, int sign, unsigned duty, uint32_t t_on_us)
{
  if (s_fsm.state != ATOM_IDLE)
  {
    return -1;
  }
  if (sign == 0)
  {
    return -1;
  }
  if (duty < 1U)
  {
    duty = 1U;
  }
  if (duty > (unsigned)MOTOR_DUTY_MAX)
  {
    duty = (unsigned)MOTOR_DUTY_MAX;
  }
  if (t_on_us == 0U)
  {
    t_on_us = 1U;
  }

  int dir = (sign > 0) ? 1 : -1;
  s_fsm.axis = axis;
  s_fsm.duty = (int16_t)(dir * (int)duty);
  s_fsm.remain_us = t_on_us;
  s_fsm.off_us = 500U;
  s_fsm.state = ATOM_ON;

  app_motor_set_duty(axis, (int)s_fsm.duty);
  return 0;
}

void app_atom_tick_1us(void)
{
  if (s_fsm.state == ATOM_IDLE)
  {
    return;
  }

  /* Solo cuenta tiempo; CCR se escribe al entrar/salir (no cada µs). */
  if (s_fsm.remain_us > 0U)
  {
    s_fsm.remain_us--;
  }

  if (s_fsm.state == ATOM_ON)
  {
    if (s_fsm.remain_us == 0U)
    {
      app_motor_set_duty(s_fsm.axis, 0);
      s_fsm.state = ATOM_OFF;
      s_fsm.remain_us = (s_fsm.off_us == 0U) ? 1U : (uint32_t)s_fsm.off_us;
    }
    return;
  }

  /* ATOM_OFF */
  if (s_fsm.remain_us == 0U)
  {
    s_fsm.state = ATOM_IDLE;
    s_fsm.duty = 0;
  }
}

void app_atom_tick_1ms(void)
{
  /* Timing migrado a app_atom_tick_1us() @ 1 MHz. */
}
