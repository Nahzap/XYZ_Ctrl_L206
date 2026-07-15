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

/* LUT v7: mismos duty v6; t_on en µs (antes ms×1000). */
static const AtomLutEntry s_lut_a[APP_ATOM_LUT_LEN]
  __attribute__((section(".atom_lut"), used)) = {
    {  160U,  2000U }, {  220U,  2000U }, {  300U,  3000U }, {  400U,  3000U },
    {  520U,  4000U }, {  700U,  5000U }, {  920U,  7000U }, { 1200U, 10000U },
};

static const AtomLutEntry s_lut_b[APP_ATOM_LUT_LEN]
  __attribute__((section(".atom_lut"), used)) = {
    {  180U,  2000U }, {  240U,  2000U }, {  320U,  3000U }, {  420U,  3000U },
    {  560U,  4000U }, {  740U,  5000U }, {  980U,  7000U }, { 1280U, 10000U },
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
  if (s_fsm.state != ATOM_IDLE)
  {
    return -1;
  }
  if (sign == 0)
  {
    return -1;
  }

  const AtomLutEntry *lut = (axis == MOTOR_A) ? s_lut_a : s_lut_b;
  const AtomLutEntry *e = &lut[idx];
  int dir = (sign > 0) ? 1 : -1;
  int duty = (int)e->duty;
  if (duty < 1)
  {
    duty = 1;
  }
  if (duty > MOTOR_DUTY_MAX)
  {
    duty = MOTOR_DUTY_MAX;
  }

  s_fsm.axis = axis;
  s_fsm.duty = (int16_t)(dir * duty);
  s_fsm.remain_us = (e->t_on_us == 0U) ? 1U : (uint32_t)e->t_on_us;
  s_fsm.off_us = 500U; /* 500 µs coast tras ON */
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
