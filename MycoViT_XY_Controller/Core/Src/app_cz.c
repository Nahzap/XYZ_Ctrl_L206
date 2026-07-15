/**
  ******************************************************************************
  * @file    app_cz.c
  * @brief   Deadband settle: quiet→SETTLED; solo pulsar si ae ≥ fire_min.
  ******************************************************************************
  */

#include "app_cz.h"
#include "app_atom.h"

typedef struct
{
  int8_t   pol;
  uint16_t sens_q12;
  uint8_t  learned;
} AxisModel;

typedef struct
{
  uint8_t  enabled;
  uint8_t  hold_latched;
  uint8_t  settled;
  uint8_t  capped;
  uint8_t  fire_n;
  uint8_t  e_init;
  uint8_t  pending_learn;
  uint8_t  last_axis;
  int8_t   last_u_sign;
  int16_t  e_pre;
  uint16_t last_E;
  uint16_t gate;
  uint16_t quiet;
  uint16_t fire_min;
  uint16_t ref_x;
  uint16_t ref_y;
  uint32_t cool_us;
  uint32_t settle_us;
  int32_t  e_ema_x;
  int32_t  e_ema_y;
  AxisModel ax;
  AxisModel ay;
} CzState;

static CzState s_cz __attribute__((section(".dtcm"), used));

static int iabs(int v)
{
  return (v >= 0) ? v : -v;
}

static uint16_t u16_clamp(uint32_t v, uint16_t lo, uint16_t hi)
{
  if (v < (uint32_t)lo)
  {
    return lo;
  }
  if (v > (uint32_t)hi)
  {
    return hi;
  }
  return (uint16_t)v;
}

static void model_init(AxisModel *m, int8_t pol_seed)
{
  m->pol = (pol_seed >= 0) ? 1 : -1;
  m->sens_q12 = (uint16_t)APP_CZ_SENS_Q12_INIT;
  m->learned = 0U;
}

static void apply_gate(uint16_t gate_adc)
{
  if (gate_adc < 1U)
  {
    gate_adc = 1U;
  }
  if (gate_adc > 40U)
  {
    gate_adc = 40U;
  }
  s_cz.gate = gate_adc;
  s_cz.quiet = (uint16_t)(gate_adc + (uint16_t)APP_CZ_QUIET_ADD);
  s_cz.fire_min = (uint16_t)(gate_adc + (uint16_t)APP_CZ_FIRE_MIN_ADD);
  if (s_cz.fire_min < 2U)
  {
    s_cz.fire_min = 2U;
  }
}

void app_cz_init(void)
{
  s_cz.enabled = 0U;
  s_cz.hold_latched = 0U;
  s_cz.settled = 0U;
  s_cz.capped = 0U;
  s_cz.fire_n = 0U;
  s_cz.e_init = 0U;
  s_cz.pending_learn = 0U;
  s_cz.last_axis = 0xFFU;
  s_cz.last_u_sign = 0;
  s_cz.e_pre = 0;
  s_cz.last_E = 0U;
  apply_gate((uint16_t)APP_CZ_GATE_DEFAULT);
  s_cz.ref_x = 0U;
  s_cz.ref_y = 0U;
  s_cz.cool_us = 0U;
  s_cz.settle_us = 0U;
  s_cz.e_ema_x = 0;
  s_cz.e_ema_y = 0;
  model_init(&s_cz.ax, 1);
  model_init(&s_cz.ay, 1);
}

void app_cz_enable(uint16_t ref_x_adc, uint16_t ref_y_adc)
{
  app_cz_enable_gate(ref_x_adc, ref_y_adc,
                     s_cz.gate ? s_cz.gate : (uint16_t)APP_CZ_GATE_DEFAULT);
}

void app_cz_enable_gate(uint16_t ref_x_adc, uint16_t ref_y_adc, uint16_t gate_adc)
{
  s_cz.ref_x = (ref_x_adc > 4095U) ? 4095U : ref_x_adc;
  s_cz.ref_y = (ref_y_adc > 4095U) ? 4095U : ref_y_adc;
  apply_gate(gate_adc);
  s_cz.cool_us = (uint32_t)APP_CZ_INIT_COOL_US;
  s_cz.settle_us = 0U;
  s_cz.e_init = 0U;
  s_cz.hold_latched = 0U;
  s_cz.settled = 0U;
  s_cz.fire_n = 0U;
  s_cz.capped = 0U;
  s_cz.pending_learn = 0U;
  s_cz.last_axis = 0xFFU;
  s_cz.last_u_sign = 0;
  s_cz.e_pre = 0;
  s_cz.last_E = 0U;
  if (s_cz.ax.learned == 0U)
  {
    s_cz.ax.sens_q12 = (uint16_t)APP_CZ_SENS_Q12_INIT;
  }
  if (s_cz.ay.learned == 0U)
  {
    s_cz.ay.sens_q12 = (uint16_t)APP_CZ_SENS_Q12_INIT;
  }
  app_atom_abort();
  s_cz.enabled = 1U;
}

void app_cz_disable(void)
{
  s_cz.enabled = 0U;
  s_cz.cool_us = 0U;
  s_cz.settle_us = 0U;
  s_cz.hold_latched = 0U;
  s_cz.settled = 0U;
  s_cz.fire_n = 0U;
  s_cz.capped = 0U;
  s_cz.pending_learn = 0U;
  app_atom_abort();
}

int app_cz_enabled(void)
{
  return (s_cz.enabled != 0U) ? 1 : 0;
}

int app_cz_hold_active(void)
{
  return (s_cz.enabled != 0U && s_cz.hold_latched != 0U) ? 1 : 0;
}

int app_cz_settled(void)
{
  return (s_cz.enabled != 0U && s_cz.settled != 0U) ? 1 : 0;
}

void app_cz_set_invert(int inv_x, int inv_y)
{
  s_cz.ax.pol = (inv_x != 0) ? (int8_t)-1 : (int8_t)1;
  s_cz.ay.pol = (inv_y != 0) ? (int8_t)-1 : (int8_t)1;
}

static void learn_axis(AxisModel *m, int e_pre, int e_post, uint16_t E)
{
  const int ae0 = iabs(e_pre);
  const int ae1 = iabs(e_post);
  const int de = e_post - e_pre;
  const int progress = ae0 - ae1;

  if (E < 1U)
  {
    return;
  }

  /* Spoil fuerte: solo subir sens (menos energía). No invertir pol a ciegas. */
  if (progress < -2)
  {
    uint32_t s = ((uint32_t)m->sens_q12 * 5U) / 4U;
    m->sens_q12 = u16_clamp(s, (uint16_t)APP_CZ_SENS_Q12_MIN, (uint16_t)APP_CZ_SENS_Q12_MAX);
    /* Invertir solo si el error cambió de signo (pulso claramente al revés). */
    if ((e_pre > 0 && e_post < 0) || (e_pre < 0 && e_post > 0))
    {
      m->pol = (int8_t)(-m->pol);
    }
  }
  else if (progress > 0 && iabs(de) >= 1)
  {
    uint32_t meas = ((uint32_t)iabs(de) << 12) / (uint32_t)E;
    if (meas < (uint32_t)APP_CZ_SENS_Q12_MIN)
    {
      meas = (uint32_t)APP_CZ_SENS_Q12_MIN;
    }
    if (meas > (uint32_t)APP_CZ_SENS_Q12_MAX)
    {
      meas = (uint32_t)APP_CZ_SENS_Q12_MAX;
    }
    uint32_t s = ((uint32_t)m->sens_q12 * 3U + meas) >> 2;
    m->sens_q12 = u16_clamp(s, (uint16_t)APP_CZ_SENS_Q12_MIN, (uint16_t)APP_CZ_SENS_Q12_MAX);
  }
  m->learned = 1U;
}

static void size_pulse(const AxisModel *m, int ae, int gate,
                       unsigned *duty_out, uint32_t *ton_out, uint16_t *E_out)
{
  int excess = ae - gate;
  if (excess < 1)
  {
    excess = 1;
  }

  int target = (excess * (int)APP_CZ_STEP_NUM) / (int)APP_CZ_STEP_DEN;
  if (target < 1)
  {
    target = 1;
  }
  if (target > excess)
  {
    target = excess;
  }
  /* Cap duro: nunca pedir más de 2 LSB de corrección por pulso. */
  if (target > 2)
  {
    target = 2;
  }

  uint32_t sens = (uint32_t)m->sens_q12;
  if (sens < (uint32_t)APP_CZ_SENS_Q12_MIN)
  {
    sens = (uint32_t)APP_CZ_SENS_Q12_MIN;
  }

  uint32_t E = ((uint32_t)target << 12) / sens;
  if (E < 1U)
  {
    E = 1U;
  }

  uint32_t ton = (uint32_t)APP_CZ_TON_MIN_US + (E / 2U);
  if (ton > (uint32_t)APP_CZ_TON_MAX_US)
  {
    ton = (uint32_t)APP_CZ_TON_MAX_US;
  }
  uint32_t duty = (E * 64U) / ((ton > 0U) ? ton : 1U);
  if (duty < (uint32_t)APP_CZ_DUTY_MIN)
  {
    duty = (uint32_t)APP_CZ_DUTY_MIN;
  }
  if (duty > (uint32_t)APP_CZ_DUTY_MAX)
  {
    duty = (uint32_t)APP_CZ_DUTY_MAX;
  }

  uint32_t pred = (duty * (ton / 64U) * sens) >> 12;
  if (pred > (uint32_t)excess)
  {
    duty = (duty * (uint32_t)excess) / (pred + 1U);
    if (duty < (uint32_t)APP_CZ_DUTY_MIN)
    {
      duty = (uint32_t)APP_CZ_DUTY_MIN;
    }
  }

  E = duty * ((ton / 64U) > 0U ? (ton / 64U) : 1U);
  if (E < 1U)
  {
    E = 1U;
  }

  *duty_out = (unsigned)duty;
  *ton_out = ton;
  *E_out = (uint16_t)((E > 65535U) ? 65535U : E);
}

void app_cz_tick_fast(uint16_t x_adc, uint16_t y_adc, uint16_t dt_us)
{
  if (s_cz.enabled == 0U)
  {
    return;
  }
  if (dt_us < 1U)
  {
    dt_us = 1U;
  }

  if (s_cz.cool_us > 0U)
  {
    if (s_cz.cool_us > (uint32_t)dt_us)
    {
      s_cz.cool_us -= (uint32_t)dt_us;
    }
    else
    {
      s_cz.cool_us = 0U;
    }
  }

  const int ex = (int)s_cz.ref_x - (int)x_adc;
  const int ey = (int)s_cz.ref_y - (int)y_adc;

  if (s_cz.e_init == 0U)
  {
    s_cz.e_ema_x = ex;
    s_cz.e_ema_y = ey;
    s_cz.e_init = 1U;
  }
  else
  {
    s_cz.e_ema_x += (ex - s_cz.e_ema_x) >> 3;
    s_cz.e_ema_y += (ey - s_cz.e_ema_y) >> 3;
  }

  const int aex = iabs((int)s_cz.e_ema_x);
  const int aey = iabs((int)s_cz.e_ema_y);
  const int ae_max = (aex >= aey) ? aex : aey;

  if (s_cz.pending_learn != 0U && s_cz.cool_us == 0U && !app_atom_busy())
  {
    AxisModel *m = (s_cz.last_axis == 0U) ? &s_cz.ax : &s_cz.ay;
    const int e_now = (s_cz.last_axis == 0U) ? (int)s_cz.e_ema_x : (int)s_cz.e_ema_y;
    learn_axis(m, (int)s_cz.e_pre, e_now, s_cz.last_E);
    s_cz.pending_learn = 0U;
  }

  /*
   * ZONA QUIETA (≤ quiet LSB): cero átomos. Acumular permanencia → SETTLED.
   * Esto es lo que faltaba: a ±12–24 µm el bang-bang solo spoilaba.
   */
  if (ae_max <= (int)s_cz.quiet)
  {
    app_atom_abort();
    s_cz.hold_latched = 1U;
    s_cz.pending_learn = 0U;

    if (ae_max <= (int)s_cz.gate)
    {
      if (s_cz.settle_us < APP_CZ_SETTLE_US)
      {
        s_cz.settle_us += (uint32_t)dt_us;
        if (s_cz.settle_us > APP_CZ_SETTLE_US)
        {
          s_cz.settle_us = APP_CZ_SETTLE_US;
        }
      }
      if (s_cz.settle_us >= APP_CZ_SETTLE_US)
      {
        s_cz.settled = 1U;
      }
    }
    else
    {
      /* En quiet pero fuera de gate estricto: pausar timer, sin pulso. */
      s_cz.settle_us = 0U;
      s_cz.settled = 0U;
    }
    return;
  }

  /* Fuera de quiet: no estamos settled/hold. */
  s_cz.hold_latched = 0U;
  s_cz.settled = 0U;
  s_cz.settle_us = 0U;

  if (s_cz.capped != 0U || s_cz.cool_us > 0U || app_atom_busy() || s_cz.pending_learn != 0U)
  {
    return;
  }

  /* Banda intermedia quiet < ae < fire_min: esperar, no bang-bang. */
  if (ae_max < (int)s_cz.fire_min)
  {
    return;
  }

  AxisModel *m;
  MotorAxis axis;
  uint8_t axis_id;
  int err;
  int ae;
  if (aex >= aey)
  {
    m = &s_cz.ax;
    axis = MOTOR_A;
    axis_id = 0U;
    err = (int)s_cz.e_ema_x;
    ae = aex;
  }
  else
  {
    m = &s_cz.ay;
    axis = MOTOR_B;
    axis_id = 1U;
    err = (int)s_cz.e_ema_y;
    ae = aey;
  }

  unsigned duty = 0U;
  uint32_t ton = 0U;
  uint16_t E = 0U;
  size_pulse(m, ae, (int)s_cz.gate, &duty, &ton, &E);

  int e_sign = (err > 0) ? 1 : -1;
  int u_sign = (int)m->pol * e_sign;
  if (u_sign == 0)
  {
    u_sign = e_sign;
  }

  if (app_atom_fire_duty(axis, u_sign, duty, ton) != 0)
  {
    return;
  }

  s_cz.e_pre = (int16_t)err;
  s_cz.last_axis = axis_id;
  s_cz.last_u_sign = (int8_t)u_sign;
  s_cz.last_E = E;
  s_cz.pending_learn = 1U;
  s_cz.cool_us = (uint32_t)APP_CZ_OBSERVE_US + ton;

  if (s_cz.fire_n < 255U)
  {
    s_cz.fire_n++;
  }
  if (s_cz.fire_n >= (uint8_t)APP_CZ_MAX_FIRES)
  {
    s_cz.capped = 1U;
  }
}
