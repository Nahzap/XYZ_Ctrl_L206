/**
  ******************************************************************************
  * @file    app_control.c
  * @brief   Control modes (MANUAL/AUTO/BRAKE) and manual pot mapping.
  *
  *          Motor update runs in app_control_tick_1ms() (SysTick) so streaming
  *          serial telemetry cannot starve the L298N outputs.
  ******************************************************************************
  */

#include "app_control.h"
#include "app_motor.h"
#include "app_adc.h"
#include "app_atom.h"
#include "app_cz.h"

/* Arduino thresholds were on a 10-bit scale (0..1023): umbralBajo=337,
   umbralAlto=675. Rescaled to the STM32 12-bit ADC (0..4095): x4. */
#define UMBRAL_BAJO_12   (337 * 4)   /* 1348 */
#define UMBRAL_ALTO_12   (675 * 4)   /* 2700 */
#define ADC_FULL_SCALE   4095

/* Full Arduino-scale output (-255..255); minimum duty to overcome friction. */
#define POTENCIA_MAX_MANUAL 255
#define POTENCIA_MIN_MANUAL 90

static volatile ControlMode s_mode = CTRL_MANUAL;
static volatile int s_power_a = 0;
static volatile int s_power_b = 0;

static int map_i(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int get_manual_power(int v)
{
  if (v <= UMBRAL_BAJO_12)
  {
    int p = map_i(v, UMBRAL_BAJO_12, 0, POTENCIA_MIN_MANUAL, POTENCIA_MAX_MANUAL);
    return -p;
  }
  else if (v <= UMBRAL_ALTO_12)
  {
    return 0;
  }
  else
  {
    return map_i(v, UMBRAL_ALTO_12, ADC_FULL_SCALE, POTENCIA_MIN_MANUAL, POTENCIA_MAX_MANUAL);
  }
}

static void control_apply_motors(void)
{
  if (s_mode == CTRL_BRAKE)
  {
    app_motor_brake(MOTOR_A);
    app_motor_brake(MOTOR_B);
    return;
  }

  app_motor_set(MOTOR_A, s_power_a);
  app_motor_set(MOTOR_B, s_power_b);
}

void app_control_init(void)
{
  s_mode = CTRL_MANUAL;
  s_power_a = 0;
  s_power_b = 0;
  app_atom_init();
  app_cz_init();
  app_motor_set(MOTOR_A, 0);
  app_motor_set(MOTOR_B, 0);
}

void app_control_tick_1ms(void)
{
  if (s_mode == CTRL_MANUAL)
  {
    uint16_t pot_a = 0;
    uint16_t pot_b = 0;
    app_adc_read_pots(&pot_a, &pot_b);
    s_power_a = get_manual_power((int)pot_a);
    s_power_b = get_manual_power((int)pot_b);
  }
  else if (s_mode == CTRL_BRAKE)
  {
    s_power_a = 0;
    s_power_b = 0;
  }

  /* Coarse @ 1 kHz. No pisar el eje mientras el átomo corre @ 1 µs. */
  if (!app_atom_busy())
  {
    control_apply_motors();
  }
}

void app_control_step(void)
{
  /* Motors handled in app_control_tick_1ms(); keep for main-loop compatibility. */
}

void app_control_set_manual(void)
{
  app_cz_disable();
  s_mode = CTRL_MANUAL;
}

void app_control_set_auto(int power_a, int power_b)
{
  /* Coarse no nulo → host retoma. A,0,0 no apaga C(z) (FOV observa). */
  if (power_a != 0 || power_b != 0)
  {
    app_cz_disable();
  }
  s_mode = CTRL_AUTO;
  s_power_a = power_a;
  s_power_b = power_b;
}

void app_control_set_brake(void)
{
  app_cz_disable();
  app_atom_abort();
  s_mode = CTRL_BRAKE;
  s_power_a = 0;
  s_power_b = 0;
}

int app_control_power_a(void)
{
  return s_power_a;
}

int app_control_power_b(void)
{
  return s_power_b;
}

const char *app_control_state_str(void)
{
  if (app_atom_busy())
  {
    return "PULSE";
  }
  if (app_cz_enabled())
  {
    return "FINE";
  }
  switch (s_mode)
  {
    case CTRL_AUTO:  return "AUTO";
    case CTRL_BRAKE: return "BRAKE";
    case CTRL_MANUAL:
    default:         return "MANUAL";
  }
}

int app_control_settled(void)
{
  if (s_mode == CTRL_BRAKE)
  {
    return 1;
  }
  return (s_power_a == 0 && s_power_b == 0) ? 1 : 0;
}
