/**
  ******************************************************************************
  * @file    app_rt.c
  * @brief   Real-time acquisition/control backbone @ 1 MHz.
  *
  *          Sensores: PC3/ADC1 IN13 (Y), PA3/ADC2 IN3 (X). TIM6 TRGO @ 1 MHz.
  *          ISR en EOC de ADC1; lectura de ADC2 solo tras flag EOC (linealidad).
  *          Sampling 15 ciclos (no 3) para fuentes de alta impedancia (pot/láser).
  *          TIM2 (trigger legacy de CubeMX para ADC2) se mantiene apagado.
  *
  *          EMA en ISR: y_f += (y - y_f) >> N  (enteros, 1-2 ciclos de ALU).
  *          N=5 → α=1/32, τ≈32 µs @ 1 MHz. Cambiar RT_EMA_SHIFT a 3 si se
  *          prefiere más respuesta (τ≈8 µs).
  ******************************************************************************
  */

#include "app_rt.h"
#include "app_atom.h"
#include "main.h"
#include "tim.h"

/* Sensor channels (see adc.c): Y_VAL = ADC1 IN13, X_VAL = ADC2 IN3. */
#define RT_ADC_Y_CHANNEL  LL_ADC_CHANNEL_13
#define RT_ADC_X_CHANNEL  LL_ADC_CHANNEL_3

/* 15 ADC cycles @ 27 MHz + 12-bit conversion = 1.0 us, fits the 1 MHz tick.
   3 cycles was too short for high-Z sources (pot): nonlinear gain. */
#define RT_ADC_SAMPLE_TIME  LL_ADC_SAMPLINGTIME_15CYCLES

#define RT_ADC2_EOC_SPIN_MAX  200U

/* EMA shift N∈{3,5}: α = 1/2^N. Default 5 (más suave para telemetría). */
#ifndef RT_EMA_SHIFT
#define RT_EMA_SHIFT  5U
#endif

static volatile uint16_t s_adc_x = 0;
static volatile uint16_t s_adc_y = 0;
/* Q0 fixed-point EMA state (same units as ADC). Updated every ISR tick. */
static volatile int32_t s_ema_x = 0;
static volatile int32_t s_ema_y = 0;
static volatile uint8_t s_ema_init = 0U;
static volatile uint32_t s_ticks = 0;

static void rt_adc_fast_init(void)
{
  /* Re-assert analog mode (PA3/PC3) after all CubeMX inits. */
  LL_GPIO_InitTypeDef gpio = {0};
  gpio.Mode = LL_GPIO_MODE_ANALOG;
  gpio.Pull = LL_GPIO_PULL_NO;

  gpio.Pin = Y_VAL_Pin;
  LL_GPIO_Init(Y_VAL_GPIO_Port, &gpio);
  gpio.Pin = X_VAL_Pin;
  LL_GPIO_Init(X_VAL_GPIO_Port, &gpio);

  /* CubeMX originally wired ADC2 to TIM2 TRGO; keep TIM2 off so only TIM6 fires. */
  LL_TIM_DisableCounter(TIM2);

  /* Both ADCs: no DMA, triggered by TIM6 TRGO (synchronous). */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
  LL_ADC_SetChannelSamplingTime(ADC1, RT_ADC_Y_CHANNEL, RT_ADC_SAMPLE_TIME);
  LL_ADC_Enable(ADC1);

  LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_NONE);
  LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
  LL_ADC_SetChannelSamplingTime(ADC2, RT_ADC_X_CHANNEL, RT_ADC_SAMPLE_TIME);
  LL_ADC_Enable(ADC2);

  for (volatile uint32_t i = 0U; i < 10000U; i++)
  {
    __NOP();
  }

  /* ADC1 EOC drives the 1 MHz control tick (ADC1/2/3 share ADC_IRQn). */
  LL_ADC_ClearFlag_EOCS(ADC1);
  LL_ADC_EnableIT_EOCS(ADC1);
  NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(ADC_IRQn);

  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
}

void app_rt_init(void)
{
  /* CubeMX left TIM6 as PSC=107, ARR=0 -> the counter never advances (CNT stuck
     at 0) and no usable TRGO is emitted. Use a real counting base instead:
     108 MHz / (0+1) / (107+1) = 1 MHz update -> TRGO at 1 MHz. */
  LL_TIM_SetPrescaler(TIM6, 0);
  LL_TIM_SetAutoReload(TIM6, 107);
  LL_TIM_GenerateEvent_UPDATE(TIM6);   /* load shadow ARR immediately */
  LL_TIM_ClearFlag_UPDATE(TIM6);

  rt_adc_fast_init();

  LL_TIM_EnableCounter(TIM6);
}

void app_rt_isr(void)
{
  if (LL_ADC_IsActiveFlag_EOCS(ADC1) == 0U)
  {
    return;
  }

  /* Reading each data register clears its EOC flag. */
  uint16_t y = LL_ADC_REG_ReadConversionData12(ADC1);

  /* ADC2 can finish slightly after ADC1; never read DR before EOC (garbage /
     compressed range with high-Z sources). */
  uint32_t spin = 0U;
  while ((LL_ADC_IsActiveFlag_EOCS(ADC2) == 0U) && (spin < RT_ADC2_EOC_SPIN_MAX))
  {
    spin++;
    __NOP();
  }
  uint16_t x = (spin < RT_ADC2_EOC_SPIN_MAX)
               ? LL_ADC_REG_ReadConversionData12(ADC2)
               : s_adc_x;
  s_adc_y = y;
  s_adc_x = x;

  /* EMA: first sample seeds the filter (no ramp-from-zero). Afterwards one
     shift-add per channel — cheap enough for the 1 MHz ISR budget. */
  if (s_ema_init == 0U)
  {
    s_ema_x = (int32_t)x;
    s_ema_y = (int32_t)y;
    s_ema_init = 1U;
  }
  else
  {
    s_ema_x += ((int32_t)x - s_ema_x) >> RT_EMA_SHIFT;
    s_ema_y += ((int32_t)y - s_ema_y) >> RT_EMA_SHIFT;
  }

  s_ticks++;

  app_rt_control_hook(x, y);
  /* Pulso LUT avanza en el mismo cuanto RT (1 µs), no en SysTick 1 ms. */
  app_atom_tick_1us();

  /* Defensive: if a tick was ever missed, drop the stale-data condition. */
  LL_ADC_ClearFlag_OVR(ADC1);
  LL_ADC_ClearFlag_OVR(ADC2);
}

uint16_t app_rt_get_x(void)
{
  return s_adc_x;
}

uint16_t app_rt_get_y(void)
{
  return s_adc_y;
}

uint16_t app_rt_get_x_filtered(void)
{
  return (uint16_t)s_ema_x;
}

uint16_t app_rt_get_y_filtered(void)
{
  return (uint16_t)s_ema_y;
}

uint32_t app_rt_tick_count(void)
{
  return s_ticks;
}

/* Fallback si app_cz.c no está linkeado. La ley real vive en app_cz.c. */
__attribute__((weak)) void app_rt_control_hook(uint16_t x, uint16_t y)
{
  (void)x;
  (void)y;
}
