/**
  ******************************************************************************
  * @file    app_rt.c
  * @brief   ADC sync TIM2 + EMA soft-clamp + dwell filter.
  ******************************************************************************
  */

#include "app_rt.h"
#include "app_atom.h"
#include "app_cz.h"
#include "main.h"
#include "tim.h"

#include "stm32f7xx.h"

#define RT_ADC_Y_CHANNEL    LL_ADC_CHANNEL_13
#define RT_ADC_X_CHANNEL    LL_ADC_CHANNEL_3
#define RT_ADC_SAMPLE_TIME  LL_ADC_SAMPLINGTIME_480CYCLES
#define RT_HALF             (APP_RT_DMA_BLK / 2U)
#define RT_DCACHE_LINE      32U
#define RT_STEP_CLAMP_ADC   20

static uint16_t s_dma_y[APP_RT_DMA_BLK] __attribute__((aligned(32)));
static uint16_t s_dma_x[APP_RT_DMA_BLK] __attribute__((aligned(32)));

static volatile uint16_t s_adc_x = 0;
static volatile uint16_t s_adc_y = 0;
static volatile int32_t  s_ema_x = 0;
static volatile int32_t  s_ema_y = 0;
static volatile int32_t  s_dwell_x = 0;
static volatile int32_t  s_dwell_y = 0;
static volatile uint8_t  s_ema_init = 0U;
static volatile uint32_t s_ticks = 0;

static void dcache_invalidate_range(const void *addr, uint32_t nbytes)
{
  uint32_t start = ((uint32_t)addr) & ~(RT_DCACHE_LINE - 1U);
  uint32_t end = ((uint32_t)addr + nbytes + RT_DCACHE_LINE - 1U) & ~(RT_DCACHE_LINE - 1U);
  SCB_InvalidateDCache_by_Addr((void *)start, (int32_t)(end - start));
}

static uint16_t trimmed_mean_half(const uint16_t *buf, unsigned off)
{
  uint32_t sum = 0U;
  uint16_t vmin = 4095U;
  uint16_t vmax = 0U;
  unsigned i;
  for (i = 0U; i < RT_HALF; i++)
  {
    const uint16_t v = buf[off + i];
    sum += (uint32_t)v;
    if (v < vmin)
    {
      vmin = v;
    }
    if (v > vmax)
    {
      vmax = v;
    }
  }
  sum -= (uint32_t)vmin;
  sum -= (uint32_t)vmax;
  return (uint16_t)(sum / (RT_HALF - 2U));
}

static void ema_soft_step(volatile int32_t *ema, uint16_t sample, int shift)
{
  int32_t cur = *ema;
  int32_t x = (int32_t)sample;
  int32_t d = x - cur;
  if (d > (int32_t)RT_STEP_CLAMP_ADC)
  {
    d = (int32_t)RT_STEP_CLAMP_ADC;
  }
  else if (d < -(int32_t)RT_STEP_CLAMP_ADC)
  {
    d = -(int32_t)RT_STEP_CLAMP_ADC;
  }
  *ema = cur + (d >> shift);
}

static void rt_process_half(unsigned off)
{
  const uint32_t bytes = RT_HALF * sizeof(uint16_t);
  dcache_invalidate_range(&s_dma_y[off], bytes);
  dcache_invalidate_range(&s_dma_x[off], bytes);

  const uint16_t y = trimmed_mean_half(s_dma_y, off);
  const uint16_t x = trimmed_mean_half(s_dma_x, off);
  s_adc_y = y;
  s_adc_x = x;

  if (s_ema_init == 0U)
  {
    s_ema_x = (int32_t)x;
    s_ema_y = (int32_t)y;
    s_dwell_x = (int32_t)x;
    s_dwell_y = (int32_t)y;
    s_ema_init = 1U;
  }
  else
  {
    ema_soft_step(&s_ema_x, x, 2);
    ema_soft_step(&s_ema_y, y, 2);
    ema_soft_step(&s_dwell_x, x, 4);
    ema_soft_step(&s_dwell_y, y, 4);
  }

  s_ticks += RT_HALF;
  app_cz_tick_fast((uint16_t)s_dwell_x, (uint16_t)s_dwell_y, APP_RT_CZ_PERIOD_US);
}

static void rt_arm_dma_stream(uint32_t stream, uint32_t channel,
                              uint32_t periph, uint32_t *mem, uint32_t n)
{
  LL_DMA_DisableStream(DMA2, stream);
  while (LL_DMA_IsEnabledStream(DMA2, stream) != 0U)
  {
  }

  LL_DMA_SetChannelSelection(DMA2, stream, channel);
  LL_DMA_SetDataTransferDirection(DMA2, stream, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA2, stream, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA2, stream, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA2, stream, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, stream, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, stream, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA2, stream, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_DisableFifoMode(DMA2, stream);

  LL_DMA_ConfigAddresses(DMA2, stream, periph, (uint32_t)mem,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA2, stream, n);
}

static void rt_tim2_adc_trigger_init(void)
{
  LL_TIM_DisableCounter(TIM2);
  LL_TIM_SetPrescaler(TIM2, 107);
  LL_TIM_SetAutoReload(TIM2, 19);
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_GenerateEvent_UPDATE(TIM2);
  LL_TIM_ClearFlag_UPDATE(TIM2);
}

static void rt_adc_dma_init(void)
{
  LL_GPIO_InitTypeDef gpio = {0};
  gpio.Mode = LL_GPIO_MODE_ANALOG;
  gpio.Pull = LL_GPIO_PULL_NO;
  gpio.Pin = Y_VAL_Pin;
  LL_GPIO_Init(Y_VAL_GPIO_Port, &gpio);
  gpio.Pin = X_VAL_Pin;
  LL_GPIO_Init(X_VAL_GPIO_Port, &gpio);

  LL_ADC_DisableIT_EOCS(ADC1);
  NVIC_DisableIRQ(ADC_IRQn);

  LL_ADC_REG_StopConversionExtTrig(ADC1);
  LL_ADC_REG_StopConversionExtTrig(ADC2);

  rt_tim2_adc_trigger_init();

  rt_arm_dma_stream(LL_DMA_STREAM_0, LL_DMA_CHANNEL_0,
                    LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                    (uint32_t *)s_dma_y, APP_RT_DMA_BLK);
  rt_arm_dma_stream(LL_DMA_STREAM_2, LL_DMA_CHANNEL_1,
                    LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA),
                    (uint32_t *)s_dma_x, APP_RT_DMA_BLK);

  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);

  NVIC_SetPriority(DMA2_Stream0_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
  LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
  LL_ADC_SetChannelSamplingTime(ADC1, RT_ADC_Y_CHANNEL, RT_ADC_SAMPLE_TIME);
  LL_ADC_SetChannelSamplingTime(ADC2, RT_ADC_X_CHANNEL, RT_ADC_SAMPLE_TIME);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  for (volatile uint32_t i = 0U; i < 10000U; i++)
  {
    __NOP();
  }

  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);

  LL_TIM_EnableCounter(TIM2);
}

void app_rt_init(void)
{
  LL_TIM_SetPrescaler(TIM6, 0);
  LL_TIM_SetAutoReload(TIM6, 107);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_GenerateEvent_UPDATE(TIM6);
  LL_TIM_ClearFlag_UPDATE(TIM6);

  rt_adc_dma_init();

  LL_TIM_EnableIT_UPDATE(TIM6);
  NVIC_SetPriority(TIM6_DAC_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  LL_TIM_EnableCounter(TIM6);
}

void app_rt_tim6_isr(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0U)
  {
    LL_TIM_ClearFlag_UPDATE(TIM6);
    app_atom_tick_1us();
  }
}

void app_rt_dma_adc1_isr(void)
{
  if (LL_DMA_IsActiveFlag_TE0(DMA2) != 0U)
  {
    LL_DMA_ClearFlag_TE0(DMA2);
  }
  if (LL_DMA_IsActiveFlag_HT0(DMA2) != 0U)
  {
    LL_DMA_ClearFlag_HT0(DMA2);
    rt_process_half(0U);
  }
  if (LL_DMA_IsActiveFlag_TC0(DMA2) != 0U)
  {
    LL_DMA_ClearFlag_TC0(DMA2);
    rt_process_half(RT_HALF);
  }
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
  return (uint16_t)s_dwell_x;
}

uint16_t app_rt_get_y_filtered(void)
{
  return (uint16_t)s_dwell_y;
}

uint16_t app_rt_get_x_dwell(void)
{
  return (uint16_t)s_dwell_x;
}

uint16_t app_rt_get_y_dwell(void)
{
  return (uint16_t)s_dwell_y;
}

uint16_t app_rt_get_x_fast(void)
{
  return (uint16_t)s_ema_x;
}

uint16_t app_rt_get_y_fast(void)
{
  return (uint16_t)s_ema_y;
}

uint32_t app_rt_tick_count(void)
{
  return s_ticks;
}
