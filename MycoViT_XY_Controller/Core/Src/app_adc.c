/**
  ******************************************************************************
  * @file    app_adc.c
  * @brief   Application ADC reads (polling, software trigger, 12-bit).
  ******************************************************************************
  */

#include "app_adc.h"
#include "main.h"
#include <stddef.h>

#define ADC3_SPIN_MAX  100000U

static void adc_settle(void)
{
  for (volatile uint32_t i = 0U; i < 10000U; i++)
  {
    __NOP();
  }
}

void app_adc_init(void)
{
  /* Override CubeMX scan+DMA: single-channel SW reads per pot (PF5/PC0). */
  LL_ADC_REG_SetDMATransfer(ADC3, LL_ADC_REG_DMA_TRANSFER_NONE);
  LL_ADC_REG_SetTriggerSource(ADC3, LL_ADC_REG_TRIG_SOFTWARE);
  LL_ADC_REG_SetSequencerLength(ADC3, LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_REG_SetContinuousMode(ADC3, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_15CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_15CYCLES);
  LL_ADC_Enable(ADC3);

  adc_settle();
}

static uint16_t adc3_read_channel(uint32_t channel)
{
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, channel);
  LL_ADC_ClearFlag_EOCS(ADC3);
  LL_ADC_REG_StartConversionSWStart(ADC3);

  uint32_t spin = 0U;
  while ((LL_ADC_IsActiveFlag_EOCS(ADC3) == 0U) && (spin < ADC3_SPIN_MAX))
  {
    spin++;
  }

  uint16_t value = LL_ADC_REG_ReadConversionData12(ADC3);
  LL_ADC_ClearFlag_EOCS(ADC3);
  return value;
}

void app_adc_read_pots(uint16_t *pot_a, uint16_t *pot_b)
{
  if (pot_a != NULL)
  {
    *pot_a = adc3_read_channel(LL_ADC_CHANNEL_15); /* PF5 POT_A */
  }
  if (pot_b != NULL)
  {
    *pot_b = adc3_read_channel(LL_ADC_CHANNEL_10); /* PC0 POT_B */
  }
}
