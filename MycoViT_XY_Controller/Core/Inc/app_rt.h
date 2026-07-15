/**
  ******************************************************************************
  * @file    app_rt.h
  * @brief   Real-time acquisition/control backbone @ 1 MHz.
  *
 *          TIM6 TRGO triggers ADC1 (Y, PC3/IN13) and ADC2 (X, PA3/IN3) at 1 MHz.
 *          The ADC end-of-conversion interrupt is the 1 MHz control tick: it
 *          captures both samples (ADC2 read waits for EOC) and calls
 *          app_rt_control_hook() (where C(z) will live). Sampling time is
 *          15 ADC cycles. No DMA — no D-cache coherency issues.
  ******************************************************************************
  */

#ifndef __APP_RT_H
#define __APP_RT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Target sampling/control rate. Kept for reference/telemetry. */
#define APP_RT_HZ 1000000UL

/* Configure ADC1/ADC2 for TIM6-triggered 1 MHz sampling, the ADC EOC
   interrupt, then start the TIM6 time base. Call after MX_* init. */
void app_rt_init(void);

/* Called from ADC_IRQHandler at 1 MHz. */
void app_rt_isr(void);

/* Latest RAW samples captured by the 1 MHz ISR (12-bit, 0..4095). */
uint16_t app_rt_get_x(void);
uint16_t app_rt_get_y(void);

/* EMA-filtered samples (same domain). Preferred for telemetry / UI. */
uint16_t app_rt_get_x_filtered(void);
uint16_t app_rt_get_y_filtered(void);

/* Total ISR ticks since boot (for measuring the effective rate). */
uint32_t app_rt_tick_count(void);

/* Control law hook @ 1 MHz ISR. Strong impl in app_cz.c (C(z) → LUT).
   Keep budget < ~1 us (enteros; actuación decimada). */
void app_rt_control_hook(uint16_t x, uint16_t y);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RT_H */
