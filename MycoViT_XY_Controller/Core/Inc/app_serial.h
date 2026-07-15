/**
  ******************************************************************************
  * @file    app_serial.h
  * @brief   USART3 command RX (interrupt) + parser.
  *          Commands: M (MANUAL), A,<a>,<b> (AUTO), B (BRAKE),
  *                    P,<axis>,<sign>,<idx> (mini-pulso fine — Fase 3).
  ******************************************************************************
  */

#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Enable USART3 RXNE interrupt. */
void app_serial_init(void);

/* Call from USART3_IRQHandler: consumes one received byte into the line buffer. */
void app_serial_isr(void);

/* Call from the main loop: if a full line arrived, parse and apply it. */
void app_serial_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SERIAL_H */
