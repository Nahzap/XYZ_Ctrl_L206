/**
  ******************************************************************************
  * @file    app_serial.c
  * @brief   USART3 command RX (interrupt) + parser for the Arduino protocol.
  ******************************************************************************
  */

#include "app_serial.h"
#include "app_control.h"
#include "app_atom.h"
#include "app_cz.h"
#include "main.h"

#include <string.h>
#include <stdlib.h>

#define CMD_BUFFER_SIZE 50

/* Filled by the ISR up to '\n'. */
static char    s_rx_buf[CMD_BUFFER_SIZE];
static uint8_t s_rx_pos = 0;

/* Snapshot of a completed line, consumed by the main loop. */
static char             s_cmd[CMD_BUFFER_SIZE];
static volatile uint8_t s_cmd_ready = 0;

void app_serial_init(void)
{
  LL_USART_EnableIT_RXNE(USART3);
}

void app_serial_isr(void)
{
  if (LL_USART_IsActiveFlag_RXNE(USART3) == 0U)
  {
    return;
  }

  char c = (char)LL_USART_ReceiveData8(USART3);

  if (c == '\r')
  {
    return; /* tolerate CRLF line endings from terminals */
  }

  if (c == '\n')
  {
    s_rx_buf[s_rx_pos] = '\0';
    if (s_cmd_ready == 0U)
    {
      memcpy(s_cmd, s_rx_buf, (size_t)s_rx_pos + 1U);
      s_cmd_ready = 1U;
    }
    s_rx_pos = 0;
  }
  else if (s_rx_pos < (CMD_BUFFER_SIZE - 1U))
  {
    s_rx_buf[s_rx_pos++] = c;
  }
}

void app_serial_process(void)
{
  if (s_cmd_ready == 0U)
  {
    return;
  }

  char line[CMD_BUFFER_SIZE];
  memcpy(line, s_cmd, sizeof(line));
  s_cmd_ready = 0U;

  if (strcmp(line, "M") == 0 || strcmp(line, "m") == 0)
  {
    app_control_set_manual();
  }
  else if (strcmp(line, "N") == 0 || strcmp(line, "n") == 0)
  {
    /* Fine OFF sin freno: apaga C(z), deja AUTO 0,0 (no spoil por B). */
    app_cz_disable();
    app_atom_abort();
    app_control_set_auto(0, 0);
  }
  else if (strcmp(line, "B") == 0 || strcmp(line, "b") == 0)
  {
    app_control_set_brake();
  }
  else if (strncmp(line, "A,", 2) == 0 || strncmp(line, "a,", 2) == 0)
  {
    int power_a = 0;
    int power_b = 0;
    char *token = strtok(line, ",");   /* "A" */
    token = strtok(NULL, ",");         /* power_a */
    if (token != NULL)
    {
      power_a = atoi(token);
    }
    token = strtok(NULL, ",");         /* power_b */
    if (token != NULL)
    {
      power_b = atoi(token);
    }
    app_control_set_auto(power_a, power_b);
  }
  else if (strncmp(line, "F,", 2) == 0 || strncmp(line, "f,", 2) == 0)
  {
    /* F,<ref_x_adc>,<ref_y_adc> — C(z) fine local @ 1 MHz / LUT */
    int ref_x = 0;
    int ref_y = 0;
    char *token = strtok(line, ",");   /* "F" */
    token = strtok(NULL, ",");
    if (token != NULL)
    {
      ref_x = atoi(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL)
    {
      ref_y = atoi(token);
    }
    if (ref_x < 0)
    {
      ref_x = 0;
    }
    if (ref_y < 0)
    {
      ref_y = 0;
    }
    if (ref_x > 4095)
    {
      ref_x = 4095;
    }
    if (ref_y > 4095)
    {
      ref_y = 4095;
    }
    app_control_set_auto(0, 0);
    app_cz_enable((uint16_t)ref_x, (uint16_t)ref_y);
  }
  else if (strncmp(line, "I,", 2) == 0 || strncmp(line, "i,", 2) == 0)
  {
    /* I,<inv_x>,<inv_y> — polaridad error→motor para C(z) */
    int inv_x = 0;
    int inv_y = 0;
    char *token = strtok(line, ",");
    token = strtok(NULL, ",");
    if (token != NULL)
    {
      inv_x = atoi(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL)
    {
      inv_y = atoi(token);
    }
    app_cz_set_invert(inv_x, inv_y);
  }
  else if (strncmp(line, "P,", 2) == 0 || strncmp(line, "p,", 2) == 0)
  {
    /* P,<axis>,<sign>,<idx>  A|X|0 = MOTOR_A (eje X); B|Y|1 = MOTOR_B (eje Y) */
    app_cz_disable();
    char *token = strtok(line, ",");   /* "P" */
    token = strtok(NULL, ",");         /* axis */
    if (token == NULL)
    {
      return;
    }
    MotorAxis axis = MOTOR_A;
    char ax = token[0];
    if (ax >= 'a' && ax <= 'z')
    {
      ax = (char)(ax - 32);
    }
    if (ax == 'B' || ax == '1' || ax == 'Y')
    {
      axis = MOTOR_B;
    }
    else if (ax == 'A' || ax == '0' || ax == 'X')
    {
      axis = MOTOR_A;
    }
    else
    {
      return;
    }

    token = strtok(NULL, ",");         /* sign */
    if (token == NULL)
    {
      return;
    }
    int sign = atoi(token);
    if (sign == 0)
    {
      return;
    }

    token = strtok(NULL, ",");         /* idx */
    if (token == NULL)
    {
      return;
    }
    unsigned idx = (unsigned)atoi(token);

    /* Fine: salir de MANUAL para que los pots no peleen tras el pulso. */
    app_control_set_auto(0, 0);
    (void)app_atom_fire(axis, sign, idx);
  }
}
