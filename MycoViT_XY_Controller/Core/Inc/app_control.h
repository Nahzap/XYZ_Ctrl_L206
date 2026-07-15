/**
  ******************************************************************************
  * @file    app_control.h
  * @brief   Control modes (MANUAL/AUTO) and manual pot mapping.
  *          Port of the Arduino currentMode / getManualPower logic to 12-bit.
  ******************************************************************************
  */

#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  CTRL_MANUAL = 0,
  CTRL_AUTO   = 1,
  CTRL_BRAKE  = 2
} ControlMode;

/* Boot state: MANUAL, both powers 0, motors forced to 0. */
void app_control_init(void);

/* One control iteration (legacy main-loop call). Motor actuation runs in
   app_control_tick_1ms() from SysTick so it is not starved by serial TX. */
void app_control_step(void);

/* 1 ms tick: read pots (MANUAL), apply motor powers. Called from SysTick. */
void app_control_tick_1ms(void);

/* Commands from the serial parser. */
void app_control_set_manual(void);
void app_control_set_auto(int power_a, int power_b);
void app_control_set_brake(void);

/* Telemetry accessors (last applied signed powers). */
int app_control_power_a(void);
int app_control_power_b(void);

/* Telemetry: state label ("MANUAL"/"AUTO"/"BRAKE") and at-rest flag (0/1). */
const char *app_control_state_str(void);
int app_control_settled(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONTROL_H */
