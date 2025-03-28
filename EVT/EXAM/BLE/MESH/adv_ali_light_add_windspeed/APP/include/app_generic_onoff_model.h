/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_generic_onoff_model.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/11/12
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef app_generic_onoff_model_H
#define app_generic_onoff_model_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MESH_LIB.h"
#define MSG_PIN    GPIO_Pin_12

extern const struct bt_mesh_model_op gen_onoff_op[];

/* *
 * @brief Get the current light status.
 *
 * @param led_pin - pin.
 *
 * @return Light status */
BOOL read_led_state(uint32_t led_pin);

/* *
 * @brief Sets the current light status.
 *
 * @param led_pin - pin.
 * @param on - Status. */
void set_led_state(uint32_t led_pin, BOOL on);

/* *
 * @brief Flip the current light status
 *
 * @param led_pin - pin. */
void toggle_led_state(uint32_t led_pin);

#ifdef __cplusplus
}
#endif

#endif
