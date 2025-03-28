/********************************** (C) COPYRIGHT *******************************
 * File Name          : app.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/11/12
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef app_H
#define app_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

#define APP_SILENT_ADV_EVT    (1 << 0)

/******************************************************************************/

/* *
 * @brief Application layer initialization */
void App_Init(void);

/* *
 * @brief Send the current light status to Tmall Elf */
void send_led_state(void);

/* *
 * @brief Send a reset event to Tmall Genie. After the sending is completed, the distribution network status will be cleared and the self-mesh network will be reset. */
void send_reset_indicate(void);

/******************************************************************************/

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
