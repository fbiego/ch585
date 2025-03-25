/* ********************************* (C) COPYRIGHT ***************************
 * File Name: lwns_adapter_no_mac.h
 * Author: WCH
 * Version: V1.0
 * Date: 2021/06/20
 * Description: lwns adapter, does not use the mac protocol, pure transparent transmission
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */
#ifndef _LWNS_ADAPTER_NO_MAC_H_
#define _LWNS_ADAPTER_NO_MAC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lwns_config.h"

#define LWNS_USE_NO_MAC    0  // Whether to enable the pure transmissive mac protocol, it is suitable for star networks where there is no concurrency, that is, star networks where the host asks and slaves asks.

#if LWNS_USE_NO_MAC

typedef enum
{
    BLE_PHY_MANAGE_STATE_FREE = 0,
    BLE_PHY_MANAGE_STATE_SENDING,
} BLE_PHY_MANAGE_STATE_t;

  #define LLE_MODE_ORIGINAL_RX          (0x80) // If this macro is added when configuring LLEMODE, the first byte is received as the original data (originally RSSI)

  #define LWNS_HTIMER_PERIOD_MS         20    // ä¸º(1000/HTIMER_SECOND_NUM)

    // The types used by RF_TX and RF_RX can be modified, but it is not recommended to change them.
  #define USER_RF_RX_TX_TYPE            0xff

  #define LWNS_PHY_OUTPUT_TIMEOUT_MS    5

    //receive process evt
  #define LWNS_PHY_RX_OPEN_EVT          1
    //send process evt
  #define LWNS_HTIMER_PERIOD_EVT        1
  #define LWNS_PHY_OUTPUT_FINISH_EVT    2

extern void RF_Init(void);

extern void lwns_init(void);

extern void lwns_shut(void);

extern void lwns_start(void);

#endif /* LWNS_USE_NO_MAC */

#ifdef __cplusplus
}
#endif

#endif /* _LWNS_ADAPTER_NO_MAC_H_ */
