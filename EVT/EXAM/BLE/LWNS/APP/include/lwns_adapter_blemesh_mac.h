/* ********************************* (C) COPYRIGHT ***************************
 * File Name: lwns_adapter_blemesh_mac.h
 * Author: WCH
 * Version: V1.0
 * Date: 2021/06/20
 * Description: lwns adapter, emulates ble sig mesh's mac protocol
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */
#ifndef _LWNS_ADAPTER_BLEMESH_MAC_H_
#define _LWNS_ADAPTER_BLEMESH_MAC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lwns_config.h"

#define LWNS_USE_BLEMESH_MAC    1  // Whether to enable the mac protocol that mimics blemesh, please note that only one mac layer protocol can be enabled.

#if LWNS_USE_BLEMESH_MAC

struct blemesh_mac_phy_manage_struct
{
    struct blemesh_mac_phy_manage_struct *next;
    uint8_t                              *data;
}; // Imitate the blemesh mac layer sending management structure

  #define LWNS_MAC_TRANSMIT_TIMES           2     // Send once, call hardware to send several times

  #define LWNS_MAC_PERIOD_MS                10    // Mac receiving cycle, switch in turn

  #define LWNS_MAC_SEND_DELAY_MAX_MS        10    // Bluetooth mesh is a random number within 10ms.

  #define LWNS_MAC_SEND_PACKET_MAX_NUM      8     // The sending linked list supports up to several packets waiting for sending

  #define BLE_PHY_ONE_PACKET_MAX_625US      5     // ble mac sends a packet at the maximum possible period

  #define LLE_MODE_ORIGINAL_RX              (0x80) // If this macro is added when configuring LLEMODE, the first byte is received as the original data (originally RSSI)

  #define LWNS_HTIMER_PERIOD_MS             20    // ä¸º(1000/HTIMER_SECOND_NUM)

    // The types used by RF_TX and RF_RX can be modified, but it is not recommended to change them.
  #define USER_RF_RX_TX_TYPE                0xff

  #define LWNS_PHY_OUTPUT_TIMEOUT_MS        5

    //receive process evt
  #define LWNS_PHY_RX_OPEN_EVT              1
  #define LWNS_PHY_RX_CHANGE_CHANNEL_EVT    2
    //send process evt
  #define LWNS_HTIMER_PERIOD_EVT            1
  #define LWNS_PHY_OUTPUT_PREPARE_EVT       2
  #define LWNS_PHY_OUTPUT_FINISH_EVT        4

extern void RF_Init(void);

extern void lwns_init(void);

extern void lwns_shut(void);

extern void lwns_start(void);

  #ifdef __cplusplus
}
  #endif

#endif /* LWNS_USE_BLEMESH_MAC */

#endif /* _LWNS_ADAPTER_BLEMESH_MAC_H_ */
