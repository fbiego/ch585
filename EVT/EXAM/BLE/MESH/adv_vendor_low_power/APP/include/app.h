/********************************** (C) COPYRIGHT *******************************
 * File Name          : app.h
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2021/11/18
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

#define APP_LPN_ENABLE_EVT              (1 << 0)
#define APP_NODE_TEST_EVT               (1 << 1)
#define APP_DELETE_LOCAL_NODE_EVT       (1 << 2)
#define APP_DELETE_NODE_INFO_EVT        (1 << 3)

#define CMD_DELETE_NODE                0xA2
#define CMD_DELETE_NODE_ACK            0x82
#define CMD_DELETE_NODE_INFO           0xA3

#define PERIPHERAL_CMD_LEN             1
#define ADDRESS_LEN                    2

// Delete node command, including 1 byte command code + 2 bytes of node address that needs to be deleted
#define DELETE_NODE_DATA_LEN           (PERIPHERAL_CMD_LEN + ADDRESS_LEN)
// Delete the node command response, including 1 byte command code
#define DELETE_NODE_ACK_DATA_LEN       (PERIPHERAL_CMD_LEN)
// Delete stored node information command, including 1 byte command code
#define DELETE_NODE_INFO_DATA_LEN      (PERIPHERAL_CMD_LEN)

/******************************************************************************/

typedef union
{
    struct
    {
        uint8_t cmd;                /* Command code CMD_DELETE_NODE */
        uint8_t addr[ADDRESS_LEN];  /* Erase address */
    } delete_node;                  /* Delete Node Command */
    struct
    {
        uint8_t cmd;                /* Command code CMD_DELETE_NODE_ACK */
    } delete_node_ack;              /* Delete node command answer */
    struct
    {
        uint8_t cmd;                /* Command code CMD_DELETE_NODE_INFO */
    } delete_node_info;             /* Delete stored node information command */
    struct
    {
        uint8_t buf[20]; /* Receive packets */
    } data;
}app_mesh_manage_t;

/* *
 * @brief Application layer initialization */
void App_Init(void);

/******************************************************************************/

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
