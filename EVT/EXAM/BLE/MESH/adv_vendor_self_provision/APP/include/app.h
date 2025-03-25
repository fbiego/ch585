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

#define APP_NODE_EVT                   (1 << 0)
#define APP_DELETE_NODE_TIMEOUT_EVT    (1 << 1)
#define APP_DELETE_LOCAL_NODE_EVT      (1 << 2)
#define APP_DELETE_NODE_INFO_EVT       (1 << 3)

#define CMD_PROVISION                  0xA1
#define CMD_DELETE_NODE                0xA2
#define CMD_DELETE_NODE_ACK            0x82
#define CMD_DELETE_NODE_INFO           0xA3
#define CMD_LOCAL_RESET                0xAF

#define PERIPHERAL_CMD_LEN             1
#define PROVISION_NET_KEY_LEN          16
#define ADDRESS_LEN                    2

// The distribution parameters received through ble include 1 byte command code + 16 byte network key + 2 byte network address
#define PROVISION_DATA_LEN             (PERIPHERAL_CMD_LEN + PROVISION_NET_KEY_LEN + ADDRESS_LEN)
// Delete node command, including 1 byte command code + 2 bytes of node address that needs to be deleted
#define DELETE_NODE_DATA_LEN           (PERIPHERAL_CMD_LEN + ADDRESS_LEN)
// Delete the node command response, including 1 byte command code
#define DELETE_NODE_ACK_DATA_LEN       (PERIPHERAL_CMD_LEN)
// Delete stored node information command, including 1 byte command code
#define DELETE_NODE_INFO_DATA_LEN      (PERIPHERAL_CMD_LEN)
// Local reset command, including 1 byte command code
#define LOCAL_RESET_DATA_LEN           (PERIPHERAL_CMD_LEN)

/******************************************************************************/

typedef struct
{
    uint16_t node_addr;
    uint16_t elem_count;
    uint16_t net_idx;
    uint16_t retry_cnt : 12,
        fixed : 1,
        blocked : 1;

} node_t;

typedef union
{
    struct
    {
        uint8_t cmd;                            /* Command code CMD_PROVISION */
        uint8_t net_key[PROVISION_NET_KEY_LEN]; /* Subsequent data length */
        uint8_t addr[ADDRESS_LEN];              /* Erase address */
    } provision;                                /* Network distribution command */
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
        uint8_t cmd;                /* Command code CMD_LOCAL_RESET */
    } local_reset;                  /* Local factory reset command */
    struct
    {
        uint8_t buf[20]; /* Receive packets */
    } data;
}app_mesh_manage_t;

void App_Init(void);

/******************************************************************************/

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
