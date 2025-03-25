/********************************** (C) COPYRIGHT *******************************
 * File Name          : app.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2022/01/18
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#include "CONFIG.h"
#include "MESH_LIB.h"
#include "app_vendor_model_srv.h"
#include "app.h"
#include "HAL.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#define ADV_TIMEOUT       K_MINUTES(10)

#define SELENCE_ADV_ON    0x01
#define SELENCE_ADV_OF    0x00

#define APP_WAIT_ADD_APPKEY_DELAY     1600*10

#define APP_DELETE_LOCAL_NODE_DELAY   3200
// shall not less than APP_DELETE_LOCAL_NODE_DELAY
#define APP_DELETE_NODE_INFO_DELAY    3200
/*********************************************************************
 * GLOBAL TYPEDEFS
 */

static uint8_t MESH_MEM[1024 * 3] = {0};

extern const ble_mesh_cfg_t app_mesh_cfg;
extern const struct device  app_dev;

static uint8_t App_TaskID = 0; // Task ID for internal task/event processing

static uint16_t App_ProcessEvent(uint8_t task_id, uint16_t events);

static uint8_t dev_uuid[16] = {0}; // UUID of this device
uint8_t        MACAddr[6];         // Mac for this device

#if(!CONFIG_BLE_MESH_PB_GATT)
NET_BUF_SIMPLE_DEFINE_STATIC(rx_buf, 65);
#endif /* !PB_GATT */

/*********************************************************************
 * LOCAL FUNCION
 */

static void cfg_srv_rsp_handler( const cfg_srv_status_t *val );
static void link_open(bt_mesh_prov_bearer_t bearer);
static void link_close(bt_mesh_prov_bearer_t bearer, uint8_t reason);
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index);
static void vendor_model_srv_rsp_handler(const vendor_model_srv_status_t *val);
static int  vendor_model_srv_send(uint16_t addr, uint8_t *pData, uint16_t len);
static void prov_reset(void);

static struct bt_mesh_cfg_srv cfg_srv = {
    .relay = BLE_MESH_RELAY_ENABLED,
    .beacon = BLE_MESH_BEACON_ENABLED,
#if(CONFIG_BLE_MESH_FRIEND)
    .frnd = BLE_MESH_FRIEND_ENABLED,
#endif
#if(CONFIG_BLE_MESH_PROXY)
    .gatt_proxy = BLE_MESH_GATT_PROXY_ENABLED,
#endif
    /* The default TTL is 3 */
    .default_ttl = 3,
    /* The underlying data is sent 7 times and the interval is 10ms (excluding internal random numbers) */
    .net_transmit = BLE_MESH_TRANSMIT(7, 10),
    /* Retry the underlying forwarding data 7 times, each time interval is 10ms (excluding internal random numbers) */
    .relay_retransmit = BLE_MESH_TRANSMIT(7, 10),
    .handler = cfg_srv_rsp_handler,
};

/* Attention on */
void app_prov_attn_on(struct bt_mesh_model *model)
{
    APP_DBG("app_prov_attn_on");
}

/* Attention off */
void app_prov_attn_off(struct bt_mesh_model *model)
{
    APP_DBG("app_prov_attn_off");
}

const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = app_prov_attn_on,
    .attn_off = app_prov_attn_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

BLE_MESH_HEALTH_PUB_DEFINE(health_pub, 8);

uint16_t cfg_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t cfg_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

uint16_t health_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t health_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

// root model loading
static struct bt_mesh_model root_models[] = {
    BLE_MESH_MODEL_CFG_SRV(cfg_srv_keys, cfg_srv_groups, &cfg_srv),
    BLE_MESH_MODEL_HEALTH_SRV(health_srv_keys, health_srv_groups, &health_srv, &health_pub),
};

struct bt_mesh_vendor_model_srv vendor_model_srv = {
    .srv_tid.trans_tid = 0xFF,
    .handler = vendor_model_srv_rsp_handler,
};

uint16_t vnd_model_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t vnd_model_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

// Custom model loading
struct bt_mesh_model vnd_models[] = {
    BLE_MESH_MODEL_VND_CB(CID_WCH, BLE_MESH_MODEL_ID_WCH_SRV, vnd_model_srv_op, NULL, vnd_model_srv_keys,
                          vnd_model_srv_groups, &vendor_model_srv, NULL),
};

// Model composition elements
static struct bt_mesh_elem elements[] = {
    {
        /* Location Descriptor (GATT Bluetooth Namespace Descriptors) */
        .loc = (0),
        .model_count = ARRAY_SIZE(root_models),
        .models = (root_models),
        .vnd_model_count = ARRAY_SIZE(vnd_models),
        .vnd_models = (vnd_models),
    }
};

// elements composition Node Composition
const struct bt_mesh_comp app_comp = {
    .cid = 0x07D7, // WCH Company ID
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

// Distribution network parameters and callbacks
static const struct bt_mesh_prov app_prov = {
    .uuid = dev_uuid,
    .link_open = link_open,
    .link_close = link_close,
    .complete = prov_complete,
    .reset = prov_reset,
};

app_mesh_manage_t app_mesh_manage;

uint16_t delete_node_info_address=0;
uint8_t settings_load_over = FALSE;

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/* ***************************************************************************
 * @fn prov_enable
 *
 * @brief enable network distribution function
 *
 * @return none */
static void prov_enable(void)
{
    if(bt_mesh_is_provisioned())
    {
        return;
    }

    // Make sure we're scanning for provisioning inviations
    bt_mesh_scan_enable();
    // Enable unprovisioned beacon sending
    bt_mesh_beacon_enable();

    if(CONFIG_BLE_MESH_PB_GATT)
    {
        bt_mesh_proxy_prov_enable();
    }
}

/* ***************************************************************************
 * @fn link_open
 *
 * @brief The link opens the callback after the network distribution time
 *
 * @param bearer - Is the current link PB_ADV or PB_GATT
 *
 * @return none */
static void link_open(bt_mesh_prov_bearer_t bearer)
{
    APP_DBG("");
}

/* ***************************************************************************
 * @fn link_close
 *
 * @brief The link after the network is distributed to close the callback
 *
 * @param bearer - Is the current link PB_ADV or PB_GATT
 * @param reason - link close reason
 *
 * @return none */
static void link_close(bt_mesh_prov_bearer_t bearer, uint8_t reason)
{
    APP_DBG("");
    if(reason != CLOSE_REASON_SUCCESS)
        APP_DBG("reason %x", reason);
}

/* ***************************************************************************
 * @fn prov_complete
 *
 * @brief The distribution network completes the callback and starts broadcasting again
 *
 * @param net_idx - index of network key
 * @param addr - link Close reason network address
 * @param flags - Is it in key refresh state
 * @param iv_index - index of the current network iv
 *
 * @return none */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    APP_DBG("net_idx %x, addr %x", net_idx, addr);
    if(settings_load_over || (vnd_models[0].keys[0]==BLE_MESH_KEY_UNUSED))
    {
        tmos_start_task(App_TaskID, APP_DELETE_LOCAL_NODE_EVT, APP_WAIT_ADD_APPKEY_DELAY);
    }
}

/* ***************************************************************************
 * @fn prov_reset
 *
 * @brief reset network function callback
 *
 * @param none
 *
 * @return none */
static void prov_reset(void)
{
    APP_DBG("");

    prov_enable();
}

/* ***************************************************************************
 * @fn cfg_srv_rsp_handler
 *
 * @brief config Model service callback
 *
 * @param val - Callback parameters, including command type, configuration command execution status
 *
 * @return none */
static void cfg_srv_rsp_handler( const cfg_srv_status_t *val )
{
    if(val->cfgHdr.status)
    {
        // The configuration command execution failed
        APP_DBG("warning opcode 0x%02x", val->cfgHdr.opcode);
        return;
    }
    if(val->cfgHdr.opcode == OP_APP_KEY_ADD)
    {
        APP_DBG("App Key Added");
        // Configure successfully, refresh and delete tasks
        tmos_start_task(App_TaskID, APP_DELETE_LOCAL_NODE_EVT, APP_WAIT_ADD_APPKEY_DELAY);
    }
    else if(val->cfgHdr.opcode == OP_MOD_APP_BIND)
    {
        APP_DBG("Vendor Model Binded");
        // Configure successfully, refresh and delete tasks
        tmos_start_task(App_TaskID, APP_DELETE_LOCAL_NODE_EVT, APP_WAIT_ADD_APPKEY_DELAY);
    }
    else if(val->cfgHdr.opcode == OP_MOD_SUB_ADD)
    {
        APP_DBG("Vendor Model Subscription Set");
        // End of configuration, cancel the delete task
        tmos_stop_task(App_TaskID, APP_DELETE_LOCAL_NODE_EVT);
    }
    else
    {
        APP_DBG("Unknow opcode 0x%02x", val->cfgHdr.opcode);
    }
}

/* ***************************************************************************
 * @fn friend_state
 *
 * @brief Friendship relationship establishment callback
 *
 * @param lpn_addr - Network address of low-power node
 * @param state - callback status
 *
 * @return none */
static void friend_state(uint16_t lpn_addr, uint8_t state)
{
    if(state == FRIEND_FRIENDSHIP_ESTABLISHED)
    {
        APP_DBG("friend friendship established, lpn addr 0x%04x", lpn_addr);
    }
    else if(state == FRIEND_FRIENDSHIP_TERMINATED)
    {
        APP_DBG("friend friendship terminated, lpn addr 0x%04x", lpn_addr);
    }
    else
    {
        APP_DBG("unknow state %x", state);
    }
}

/* ***************************************************************************
 * @fn vendor_model_srv_rsp_handler
 *
 * @brief Custom model service callback
 *
 * @param val - Callback parameters, including message type, data content, length, source address
 *
 * @return none */
static void vendor_model_srv_rsp_handler(const vendor_model_srv_status_t *val)
{
    if(val->vendor_model_srv_Hdr.status)
    {
        // There is a response data transmission timeout no response was received
        APP_DBG("Timeout opcode 0x%02x", val->vendor_model_srv_Hdr.opcode);
        return;
    }
    if(val->vendor_model_srv_Hdr.opcode == OP_VENDOR_MESSAGE_TRANSPARENT_MSG)
    {
        // Transmission data received
        APP_DBG("len %d, data 0x%02x from 0x%04x", val->vendor_model_srv_Event.trans.len,
                val->vendor_model_srv_Event.trans.pdata[0],
                val->vendor_model_srv_Event.trans.addr);
        tmos_memcpy(&app_mesh_manage, val->vendor_model_srv_Event.trans.pdata, val->vendor_model_srv_Event.trans.len);
        switch(app_mesh_manage.data.buf[0])
        {
            // Determine whether it is a delete command
            case CMD_DELETE_NODE:
            {
                if(val->vendor_model_srv_Event.trans.len != DELETE_NODE_DATA_LEN)
                {
                    APP_DBG("Delete node data err!");
                    return;
                }
                uint8_t status;
                APP_DBG("receive delete cmd, send ack and start delete node delay");
                app_mesh_manage.delete_node_ack.cmd = CMD_DELETE_NODE_ACK;
                status = vendor_model_srv_send(val->vendor_model_srv_Event.trans.addr,
                                                app_mesh_manage.data.buf, DELETE_NODE_ACK_DATA_LEN);
                if(status)
                {
                    APP_DBG("send ack failed %d", status);
                }
                // I'm about to delete myself, send the CMD_DELETE_NODE_INFO command first
                APP_DBG("send to all node to let them delete stored info ");
                app_mesh_manage.delete_node_info.cmd = CMD_DELETE_NODE_INFO;
                status = vendor_model_srv_send(BLE_MESH_ADDR_ALL_NODES,
                                                app_mesh_manage.data.buf, DELETE_NODE_INFO_DATA_LEN);
                if(status)
                {
                    APP_DBG("send ack failed %d", status);
                }
                tmos_start_task(App_TaskID, APP_DELETE_LOCAL_NODE_EVT, APP_DELETE_LOCAL_NODE_DELAY);
                break;
            }

            // Determine whether a node has been deleted and the stored node information needs to be deleted.
            case CMD_DELETE_NODE_INFO:
            {
                if(val->vendor_model_srv_Event.trans.len != DELETE_NODE_INFO_DATA_LEN)
                {
                    APP_DBG("Delete node info data err!");
                    return;
                }
                delete_node_info_address = val->vendor_model_srv_Event.trans.addr;
                tmos_start_task(App_TaskID, APP_DELETE_NODE_INFO_EVT, APP_DELETE_NODE_INFO_DELAY);
                break;
            }
        }
    }
    else if(val->vendor_model_srv_Hdr.opcode == OP_VENDOR_MESSAGE_TRANSPARENT_WRT)
    {
        // Write data received
        APP_DBG("len %d, data 0x%02x from 0x%04x", val->vendor_model_srv_Event.write.len,
                val->vendor_model_srv_Event.write.pdata[0],
                val->vendor_model_srv_Event.write.addr);
    }
    else if(val->vendor_model_srv_Hdr.opcode == OP_VENDOR_MESSAGE_TRANSPARENT_IND)
    {
        // The sent indication has received a response
    }
    else
    {
        APP_DBG("Unknow opcode 0x%02x", val->vendor_model_srv_Hdr.opcode);
    }
}

/* ***************************************************************************
 * @fn vendor_model_srv_send
 *
 * @brief send data through the manufacturer's custom model
 *
 * @param addr - The destination address to be sent
 * pData - Data pointer to send
 * len - length of data to be sent
 *
 * @return Reference Global_Error_Code */
static int vendor_model_srv_send(uint16_t addr, uint8_t *pData, uint16_t len)
{
    struct send_param param = {
        .app_idx = vnd_models[0].keys[0], // The app key used by this message, if there is no specificity, the 0th key is used
        .addr = addr,          // The destination address this message is sent to, and the routine is sent to the subscription address, including yourself
        .trans_cnt = 0x01,                // The number of times this message is sent by the user layer
        .period = K_MSEC(400),            // The interval for retransmission of this message is recommended to be no less than (200+50*TTL)ms. If the data is large, it is recommended to lengthen it.
        .rand = (0),                      // Random delay of this message sending
        .tid = vendor_srv_tid_get(),      // tid, each independent message increment loop, srv uses 128~191
        .send_ttl = BLE_MESH_TTL_DEFAULT, // ttl, if there is no specific, use the default value
    };
// Return vendor_message_srv_indicate(&param, pData, len); // Call the reply indicator function of the custom model service to send data, the default timeout is 2s
    return vendor_message_srv_send_trans(&param, pData, len); // Or call the custom model service's transparent transmission function to send data, only send, no response mechanism
}

/* ***************************************************************************
 * @fn keyPress
 *
 * @brief key callback
 *
 * @param keys - key type
 *
 * @return none */
void keyPress(uint8_t keys)
{
    APP_DBG("%d", keys);

    switch(keys)
    {
        default:
        {
            int status;
            uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
            // Sent to the distribution network node
            status = vendor_model_srv_send(0x0001, data, 8);
            if(status)
            {
                APP_DBG("send failed %d", status);
            }
            break;
        }
    }
}

/* ***************************************************************************
 * @fn blemesh_on_sync
 *
 * @brief Synchronize mesh parameters, enable corresponding functions, and it is not recommended to modify them
 *
 * @return none */
void blemesh_on_sync(void)
{
    int        err;
    mem_info_t info;

    if(tmos_memcmp(VER_MESH_LIB, VER_MESH_FILE, strlen(VER_MESH_FILE)) == FALSE)
    {
        PRINT("head file error...\n");
        while(1);
    }

    info.base_addr = MESH_MEM;
    info.mem_len = ARRAY_SIZE(MESH_MEM);

#if(CONFIG_BLE_MESH_FRIEND)
    friend_init_register(bt_mesh_friend_init, friend_state);
#endif /* FRIEND */
#if(CONFIG_BLE_MESH_LOW_POWER)
    lpn_init_register(bt_mesh_lpn_init, lpn_state);
#endif /* LPN */

    GetMACAddress(MACAddr);
    tmos_memcpy(dev_uuid, MACAddr, 6);
    err = bt_mesh_cfg_set(&app_mesh_cfg, &app_dev, MACAddr, &info);
    if(err)
    {
        APP_DBG("Unable set configuration (err:%d)", err);
        return;
    }
    hal_rf_init();
    err = bt_mesh_comp_register(&app_comp);

#if(CONFIG_BLE_MESH_RELAY)
    bt_mesh_relay_init();
#endif /* RELAY  */
#if(CONFIG_BLE_MESH_PROXY || CONFIG_BLE_MESH_PB_GATT)
  #if(CONFIG_BLE_MESH_PROXY)
    bt_mesh_proxy_beacon_init_register((void *)bt_mesh_proxy_beacon_init);
    gatts_notify_register(bt_mesh_gatts_notify);
    proxy_gatt_enable_register(bt_mesh_proxy_gatt_enable);
  #endif /* PROXY  */
  #if(CONFIG_BLE_MESH_PB_GATT)
    proxy_prov_enable_register(bt_mesh_proxy_prov_enable);
  #endif /* PB_GATT  */

    bt_mesh_proxy_init();
#endif /* PROXY || PB-GATT */

#if(CONFIG_BLE_MESH_PROXY_CLI)
    bt_mesh_proxy_client_init(cli); // To be added
#endif                              /* PROXY_CLI */

    bt_mesh_prov_retransmit_init();
#if(!CONFIG_BLE_MESH_PB_GATT)
    adv_link_rx_buf_register(&rx_buf);
#endif /* !PB_GATT */
    err = bt_mesh_prov_init(&app_prov);

    bt_mesh_mod_init();
    bt_mesh_net_init();
    bt_mesh_trans_init();
    bt_mesh_beacon_init();

    bt_mesh_adv_init();

#if((CONFIG_BLE_MESH_PB_GATT) || (CONFIG_BLE_MESH_PROXY) || (CONFIG_BLE_MESH_OTA))
    bt_mesh_conn_adv_init();
#endif /* PROXY || PB-GATT || OTA */

#if(CONFIG_BLE_MESH_SETTINGS)
    bt_mesh_settings_init();
#endif /* SETTINGS */

#if(CONFIG_BLE_MESH_PROXY_CLI)
    bt_mesh_proxy_cli_adapt_init();
#endif /* PROXY_CLI */

#if((CONFIG_BLE_MESH_PROXY) || (CONFIG_BLE_MESH_PB_GATT) || \
    (CONFIG_BLE_MESH_PROXY_CLI) || (CONFIG_BLE_MESH_OTA))
    bt_mesh_adapt_init();
#endif /* PROXY || PB-GATT || PROXY_CLI || OTA */

    if(err)
    {
        APP_DBG("Initializing mesh failed (err %d)", err);
        return;
    }

    APP_DBG("Bluetooth initialized");

#if(CONFIG_BLE_MESH_SETTINGS)
    settings_load();
    settings_load_over = TRUE;
#endif /* SETTINGS */

    if(bt_mesh_is_provisioned())
    {
        APP_DBG("Mesh network restored from flash");
    }
    else
    {
        prov_enable();
    }

    APP_DBG("Mesh initialized");
}

/* ***************************************************************************
 * @fn App_Init
 *
 * @brief Application layer initialization
 *
 * @return none */
void App_Init()
{
    App_TaskID = TMOS_ProcessEventRegister(App_ProcessEvent);

    vendor_model_srv_init(vnd_models);
    blemesh_on_sync();
    HAL_KeyInit();
    HalKeyConfig(keyPress);
    tmos_start_task(App_TaskID, APP_NODE_TEST_EVT, 1600);
}

/* ***************************************************************************
 * @fn App_ProcessEvent
 *
 * @brief application layer event handling function
 *
 * @param task_id - The TMOS assigned task ID.
 * @param events - events to process. This is a bit map and can
 * contains more than one event.
 *
 * @return events not processed */
static uint16_t App_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & APP_NODE_TEST_EVT)
    {
        tmos_start_task(App_TaskID, APP_NODE_TEST_EVT, 2400);
        return (events ^ APP_NODE_TEST_EVT);
    }

    if(events & APP_DELETE_LOCAL_NODE_EVT)
    {
        // Received the delete command to delete your own network information
        APP_DBG("Delete local node");
        // Reset your own network status
        bt_mesh_reset();
        return (events ^ APP_DELETE_LOCAL_NODE_EVT);
    }

    if(events & APP_DELETE_NODE_INFO_EVT)
    {
        // Delete stored information about deleted nodes
        bt_mesh_delete_node_info(delete_node_info_address,app_comp.elem_count);
        APP_DBG("Delete stored node info complete");
        return (events ^ APP_DELETE_NODE_INFO_EVT);
    }

    // Discard unknown events
    return 0;
}

/******************************** endfile @ main ******************************/
