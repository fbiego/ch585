/********************************** (C) COPYRIGHT *******************************
 * File Name          : app.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2021/03/24
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#include "CONFIG.h"
#include "MESH_LIB.h"
#include "app_vendor_model.h"
#include "app_generic_onoff_model.h"
#include "app_als_windspeed_attr.h"
#include "app.h"
#include "HAL.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#define ADV_TIMEOUT       K_MINUTES(10)

#define SELENCE_ADV_ON    0x01
#define SELENCE_ADV_OF    0x00

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

static uint8_t MESH_MEM[1024 * 2] = {0};

extern const ble_mesh_cfg_t app_mesh_cfg;
extern const struct device  app_dev;

static uint8_t App_TaskID = 0; // Task ID for internal task/event processing

static uint16_t App_ProcessEvent(uint8_t task_id, uint16_t events);

#if(!CONFIG_BLE_MESH_PB_GATT)
NET_BUF_SIMPLE_DEFINE_STATIC(rx_buf, 65);
#endif /* !PB_GATT */

/*********************************************************************
 * LOCAL FUNCION
 */

static void link_open(bt_mesh_prov_bearer_t bearer);
static void link_close(bt_mesh_prov_bearer_t bearer, uint8_t reason);
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index);
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
    .default_ttl = 3,

    /* 8 transmissions with 10ms interval */
    .net_transmit = BLE_MESH_TRANSMIT(7, 10),
    .relay_retransmit = BLE_MESH_TRANSMIT(7, 10),
};

static struct bt_mesh_health_srv health_srv;

BLE_MESH_HEALTH_PUB_DEFINE(health_pub, 8);

uint16_t cfg_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t cfg_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

uint16_t health_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t health_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

uint16_t gen_onoff_srv_keys[CONFIG_MESH_MOD_KEY_COUNT_DEF] = {BLE_MESH_KEY_UNUSED};
uint16_t gen_onoff_srv_groups[CONFIG_MESH_MOD_GROUP_COUNT_DEF] = {BLE_MESH_ADDR_UNASSIGNED};

static struct bt_mesh_model root_models[] = {
    BLE_MESH_MODEL_CFG_SRV(cfg_srv_keys, cfg_srv_groups, &cfg_srv),
    BLE_MESH_MODEL_HEALTH_SRV(health_srv_keys, health_srv_groups, &health_srv, &health_pub),
    BLE_MESH_MODEL(BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_op, NULL, gen_onoff_srv_keys, gen_onoff_srv_groups, NULL),
};

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
    .uuid = tm_uuid,
    .static_val_len = ARRAY_SIZE(static_key),
    .static_val = static_key,
    .link_open = link_open,
    .link_close = link_close,
    .complete = prov_complete,
    .reset = prov_reset,
};

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/* ***************************************************************************
 * @fn silen_adv_set
 *
 * @brief Set silent broadcast
 *
 * @param flag - 0 (in undistributed broadcast state), 1 (in silent broadcast state).
 *
 * @return none */
static void silen_adv_set(uint8_t flag)
{
    tm_uuid[13] &= ~BIT(0);
    tm_uuid[13] |= (BIT_MASK(1) & flag);
}

/* ***************************************************************************
 * @fn prov_enable
 *
 * @brief enable network distribution function
 *
 * @param none
 *
 * @return none */
static void prov_enable(void)
{
    silen_adv_set(SELENCE_ADV_OF);

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

    tmos_start_task(App_TaskID, APP_SILENT_ADV_EVT, ADV_TIMEOUT);
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
    APP_DBG(" ");

    tmos_stop_task(App_TaskID, APP_SILENT_ADV_EVT);
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

    if(!bt_mesh_is_provisioned())
    {
        prov_enable();
    }
    else
    {
        /* Tmall Genie will not issue Config_model_app_bind and Config_Model_Subscription_Add messages.
            IOT devices need to bind all models of Element to all Elements and issue AppKeys according to the product type.
            Model subscribes to the corresponding multicast address (see the software specifications of each product for specific multicast addresses in the category). Bluetooth Mesh Device
            After completing the distribution network, a message reporting needs to be reported. The reporting message includes all the reported attributes supported by the device. */

        /* For Light Subscription group address */
        root_models[2].groups[0] = (uint16_t)0xC007;
        root_models[2].groups[1] = (uint16_t)0xCFFF;
        bt_mesh_store_mod_sub(&root_models[2]);

        root_models[2].keys[0] = (uint16_t)0x0000;
        bt_mesh_store_mod_bind(&root_models[2]);

        /* For Light Subscription group address */
        vnd_models[0].groups[0] = (uint16_t)0xC007;
        vnd_models[0].groups[1] = (uint16_t)0xCFFF;
        bt_mesh_store_mod_sub(&vnd_models[0]);

        vnd_models[0].keys[0] = (uint16_t)0x0000;
        bt_mesh_store_mod_bind(&vnd_models[0]);
    }
}

/* ***************************************************************************
 * @fn prov_complete
 *
 * @brief The distribution network completes the callback and starts broadcasting again
 *
 * @param net_idx - index of network key
 * @param addr - Network address
 * @param flags - Is it in key refresh state
 * @param iv_index - index of the current network iv
 *
 * @return none */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    /* After the device is powered on, if the network is already distributed, all supported attribute statuses need to be reported after a random interval of 1 to 10 seconds. */
    tmosTimer rand_timer;
    APP_DBG(" ");

    rand_timer = K_SECONDS(5) + (tmos_rand() % K_SECONDS(6));
    tmos_start_task(App_TaskID, APP_SILENT_ADV_EVT, rand_timer);
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
 * @fn ind_end_cb
 *
 * @brief send reset event to complete callback
 *
 * @param err - Error code
 * @param cb_data - callback parameters
 *
 * @return none */
static void ind_end_cb(int err, void *cb_data)
{
    APP_DBG(" bt_mesh_reset ");
    bt_mesh_reset();
}

static const struct bt_adv_ind_send_cb reset_cb = {
    .end = ind_end_cb,
};

/* ***************************************************************************
 * @fn send_support_attr
 *
 * @brief Send all supported reporting attributes to Tmall Genie. This message determines which functions the Tmall Genie determines
 *
 * @param none
 *
 * @return none */
void send_support_attr(void)
{
    struct bt_mesh_indicate *ind;
    APP_DBG("");

    if(!bt_mesh_is_provisioned())
    {
        APP_DBG("Local Dev Unprovisioned");
        return;
    }

    ind = bt_mesh_ind_alloc(32);
    if(!ind)
    {
        APP_DBG("Unable allocate buffers");
        return;
    }
    ind->param.trans_cnt = 0x09;
    ind->param.period = K_MSEC(300);
    ind->param.send_ttl = BLE_MESH_TTL_DEFAULT;
    ind->param.tid = als_avail_tid_get();

    /* Init indication opcode */
    bt_mesh_model_msg_init(&(ind->buf->b), OP_VENDOR_MESSAGE_ATTR_INDICATION);

    /* Add tid field */
    net_buf_simple_add_u8(&(ind->buf->b), ind->param.tid);

    // Add switch properties
    {
        /* Add generic onoff attrbute op */
        net_buf_simple_add_le16(&(ind->buf->b), ALI_GEN_ATTR_TYPE_POWER_STATE);

        /* Add current generic onoff status */
        net_buf_simple_add_u8(&(ind->buf->b), read_led_state(MSG_PIN));
    }
    // Add windspeed attribute
    {
        /* Add windspeed attrbute opcode */
        net_buf_simple_add_le16(&(ind->buf->b), ALI_GEN_ATTR_TYPE_WINDSPEED);

        /* Add windspeed status  */
        net_buf_simple_add_u8(&(ind->buf->b), read_windspeed());
    }

    bt_mesh_indicate_send(ind);
}

/* ***************************************************************************
 * @fn send_led_state
 *
 * @brief Send the current light status to Tmall Elf
 *
 * @param none
 *
 * @return none */
void send_led_state(void)
{
    APP_DBG("");
    struct indicate_param param = {
        .trans_cnt = 0x09,
        .period = K_MSEC(300),
        .send_ttl = BLE_MESH_TTL_DEFAULT,
        .tid = als_avail_tid_get(),
    };

    toggle_led_state(MSG_PIN);

    if(!bt_mesh_is_provisioned())
    {
        APP_DBG("Local Dev Unprovisioned");
        return;
    }

    send_led_indicate(&param);
}

/* ***************************************************************************
 * @fn send_reset_indicate
 *
 * @brief Send a reset event to Tmall Genie. After the sending is completed, the distribution network status will be cleared and the self-mesh network will be reset.
 *
 * @param none
 *
 * @return none */
void send_reset_indicate(void)
{
    struct bt_mesh_indicate *ind;
    APP_DBG("");

    if(!bt_mesh_is_provisioned())
    {
        APP_DBG("Local Dev Unprovisioned");
        return;
    }

    ind = bt_mesh_ind_alloc(16);
    if(!ind)
    {
        APP_DBG("Unable allocate buffers");
        return;
    }
    ind->param.trans_cnt = 0x09;
    ind->param.period = K_MSEC(300);
    ind->param.cb = &reset_cb;
    ind->param.send_ttl = BLE_MESH_TTL_DEFAULT;
    ind->param.tid = als_avail_tid_get();

    /* Init indication opcode */
    bt_mesh_model_msg_init(&(ind->buf->b), OP_VENDOR_MESSAGE_ATTR_INDICATION);

    /* Add tid field */
    net_buf_simple_add_u8(&(ind->buf->b), ind->param.tid);

    /* Add event report opcode */
    net_buf_simple_add_le16(&(ind->buf->b), ALI_GEN_ATTR_TYPE_EVENT_TRIGGER);

    /* Add reset event */
    net_buf_simple_add_u8(&(ind->buf->b), ALI_GEN_ATTR_TYPE_HARDWARE_RESET);

    bt_mesh_indicate_send(ind);
}

#define HAL_KEY_SEND_MSG    BIT(0)
#define HAL_KEY_RESET       BIT(1)

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
    APP_DBG("keys : %d ", keys);

    switch(keys)
    {
        case HAL_KEY_SEND_MSG:
            send_led_state();
            break;
        case HAL_KEY_RESET:
            send_reset_indicate();
            break;
    }
}

/* ***************************************************************************
 * @fn app_silent_adv
 *
 * @brief If the network is not successfully allocated after the timeout, enter silent broadcast mode. If the network is already allocated, send the supported attributes to the Tmall Elf
 *
 * @param none
 *
 * @return none */
static void app_silent_adv(void)
{
    APP_DBG("");
    if(bt_mesh_is_provisioned())
    {
        send_support_attr();
        return;
    }

    silen_adv_set(SELENCE_ADV_ON);

    /* Disable Scanner not response Provisioner message */
    bt_mesh_scan_disable();
}

/* ***************************************************************************
 * @fn blemesh_on_sync
 *
 * @brief Synchronize mesh parameters, enable corresponding functions, and it is not recommended to modify them
 *
 * @param none
 *
 * @return none */
void blemesh_on_sync(void)
{
    int        err;
    mem_info_t info;

    if(tmos_memcmp(VER_MESH_LIB, VER_MESH_FILE, strlen(VER_MESH_FILE)) == FALSE)
    {
        APP_DBG("head file error...\n");
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

    err = bt_mesh_cfg_set(&app_mesh_cfg, &app_dev, MacAddr, &info);
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
 * @param none
 *
 * @return none */
void App_Init()
{
    App_TaskID = TMOS_ProcessEventRegister(App_ProcessEvent);

    als_vendor_init(vnd_models);
    blemesh_on_sync();
    HAL_KeyInit();
    HalKeyConfig(keyPress);
    set_led_state(MSG_PIN, 0);
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
    if(events & APP_SILENT_ADV_EVT)
    {
        app_silent_adv();
        return (events ^ APP_SILENT_ADV_EVT);
    }

    // Discard unknown events
    return 0;
}

/******************************** endfile @ main ******************************/
