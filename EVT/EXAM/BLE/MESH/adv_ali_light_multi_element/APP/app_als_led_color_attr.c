/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_als_led_color_attr.c
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
#include "app_mesh_config.h"
#include "app_als_led_color_attr.h"
#include "app_vendor_model.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

#define ALI_DEF_TTL    (10)

// Simulate LED_color value
int32_t device_led_color_adj = 0;

/* *********************************************************************************************
 * Function Name: read_led_color
 * Description: Get the current led_color
 * Input: None
 * Return : None
 ********************************************************************************************* */
void read_led_color(int32_t *pcolor)
{
    APP_DBG("device_led_color_adj: %d ", (int32_t)device_led_color_adj);
    *pcolor = device_led_color_adj;
}

/* *********************************************************************************************
 * Function Name : set_led_colors
 * Description: Set the current led_color
 * Input: None
 * Return : None
 ********************************************************************************************* */
void set_led_color(int32_t color)
{
    device_led_color_adj = color;
}

/* *********************************************************************************************
 * Function Name: gen_led_color_status
 * Description: Reply to Tmall Elf led_color
 * Input: model: Model parameters
 * ctx: Data parameters
 * Return : None
 ********************************************************************************************* */
static void gen_led_color_status(struct bt_mesh_model   *model,
                                 struct bt_mesh_msg_ctx *ctx)
{
    NET_BUF_SIMPLE_DEFINE(msg, 32);
    int     err;
    int32_t color;
    ////////////////////////////////////////////////////////////////////////
    //  0xD3  0xA8  0x01  |  0x##   |  0x##  0x##       |  0x##  0x## ....//
    //      Opcode        |  TID    | Attribute Type    | Attribute Value //
    ////////////////////////////////////////////////////////////////////////
    bt_mesh_model_msg_init(&msg, OP_VENDOR_MESSAGE_ATTR_STATUS);
    net_buf_simple_add_u8(&msg, als_avail_tid_get());
    net_buf_simple_add_le16(&msg, ALI_GEN_ATTR_TYPE_LIGHTCOLOR_ADJ);
    read_led_color(&color);
    net_buf_simple_add_le32(&msg, color);

    APP_DBG("ttl: 0x%02x dst: 0x%04x", ctx->recv_ttl, ctx->recv_dst);

    if(ctx->recv_ttl != ALI_DEF_TTL)
    {
        ctx->send_ttl = BLE_MESH_TTL_DEFAULT;
    }
    else
    {
        ctx->send_ttl = 0;
    }

    err = bt_mesh_model_send(model, ctx, &msg, NULL, NULL);
    if(err)
    {
        APP_DBG("send status failed: %d", err);
    }
}

/* *********************************************************************************************
 * Function Name: gen_led_color_get
 * Description: The get led_color command issued by Tmall elves
 * Input: model: Model parameters
 * ctx: Data parameters
 * buf: Data content
 * Return : None
 ********************************************************************************************* */
void gen_led_color_get(struct bt_mesh_model   *model,
                       struct bt_mesh_msg_ctx *ctx,
                       struct net_buf_simple  *buf)
{
    APP_DBG(" ");
    gen_led_color_status(model, ctx);
}

/* *********************************************************************************************
* Function Name : gen_led_color_set
* Description: The setting led_color command issued by Tmall elves
                                        If it is different from the current led_color, it is also necessary to send an ind to Tmall
* Input: model: Model parameters
* ctx: Data parameters
* buf: Data content
* Return : None
********************************************************************************************* */
void gen_led_color_set(struct bt_mesh_model   *model,
                       struct bt_mesh_msg_ctx *ctx,
                       struct net_buf_simple  *buf)
{
    struct indicate_param param = {
        .trans_cnt = 0x09,
        .period = K_MSEC(300),
        .rand = (tmos_rand() % 50),
        .tid = als_avail_tid_get(),
    };

    APP_DBG("ttl: 0x%02x dst: 0x%04x rssi: %d len %d",
            ctx->recv_ttl, ctx->recv_dst, ctx->recv_rssi, buf->len);

    if((buf->data[1] | (buf->data[2] << 8)) == ALI_GEN_ATTR_TYPE_LIGHTCOLOR_ADJ)
    {
        APP_DBG("%x %x %x %x %x %x ",
                buf->data[0], buf->data[1], buf->data[2], buf->data[3], buf->data[4], buf->data[5]);
        int32_t color = (buf->data[3] | (buf->data[4] << 8) | (buf->data[5] << 16) | (buf->data[6] << 24));
        // The command is the set value
        set_led_color(color);
    }

    if(ctx->recv_ttl != ALI_DEF_TTL)
    {
        param.send_ttl = BLE_MESH_TTL_DEFAULT;
    }

    /* Overwrite default configuration */
    if(BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst))
    {
        param.rand = 0;
        param.send_ttl = BLE_MESH_TTL_DEFAULT;
        param.period = K_MSEC(100);
    }

    send_led_color_indicate(&param);

    gen_led_color_status(model, ctx);
}

/* *********************************************************************************************
 * Function Name : gen_led_color_set_unack
 * Description: The setting led_color command issued by Tmall elves (no response)
 * Input: model: Model parameters
 * ctx: Data parameters
 * buf: Data content
 * Return : None
 ********************************************************************************************* */
void gen_led_color_set_unack(struct bt_mesh_model   *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple  *buf)
{
    APP_DBG(" ");

    if((buf->data[1] | (buf->data[2] << 8)) == ALI_GEN_ATTR_TYPE_LIGHTCOLOR_ADJ)
    {
        uint32_t color = (buf->data[3] | (buf->data[4] << 8) | (buf->data[5] << 16) | (buf->data[6] << 24));
        // The command is the set value
        set_led_color(color);
    }
}

/******************************** endfile @ main ******************************/
