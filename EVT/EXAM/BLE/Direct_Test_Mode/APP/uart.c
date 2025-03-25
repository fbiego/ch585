/********************************** (C) COPYRIGHT *******************************
 * File Name          : uart.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/06/30
 * Description        : 
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include <usbhs_device.h>
#include "CH58x_common.h"
#include "config.h"
#include "uart.h"
#include "test_dtm.h"
#include "app_usb.h"
#include "stdbool.h"

static tmosTaskID uart_taskid;
static tmosTaskID usb_taskid;

static uint8_t rx_buf[100];
static struct simple_buf *uart_buf = NULL;
static struct simple_buf uart_buffer;
static uint8_t rx_ubuf[100];
struct simple_buf *usb_buf = NULL;
static struct simple_buf usb_buffer;

uint8_t uart_flag;
uint8_t usb_flag;

/*********************************************************************
 * @fn      uart_buffer_create
 *
 * @brief   Create a file called uart_buffer simple buffer of the buffer
 *          and assign its address to the variable pointed to by the buffer pointer.
 *
 * @param   buf    -   a parameter buf pointing to a pointer.
 *
 * @return  none
 */
void uart_buffer_create(struct simple_buf **buf)
{
    *buf = simple_buf_create(&uart_buffer, rx_buf, sizeof(rx_buf));
}

/*********************************************************************
 * @fn      usb_buffer_create
 *
 * @brief   Create a file called usb_buffer simple buffer of the buffer
 *          and assign its address to the variable pointed to by the buffer pointer.
 *
 * @param   buf    -   a parameter buf pointing to a pointer.
 *
 * @return  none
 */
void usb_buffer_create(struct simple_buf **buf)
{
    *buf = simple_buf_create(&usb_buffer, rx_ubuf, sizeof(rx_ubuf));
}

/*********************************************************************
 * @fn      uart_start_receiving
 *
 * @brief   Serial port starts receiving.
 *
 * @param   none
 *
 * @return  none
 */
int uart_start_receiving(void)
{
#if DEBUG == 1
    uart_flag = UART_STATUS_RCVING;
    uart_buffer_create(&uart_buf);
    PFIC_EnableIRQ(UART0_IRQn);

#elif DEBUG == 0
    uart_flag = UART_STATUS_RCVING;
    uart_buffer_create(&uart_buf);
    PFIC_EnableIRQ(UART1_IRQn);
#endif

    return 0;
}

/*********************************************************************
 * @fn      usb_start_receiving
 *
 * @brief   Usb starts receiving.
 *
 * @param   none
 *
 * @return  none
 */
int usb_start_receiving(void)
{
    usb_flag = USB_STATUS_RCVING;
    usb_buffer_create(&usb_buf);

    return 0;
}

/*********************************************************************
 * @fn      uart_send
 *
 * @brief   Serial port starts Sending.
 *
 * @param   buf    -   defined simple_buf structural object.
 *
 * @return  none
 */
int uart_send(struct simple_buf *buf)
{
#if DEBUG == 1
    uart_flag = UART_STATUS_SENDING;

    uint8_t *send_data;
    uint16_t send_len;

    send_len = buf->len;
    send_data = simple_buf_pull(buf, buf->len);

    if(send_len > 0)
    {
        UART0_SendString(send_data, send_len);
        uart_flag = UART_STATUS_IDLE;

//        PRINT("uart0 send %d bytes\n", send_len);
//        PRINT("[\n");
//        for(int i = 0; i < send_len; i++)
//        {
//            if(i) PRINT(" ");
//            PRINT("%#x", send_data[i]);
//        }
//        PRINT("]\n");
    }
#elif DEBUG == 0
    uart_flag = UART_STATUS_SENDING;

    uint8_t *send_data;
    uint16_t send_len;

    send_len = buf->len;
    send_data = simple_buf_pull(buf, buf->len);

    UART1_SendString(send_data, send_len);
    uart_flag = UART_STATUS_IDLE;


//    PRINT("uart1 send %d bytes\n", send_len);
//    PRINT("[\n");
//    for(int i = 0; i < send_len; i++)
//    {
//        if(i) PRINT(" ");
//        PRINT("%#x", send_data[i]);
//    }
//    PRINT("]\n");

#endif

    return 0;
}

/*********************************************************************
 * @fn      usb_send
 *
 * @brief   Usb starts Sending.
 *
 * @param   buf    -   defined simple_buf structural object.
 *
 * @return  none
 */
int usb_send(struct simple_buf *buf)
{
    usb_flag = USB_STATUS_SENDING;

    uint8_t *send_data;
    uint16_t send_len;

    send_len = buf->len;
    send_data = simple_buf_pull(buf, buf->len);

#if USBFS_ENABLE == 1
    USBSendData(send_data, send_len);
#endif
#if USBHS_ENABLE == 1
    USBHS_Endp_DataUp(DEF_UEP2, send_data, send_len, DEF_UEP_CPY_LOAD);
#endif

    usb_flag = UART_STATUS_IDLE;
//    PRINT("usb send %d bytes\n", send_len);
//    PRINT("[\n");
//    for(int i = 0; i < send_len; i++)
//    {
//        if(i) PRINT(" ");
//        PRINT("%#x", send_data[i]);
//    }
//    PRINT("]\n");

    return 0;
}

/*********************************************************************
 * @fn      uart_processevent
 *
 * @brief   process handle
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
tmosEvents uart_processevent(tmosTaskID task_id, tmosEvents events)
{
    if(events & SYS_EVENT_MSG)
    { // HALtmos_msg_receive
        uint8_t *msgPtr;

        msgPtr = tmos_msg_receive(task_id);
        if(msgPtr)
        {
            /* De-allocate */
            tmos_msg_deallocate(msgPtr);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & UART_RECEIVE_POLL_EVT)
    {
        if(uart_flag == UART_STATUS_RCV_END)
        {
//            PRINT("uart recevied %d bytes\n", uart_buf->len);
//            PRINT("[\n");
//            for(int i = 0; i < uart_buf->len; i++)
//            {
//                if(i)
//                   PRINT(" ");
//                   PRINT("%#x", uart_buf->data[i]);
//            }
//            PRINT("]\n");

            struct uart_process_msg *uart_msg;

            uart_msg = (struct uart_process_msg *)      \
                tmos_msg_allocate(sizeof(struct uart_process_msg));

            if(uart_msg)
            {
                uart_msg->hdr.event = UART_PROCESS_EVT;
                uart_msg->hdr.status = true;
                uart_msg->data = (uint8_t *)uart_buf;
                tmos_msg_send(test_taskid, (uint8_t *)uart_msg );
            }
        }

        return (events ^ UART_RECEIVE_POLL_EVT);
    }

    if(events & USB_RECEIVE_POLL_EVT)
    {
//        PRINT("usb_flag:%d\n",usb_flag);
        if(usb_flag == USB_STATUS_RCV_END)
        {
//            PRINT("usb recevied %d bytes\n", usb_buf->len);
//            PRINT("[\n");
//            for(int i = 0; i < usb_buf->len; i++)
//            {
//                if(i)
//                   PRINT(" ");
//                   PRINT("%#x", usb_buf->data[i]);
//            }
//            PRINT("]\n");

            struct usb_process_msg *usb_msg;

            usb_msg = (struct usb_process_msg *)      \
                tmos_msg_allocate(sizeof(struct usb_process_msg));

            if(usb_msg)
            {
                usb_msg->hdr.event = USB_PROCESS_EVT;
                usb_msg->hdr.status = true;
                usb_msg->data = (uint8_t *)usb_buf;
                tmos_msg_send(test_taskid, (uint8_t *)usb_msg );
            }
        }

        return (events ^ USB_RECEIVE_POLL_EVT);
    }

    return 0;
}

/*********************************************************************
 * @fn      uart_task_init
 *
 * @brief   Serial port initialization and task initialization.
 *
 * @param   none
 *
 * @return  none
 */
void uart_task_init(void)
{
    uart_taskid = TMOS_ProcessEventRegister(uart_processevent);

#if DEBUG == 0
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(bRXD1, GPIO_ModeIN_PU);

    UART1_DefInit();
    UART1_ByteTrigCfg(UART_7BYTE_TRIG);
    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    tmos_start_reload_task(uart_taskid, UART_RECEIVE_POLL_EVT,
        MS1_TO_SYSTEM_TIME(100));

    uart_flag = UART_STATUS_IDLE;
    uart_start_receiving();

#elif DEBUG == 1
    GPIOB_SetBits(bTXD0);
    GPIOB_ModeCfg(bTXD0, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(bRXD0, GPIO_ModeIN_PU);

    UART0_DefInit();
    UART0_ByteTrigCfg(UART_7BYTE_TRIG);
    UART0_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    tmos_start_reload_task(uart_taskid, UART_RECEIVE_POLL_EVT,
        MS1_TO_SYSTEM_TIME(100));

    uart_flag = UART_STATUS_IDLE;
    uart_start_receiving();
#endif
}

/*********************************************************************
 * @fn      USB_Task_Init
 *
 * @brief   USB initialization and task initialization.
 *
 * @param   none
 *
 * @return  none
 */
void USB_Task_Init(void)
{
    usb_taskid = TMOS_ProcessEventRegister(uart_processevent);

    tmos_start_reload_task(usb_taskid, USB_RECEIVE_POLL_EVT,
        MS1_TO_SYSTEM_TIME(100));

    usb_flag = USB_STATUS_IDLE;
    usb_start_receiving();
}

/*********************************************************************
 * @fn      UART0_IRQHandler
 *
 * @brief   UART0
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void UART0_IRQHandler(void)
{
    volatile uint8_t i;

    switch(UART0_GetITFlag())
    {
        case UART_II_LINE_STAT: // 
        {
            (void)UART0_GetLinSTA();
            break;
        }

        case UART_II_RECV_RDY: // 
            for(i = 0; i < (7 - 1); i++)
            {
                simple_buf_add_u8(uart_buf, UART0_RecvByte());
            }
            break;

        case UART_II_RECV_TOUT: // 
            while(R8_UART0_RFC)
            {
                simple_buf_add_u8(uart_buf, UART0_RecvByte());
            }
            uart_flag = UART_STATUS_RCV_END;
            PFIC_DisableIRQ(UART0_IRQn);
            break;

        case UART_II_THR_EMPTY: // 
            break;

        case UART_II_MODEM_CHG: // 0
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      UART1_IRQHandler
 *
 * @brief   UART1
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void UART1_IRQHandler(void)
{
    volatile uint8_t i;

    switch(UART1_GetITFlag())
    {
        case UART_II_LINE_STAT: // 
        {
            (void)UART1_GetLinSTA();
            break;
        }

        case UART_II_RECV_RDY: // 
            for(i = 0; i < (7 - 1); i++)
            {
                simple_buf_add_u8(uart_buf, UART1_RecvByte());
            }
            break;

        case UART_II_RECV_TOUT: // 
            while(R8_UART1_RFC)
            {
                simple_buf_add_u8(uart_buf, UART1_RecvByte());
            }
            uart_flag = UART_STATUS_RCV_END;
            PFIC_DisableIRQ(UART1_IRQn);
            break;

        case UART_II_THR_EMPTY: // 
            break;

        default:
            break;
    }
}
