/* ********************************* (C) COPYRIGHT ***************************
 * File Name: rf_basic.c
 * Author: WCH
 * Version: V1.0
 * Date: 2024/08/15
 * Description: 2.4G library basic mode transceiver test routine
 *
 * Power settings
 * RFIP_SetTxPower
 * 1. Supports -20dBm ~ 4dBm dynamic adjustment
 *
 * Send status switching stability time
 * RFIP_SetTxDelayTime
 * 1. If you need to switch channel transmission, the stability time shall not be less than 80us
 *
 * Get signal strength
 * RFIP_ReadRssi
 * 1. The RSSI value of the current packet can be read
 *
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************************* */

/******************************************************************************/
/* The header file contains */
#include "rf_test.h"
#include "hal.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#define  ALIGNED4(x)       ((x+3)/4*4)

tmosTaskID rfTaskID;
rfRoleParam_t gParm;

rfipTx_t gTxParam;
rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];
__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // The received DMA buf cannot be less than 264 bytes

#define  MODE_RX     0
#define  MODE_TX     1


#define  WAIT_ACK         0               // Whether to enable ACK
#define  TEST_DATA_LEN    4               // Data length
#define  TEST_FREQUENCY   16              // Communication frequency

#define  TEST_MODE     MODE_TX            // Send mode
// #define TEST_MODE MODE_RX // Receive mode

#if WAIT_ACK
#define  RF_DEVICE_PERIDOC    4000        // Wait for the transmission rate of ACK mode
#else
#define  RF_DEVICE_PERIDOC    8000        // Send rate without ACK mode
#endif

int8_t gRssi;

int8_t gRssiAverage;
uint32_t gTxCount;
uint32_t gRxCount;

/* *************************************** Send related functions ********************************* */
/* *
 * @brief configures the frequency of sending
 *
 * @param channel_num - channel to switch
 *
 * @return None. */

void rf_tx_set_channel( uint8_t channel_num )
{
    gTxParam.frequency = channel_num;
}

/* *
 * @brief Configure the address to send
 *
 * @param sync_word - Access address to be configured
 *
 * @return None. */

void rf_tx_set_sync_word( uint32_t sync_word )
{
    gTxParam.accessAddress = sync_word;
}

/* *
 * @brief rf send data subroutine
 *
 * @param pBuf - DMA address sent
 *
 * @return None. */
__HIGH_CODE
void rf_tx_start( uint8_t *pBuf )
{
    RFIP_SetTxStart( );
    // Configure the frequency of sending
    gTxParam.frequency = TEST_FREQUENCY;
    // The DMA address sent
    gTxParam.txDMA = (uint32_t)pBuf;
// gTxParam.accessAddress = gParm.accessAddress; // Send synchronous words
// gTxParam.sendCount = 1; // Number of sends
    RFIP_SetTxParm( &gTxParam );
}

/* ****************************** Receive related functions ********************************* */
/* *
 * @brief Configure the received address
 *
 * @param sync_word - Access address to be configured
 *
 * @return None. */

void rf_rx_set_sync_word( uint32_t sync_word )
{
    gRxParam.accessAddress = sync_word;
}

/* *
 * @brief Configure the received frequency
 *
 * @param channel_num - channel to switch
 *
 * @return None. */

void rf_rx_set_channel( uint8_t channel_num )
{
    gRxParam.frequency = channel_num;
}

/* *
 * @brief rf receive data subroutine
 *
 * @param None.
 *
 * @return None. */
__HIGH_CODE
void rf_rx_start( void )
{
    // Configure the frequency of sending
    gRxParam.frequency = TEST_FREQUENCY;
    // Configure the timeout time of reception, no timeout if 0 is
    gRxParam.timeOut = 0;
// gRxParam.accessAddress = gParm.accessAddress; // Receive synchronous words
    RFIP_SetRx( &gRxParam );
}

/* *
 * @brief rf receive data processing
 *
 * @param None.
 *
 * @return None. */
__HIGH_CODE
void rf_rx_process_data( void )
{
    // Obtain signal strength
    gRssi = RFIP_ReadRssi();
    if( !gRssiAverage ) gRssiAverage =  gRssi;
    else gRssiAverage = (gRssi+gRssiAverage)/2;
    gRxCount ++;
//    {
//        uint8_t *pData = (uint8_t *)gRxParam.rxDMA;
//        PRINT("#R %d %d\n",(int8_t)pData[3],(int8_t)gRssi);
//    }
}

/* *********************************************************************************************
 * @fn RF_ProcessCallBack
 *
 * @brief rf interrupt handler
 *
 * @param sta - Interrupt status.
 * id - reserved
 *
 * @return None. */
__HIGH_CODE
void RF_ProcessCallBack( rfRole_States_t sta,uint8_t id  )
{
    if( sta&RF_STATE_RX )
    {
        rf_rx_process_data();
#if( TEST_MODE == MODE_RX )
#if( WAIT_ACK )
        TxBuf[0] = 0x0d;
        TxBuf[1] = 0;
        rf_tx_start( TxBuf );
#else
        rf_rx_start( );
#endif
#endif
    }
    if( sta&RF_STATE_RX_CRCERR )
    {
#if( TEST_MODE == MODE_RX )
        rf_rx_start( );
#else
        PRINT("nak@crc\n");
#endif
    }
    if( sta&RF_STATE_TX_FINISH )
    {
#if( WAIT_ACK )
        rf_rx_start( );
#endif
        gTxCount ++;
    }
    if( sta&RF_STATE_TIMEOUT )
    {
#if( TEST_MODE ==  MODE_RX )
        PRINT("send error.\n");
        rf_rx_start( );
#endif
    }
}

/* *********************************************************************************************
 * @fn RF_ProcessEvent
 *
 * @brief RF layer system task processing
 *
 * @param None.
 *
 * @return None. */
tmosEvents RFRole_ProcessEvent( tmosTaskID task_id, tmosEvents events )
{
    if( events & SYS_EVENT_MSG )
    {
        uint8_t * msgPtr;

        msgPtr = tmos_msg_receive(task_id);
        if( msgPtr )
        {
            /* De-allocate */
             tmos_msg_deallocate( msgPtr );
        }
        return events ^ SYS_EVENT_MSG;
    }
    if( events & RF_START_RX_EVENT )
    {
        rf_rx_start( );
    }
    if( events & RF_TEST_TX_EVENT )
    {
        PRINT("%d\t%d\t%d\n",gTxCount,gRxCount,gRssiAverage );
        gTxCount = 0;
        gRxCount = 0;
        gRssiAverage = 0;
        return events ^ RF_TEST_TX_EVENT;
    }
    if( events & RF_TEST_RX_EVENT )
    {
        PRINT("%d\t%d\t%d\n",gTxCount,gRxCount,gRssiAverage );
        gTxCount = 0;
        gRxCount = 0;
        gRssiAverage = 0;
        return events ^ RF_TEST_RX_EVENT;
    }
    return 0;
}

/* ***************************************************************************
 * @fn TMR0_IRQHandler
 *
 * @brief TMR0 interrupt function
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) // TMR0 Timed interrupt
{
    if(TMR0_GetITFlag(TMR0_3_IT_CYC_END))
    {
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END); // Clear the interrupt flag
        // Initialize the sent data
        TxBuf[0] = 0x55;
        TxBuf[1] = TEST_DATA_LEN;
        TxBuf[2]++;
        TxBuf[3] = gRssi;
        rf_tx_start( TxBuf );
    }
}

/* *********************************************************************************************
 * @fn RFRole_Init
 *
 * @brief RF application layer initialization
 *
 * @param None.
 *
 * @return None. */
void RFRole_Init(void)
{
    rfTaskID = TMOS_ProcessEventRegister( RFRole_ProcessEvent );
    {
        rfRoleConfig_t conf ={0};

        conf.TxPower = LL_TX_POWEER_4_DBM;
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_RX|RF_STATE_RX_CRCERR|RF_STATE_TX_FINISH|RF_STATE_TIMEOUT;
        RFRole_BasicInit( &conf );
    }
    {
        gParm.accessAddress = 0x71762345;
        gParm.crcInit = 0x555555;
        // Configure PHY type
        gParm.properties = LLE_MODE_PHY_2M;
        // Configure the retransmission interval, and it is valid if the number of sending times is greater than 1.
        gParm.sendInterval = 1999*2;
        // Configure sending stability time
        gParm.sendTime = 10*2;
        RFRole_SetParam( &gParm );
    }
    // TX-related parameters, global variables
    {
        gTxParam.accessAddress = gParm.accessAddress;
        gTxParam.crcInit = gParm.crcInit;
        gTxParam.properties = gParm.properties;
        gTxParam.sendCount = 1;
        gTxParam.txDMA = (uint32_t)TxBuf;
    }
    // RX-related parameters, global variables
    {
        gRxParam.accessAddress = gParm.accessAddress;
        gRxParam.crcInit = gParm.crcInit;
        gRxParam.properties = gParm.properties;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }
    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );
    PRINT("rf role init.id=%d\n",rfTaskID);

#if( TEST_MODE == MODE_RX )
    PRINT("----------------- rx -----------------\n");
    gRssiAverage = 0;
    gTxCount = 0;
    gRxCount = 0;
    tmos_set_event(rfTaskID, RF_START_RX_EVENT );
    tmos_start_reload_task(rfTaskID, RF_TEST_RX_EVENT, MS1_TO_SYSTEM_TIME(1000));
#else
    PRINT("----------------- tx -----------------\n");
    gRssiAverage = 0;
    gTxCount = 0;
    gRxCount = 0;
    tmos_start_reload_task(rfTaskID, RF_TEST_TX_EVENT, MS1_TO_SYSTEM_TIME(1000));
    TMR0_TimerInit( GetSysClock() / RF_DEVICE_PERIDOC );
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // Turn on interrupt
#endif
    PFIC_SetPriority( TMR0_IRQn, 0x80 );
    PFIC_EnableIRQ(TMR0_IRQn);
}

/******************************** endfile @rf ******************************/
