/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_SPI1.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/15
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* ***************************************************************************
 * @fn SPI1_MasterDefInit
 *
 * @brief Host mode default initialization: Mode 0+3 line full duplex + 8MHz
 *
 * @param none
 *
 * @return none */
void SPI1_MasterDefInit(void)
{
    R8_SPI1_CLOCK_DIV = 4; // Main frequency clock 4 times
    R8_SPI1_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI1_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE;
    R8_SPI1_CTRL_CFG |= RB_SPI_AUTO_IF; // Access BUFFER/FIFO to automatically clear the IF_BYTE_END flag
}

/* ***************************************************************************
 * @fn SPI1_CLKCfg
 *
 * @brief SPI1 reference clock configuration, = d*Tsys
 *
 * @param c - clock frequency division coefficient
 *
 * @return none */
void SPI1_CLKCfg(uint8_t c)
{
    if(c == 2)
    {
        R8_SPI1_CTRL_CFG |= RB_SPI_MST_DLY_EN;
    }
    else
    {
        R8_SPI1_CTRL_CFG &= ~RB_SPI_MST_DLY_EN;
    }
    R8_SPI1_CLOCK_DIV = c;
}

/* ***************************************************************************
 * @fn SPI1_DataMode
 *
 * @brief Set data flow mode
 *
 * @param m - Data flow mode refer to ModeBitOrderTypeDef
 *
 * @return none */
void SPI1_DataMode(ModeBitOrderTypeDef m)
{
    switch(m)
    {
        case Mode0_LowBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode0_HighBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        case Mode3_LowBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode3_HighBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        default:
            break;
    }
}

/* ***************************************************************************
 * @fn SPI1_MasterSendByte
 *
 * @brief Send a single byte (buffer)
 *
 * @param d - Send bytes
 *
 * @return none */
void SPI1_MasterSendByte(uint8_t d)
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI1_BUFFER = d;
    while(!(R8_SPI1_INT_FLAG & RB_SPI_FREE));
}

/* ***************************************************************************
 * @fn SPI1_MasterRecvByte
 *
 * @brief Receive single byte (buffer)
 *
 * @param none
 *
 * @return Received bytes */
uint8_t SPI1_MasterRecvByte(void)
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI1_BUFFER = 0xFF; // Start the transfer
    while(!(R8_SPI1_INT_FLAG & RB_SPI_FREE));
    return (R8_SPI1_BUFFER);
}

/* ***************************************************************************
 * @fn SPI1_MasterTrans
 *
 * @brief sends multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data content to be sent
 * @param len - The length of the data requested to send, maximum 4095
 *
 * @return none */
void SPI1_MasterTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR; // Set the data direction to output
    R16_SPI1_TOTAL_CNT = sendlen;         // Set the length of data to be sent
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while(sendlen)
    {
        if(R8_SPI1_FIFO_COUNT < SPI_FIFO_SIZE)
        {
            R8_SPI1_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while(R8_SPI1_FIFO_COUNT != 0); // Wait for all data in FIFO to be sent to complete
}

/* ***************************************************************************
 * @fn SPI1_MasterRecv
 *
 * @brief Receive multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data to be received
 * @param len - The length of data to be received, maximum of 4095
 *
 * @return none */
void SPI1_MasterRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t readlen;

    readlen = len;
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR; // Set the data direction to input
    R16_SPI1_TOTAL_CNT = len;            // Set the length of data to be received. If the FIFO direction is input length not 0, the transmission will be started */
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while(readlen)
    {
        if(R8_SPI1_FIFO_COUNT)
        {
            *pbuf = R8_SPI1_FIFO;
            pbuf++;
            readlen--;
        }
    }
}
