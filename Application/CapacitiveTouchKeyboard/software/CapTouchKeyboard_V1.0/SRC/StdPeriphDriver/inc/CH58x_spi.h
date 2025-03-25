/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_SPI.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH58x_SPI_H__
#define __CH58x_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SPI0 interrupt bit define
 */
#define SPI0_IT_FST_BYTE    RB_SPI_IE_FST_BYTE  // In the first byte command mode of slave mode, first byte interrupt is received
#define SPI0_IT_FIFO_OV     RB_SPI_IE_FIFO_OV   // FIFO Overflow
#define SPI0_IT_DMA_END     RB_SPI_IE_DMA_END   // DMA transmission ends
#define SPI0_IT_FIFO_HF     RB_SPI_IE_FIFO_HF   // FIFO has been used for more than half
#define SPI0_IT_BYTE_END    RB_SPI_IE_BYTE_END  // Single-byte transmission is completed
#define SPI0_IT_CNT_END     RB_SPI_IE_CNT_END   // All byte transfer is completed

/**
 * @brief  Configuration data mode
 */
typedef enum
{
    Mode0_LowBitINFront = 0, // Mode 0, low position in front
    Mode0_HighBitINFront,    // Mode 0, high position in front
    Mode3_LowBitINFront,     // Mode 3, low position in front
    Mode3_HighBitINFront,    // Mode 3, high position in front
} ModeBitOrderTypeDef;

/**
 * @brief  Configuration SPI0 slave mode
 */
typedef enum
{
    Mode_DataStream = 0, // Data flow mode
    Mose_FirstCmd,       // First-byte command mode
} Slave_ModeTypeDef;

/* *
 * @brief Host mode default initialization: Mode 0+3 line full duplex + 8MHz */
void SPI0_MasterDefInit(void);

/* *
 * @brief SPI0 reference clock configuration, = d*Tsys
 *
 * @param c - clock frequency division coefficient */
void SPI0_CLKCfg(uint8_t c);

/* *
 * @brief Set data flow mode
 *
 * @param m - Data flow mode refer to ModeBitOrderTypeDef */
void SPI0_DataMode(ModeBitOrderTypeDef m);

/* *
 * @brief Send a single byte (buffer)
 *
 * @param d - Send bytes */
void SPI0_MasterSendByte(uint8_t d);

/* *
 * @brief Receive single byte (buffer)
 *
 * @param none */
uint8_t SPI0_MasterRecvByte(void);

/* *
 * @brief sends multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data content to be sent
 * @param len - The length of the data requested to send, maximum 4095 */
void SPI0_MasterTrans(uint8_t *pbuf, uint16_t len);

/* *
 * @brief Receive multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data to be received
 * @param len - The length of data to be received, maximum of 4095 */
void SPI0_MasterRecv(uint8_t *pbuf, uint16_t len);

/* *
 * @brief data is sent continuously in DMA mode
 *
 * @param pbuf - The starting address of data to be sent, four bytes are required to
 * @param len - length of data to be sent */
void SPI0_MasterDMATrans(uint8_t *pbuf, uint16_t len);

/* *
 * @brief DMA mode continuously receives data
 *
 * @param pbuf - The starting address of data to be received, four bytes are required to store it
 * @param len - length of data to be received */
void SPI0_MasterDMARecv(uint8_t *pbuf, uint16_t len);

/* *
 * @brief Host mode default initialization: Mode 0+3 line full duplex + 8MHz */
void SPI1_MasterDefInit(void);

/* *
 * @brief SPI1 reference clock configuration, = d*Tsys
 *
 * @param c - clock frequency division coefficient */
void SPI1_CLKCfg(UINT8 c);

/* *
 * @brief Set data flow mode
 *
 * @param m - Data flow mode refer to ModeBitOrderTypeDef */
void SPI1_DataMode(ModeBitOrderTypeDef m);

/* *
 * @brief Send a single byte (buffer)
 *
 * @param d - Send bytes */
void  SPI1_MasterSendByte(UINT8 d);

/* *
 * @brief Receive single byte (buffer)
 *
 * @param none */
UINT8 SPI1_MasterRecvByte(void);

/* *
 * @brief sends multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data content to be sent
 * @param len - The length of the data requested to send, maximum 4095 */
void SPI1_MasterTrans(UINT8 *pbuf, UINT16 len);

/* *
 * @brief Receive multibytes continuously using FIFO
 *
 * @param pbuf - The first address of the data to be received
 * @param len - The length of data to be received, maximum of 4095 */
void SPI1_MasterRecv(UINT8 *pbuf, UINT16 len);

/* *
 * @brief The device mode is initialized by default. It is recommended to set the GPIO of MISO to the input mode. */
void SPI0_SlaveInit(void);

/* *
 * @brief loads first byte data content
 *
 * @param d - first byte data content */
#define SetFirstData(d)    (R8_SPI0_SLAVE_PRE = d)

/* *
 * @brief slave mode, send one byte of data
 *
 * @param d - Data to be sent */
void SPI0_SlaveSendByte(uint8_t d);

/* *
 * @brief slave mode, receive one byte of data
 *
 * @return Received data */
uint8_t SPI0_SlaveRecvByte(void);

/* *
 * @brief slave mode, send multibyte data
 *
 * @param pbuf - The first address of the data content to be sent
 * @param len - The length of the data requested to send, maximum 4095 */
void SPI0_SlaveTrans(uint8_t *pbuf, uint16_t len);

/* *
 * @brief slave mode, receive multibyte data
 *
 * @param pbuf - Start address for receiving and receiving data storage
 * @param len - Request received data length */
void SPI0_SlaveRecv(uint8_t *pbuf, uint16_t len);

/* *
 * @brief data is sent continuously in DMA mode
 *
 * @param pbuf - The starting address of data to be sent, four bytes are required to
 * @param len - length of data to be sent */
void SPI0_SlaveDMATrans(uint8_t *pbuf, uint16_t len);

/* *
 * @brief DMA mode continuously receives data
 *
 * @param pbuf - The starting address of data to be received, four bytes are required to store it
 * @param len - length of data to be received */
void SPI0_SlaveDMARecv(uint8_t *pbuf, uint16_t len);

/* *
 * @brief Configure SPI0 interrupt
 *
 * @param s - Enable/Close
 * @param f - refer to SPI0 interrupt bit define */
#define SPI0_ITCfg(s, f)       ((s) ? (R8_SPI0_INTER_EN |= f) : (R8_SPI0_INTER_EN &= ~f))

/* *
 * @brief Get interrupt flag status, 0-not set, (!0)-triggered
 *
 * @param f - refer to SPI0 interrupt bit define */
#define SPI0_GetITFlag(f)      (R8_SPI0_INT_FLAG & f)

/* *
 * @brief Clear the current interrupt flag
 *
 * @param f - refer to SPI0 interrupt bit define */
#define SPI0_ClearITFlag(f)    (R8_SPI0_INT_FLAG = f)

/* *
 * @brief Close SPI0 */
#define SPI0_Disable()         (R8_SPI0_CTRL_MOD &= ~(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_MISO_OE))

#ifdef __cplusplus
}
#endif

#endif // __CH58x_SPI_H__
