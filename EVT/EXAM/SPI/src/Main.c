/* ********************************* (C) COPYRIGHT ***************************
 * File Name: Main.c
 * Author: WCH
 * Version: V1.0
 * Date: 2020/08/06
 * Description: SPI0 demonstration Master/Slave mode data sending and receiving
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#include "CH58x_common.h"

__attribute__((aligned(4))) uint8_t spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) uint8_t spiBuffrev[16];

void DebugInit(void)                                  // SPI0 conflicts with the routine's idiomatic serial port printing pin, so this routine uses UART1_output printing instead.
{
    GPIOB_SetBits(GPIO_Pin_13);
    GPIOPinRemap(ENABLE, RB_PIN_UART1);
    GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
}

int main()
{
    uint8_t i;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

    /* Configure serial debugging */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if 1
    /* Host Mode */
    PRINT("1.spi0 mul master mode send data ...\n");
    DelayMs(100);

    #if 1
        /* SPI 0 */
        GPIOA_SetBits(GPIO_Pin_12);
        GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
        SPI0_MasterDefInit();

        // Single byte send
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI0_MasterSendByte(0x55);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(1);
        GPIOA_ResetBits(GPIO_Pin_12);
        i = SPI0_MasterRecvByte();
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        PRINT("receive %x\n",i);

        // FIFO continuous sending
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI0_MasterTrans(spiBuff, 8);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI0_MasterRecv(spiBuffrev, 8);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        PRINT("FIFO recv ");
        for(i = 0; i < 8; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");


        // DMA continuous sending
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI0_MasterDMATrans(spiBuff, 12);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI0_MasterDMARecv(spiBuffrev, 12);
        GPIOA_SetBits(GPIO_Pin_12);
        PRINT("DMA recv ");
        for(i = 0; i < 12; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");
    #else
        /* SPI 1 only supports CH585 */
        GPIOA_SetBits(GPIO_Pin_12);
        GPIOA_ModeCfg(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12, GPIO_ModeOut_PP_5mA);
        SPI1_MasterDefInit();

        // Single byte send
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI1_MasterSendByte(0x55);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(1);
        GPIOA_ResetBits(GPIO_Pin_12);
        i = SPI1_MasterRecvByte();
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        PRINT("receive %x\n",i);

        // FIFO continuous sending
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI1_MasterTrans(spiBuff, 8);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        GPIOA_ResetBits(GPIO_Pin_12);
        SPI1_MasterRecv(spiBuffrev, 8);
        GPIOA_SetBits(GPIO_Pin_12);
        DelayMs(2);
        PRINT("FIFO recv ");
        for(i = 0; i < 8; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");
    #endif
    PRINT("END ...\n");
    while(1);
#endif

#if 0
    /* Device Mode */
    PRINT("1.spi0 mul slave mode \n");
    GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_ModeIN_PU);
    SPI0_SlaveInit();
    i = SPI0_SlaveRecvByte();
    SPI0_SlaveSendByte(~i);
    PRINT("receive %x\n",i);

    SPI0_SlaveRecv(spiBuffrev, 8);
    SPI0_SlaveTrans(spiBuffrev, 8);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    SPI0_SlaveDMARecv(spiBuffrev, 12);
    SPI0_SlaveDMATrans(spiBuffrev, 12);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    PRINT("END ...\n");

    while(1);
#endif

    while(1);
}
