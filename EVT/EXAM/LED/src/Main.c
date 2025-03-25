/* ********************************* (C) COPYRIGHT ***************************
* File Name: Main.c
* Author: WCH
* Version: V1.0
* Date: 2024/11/20
* Description: LED example
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#include "CH58x_common.h"
#include "ch58x_drv_ledc.h"

__attribute__((__aligned__(4))) uint32_t tx_data[8] = {0x01020408,0x10204080,0x03,0x04,0x05,0x06,0x07,0x08};

#define  LSB_HSB         0           // LED serial data bit sequence, 1: High bit is ahead; 0: Low bit is ahead
#define  POLAR           0           // LED data output polarity, 0: passthrough, data 0 output 0, data 1 output 1; 1 is inverting

/* ***************************************************************************
 * @fn DebugInit
 *
 * @brief debug initialization
 *
 * @return none */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
}

/* ***************************************************************************
 * @fn main
 *
 * @brief main function
 *
 * @return none */
int main()
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    /* Configure serial debugging */
    DebugInit();
    PRINT( "Start @ChipID=%02X\n", R8_CHIP_ID );

    //led clk
    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeOut_PP_5mA );

    //led data
    //LED 0
    GPIOA_ModeCfg( GPIO_Pin_0, GPIO_ModeOut_PP_5mA );
    //LED 1
    GPIOA_ModeCfg( GPIO_Pin_1, GPIO_ModeOut_PP_5mA );
    //LED 2
    GPIOA_ModeCfg( GPIO_Pin_2, GPIO_ModeOut_PP_5mA );
    //LED 3
    GPIOA_ModeCfg( GPIO_Pin_3, GPIO_ModeOut_PP_5mA );
    //lED 4
    GPIOA_ModeCfg( GPIO_Pin_5 , GPIO_ModeOut_PP_5mA );
    //lED 5
    GPIOA_ModeCfg( GPIO_Pin_6 , GPIO_ModeOut_PP_5mA );
    //lED 6
    GPIOA_ModeCfg( GPIO_Pin_7 , GPIO_ModeOut_PP_5mA );
    //lED 7
    GPIOA_ModeCfg( GPIO_Pin_8 , GPIO_ModeOut_PP_5mA );


    // Configure frequency division and mode selection
    ch58x_led_controller_init(CH58X_LED_OUT_MODE_FOUR_EXT, 128);

    // Start sending, and then sending it in the interrupt
    R32_LED_DMA_BEG = ((uint32_t)(tx_data)& RB_LED_DMA_BEG);
    R16_LED_DMA_LEN = 2;
    R8_LED_CTRL_MOD |= RB_LED_DMA_EN;

#if LSB_HSB   //LSB HSB
    R8_LED_CTRL_MOD ^= RB_LED_BIT_ORDER;
#endif

#if POLAR     // polarity
    R8_LED_CTRL_MOD ^= RB_LED_OUT_POLAR;
#endif

    LED_ENABLE();
    PFIC_EnableIRQ(LED_IRQn);

    while(1);
}

/* ***************************************************************************
 * @fn LED_IRQHandler
 *
 * @brief LED interrupt function
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
void LED_IRQHandler(void)
{
    // Clear the interrupt sign
    uint16_t LED_status;
    LED_status = R16_LED_STATUS;
    R16_LED_STATUS = LED_status;

    ch58x_led_controller_send(tx_data, 2);
}
