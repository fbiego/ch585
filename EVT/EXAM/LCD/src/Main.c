/* ********************************* (C) COPYRIGHT ***************************
 * File Name: Main.c
 * Author: WCH
 * Version: V1.0
 * Date: 2023/02/24
 * Description: LCD demonstration
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#include "CH58x_common.h"
#include "CH58x_lcd.h"

uint16_t aux_power;
unsigned char const lcd[14]={0x7d, 0x60, 0x3e, 0x7a, 0x63, 0x5b, 0x5f, 0x70, 0x03, 0x9b, 0x7d, 0x60, 0x3e, 0x7a,};
/*     A
     |----|
    F|    |B
     |-G--|
    E|    |C
     |----| .P
       D
*/
/* Note: Using this routine, the external manual reset function must be turned off when downloading. */
int main()
{
    uint32_t VER = 0;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    LCD_Init(LCD_1_4_Duty, LCD_1_3_Bias);

    LCD_WriteData0( lcd[0] );
    LCD_WriteData1( lcd[1] );
    LCD_WriteData2( lcd[2] );
    LCD_WriteData3( lcd[3] );
    LCD_WriteData4( lcd[4] );
    LCD_WriteData5( lcd[5] );
    LCD_WriteData6( lcd[6] );
    LCD_WriteData7( lcd[7] );
    LCD_WriteData8( lcd[8] );
    LCD_WriteData9( lcd[9] );
    LCD_WriteData10( lcd[10] );
    LCD_WriteData11( lcd[11] );
    LCD_WriteData12( lcd[12] );
    LCD_WriteData13( lcd[13] );


    /* LCD + sleep example */
#if 1
    /* Configure wakeup source as GPIO - PA5 */
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_5, GPIO_ITMode_FallEdge); // Wake up on the falling edge
    PFIC_EnableIRQ(GPIO_A_IRQn);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
    LowPower_Sleep(RB_PWR_RAM32K | RB_PWR_RAM96K | RB_XT_PRE_EN); // Only 96+32K SRAM power supply is retained
    HSECFG_Current(HSE_RCur_100);                 // Reduced to rated current (HSE bias current is increased in low power consumption function)
#endif

    while(1);

}

/* ***************************************************************************
 * @fn GPIOA_IRQHandler
 *
 * @brief GPIOA interrupt function
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void)
{
    GPIOA_ClearITFlagBit(GPIO_Pin_5);
}
