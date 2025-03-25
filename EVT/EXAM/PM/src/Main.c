/* ********************************* (C) COPYRIGHT ***************************
 * File Name: Main.c
 * Author: WCH
 * Version: V1.0
 * Date: 2020/08/06
 * Description: System sleep mode and wake up demonstration: GPIOA_5 is used as the wake-up source, with a total of 4 sleep levels
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/* Note: Switch to the HSE clock source, the required waiting stability time is related to the selected external crystal parameters. It is best to choose a new crystal. The crystal provided by the manufacturer and its
 Load capacitance parameter value. By configuring the R8_XT32M_TUNE register, different load capacitances and bias currents can be configured to adjust the crystal stability time. */

#include "CH58x_common.h"

#define CLK_PER_US                  (1.0 / ((1.0 / CAB_LSIFQ) * 1000 * 1000))

#define US_PER_CLK                  (1.0 / CLK_PER_US)

#define RTC_TO_US(clk)              ((uint32_t)((clk) * US_PER_CLK + 0.5))

#define US_TO_RTC(us)               ((uint32_t)((us) * CLK_PER_US + 0.5))

void PM_LowPower_Sleep(void);
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
    PWR_DCDCCfg(ENABLE);
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);

    /* Configure serial debugging */
    DebugInit();
    PRINT("Start @ChipID=%02x\n", R8_CHIP_ID);
    DelayMs(200);

#if 1
    /* Configure wakeup source as GPIO - PA5 */
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_5, GPIO_ITMode_FallEdge); // Wake up on the falling edge
    PFIC_EnableIRQ(GPIO_A_IRQn);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
#endif

#if 1
    PRINT("IDLE mode sleep \n");
    DelayMs(1);
    LowPower_Idle();
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("Halt mode sleep \n");
    DelayMs(2);
    LowPower_Halt();
    HSECFG_Current(HSE_RCur_100); // Reduced to rated current (HSE bias current is increased in low power consumption function)
    DelayMs(2);
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("sleep mode sleep \n");
    DelayMs(2);
    PM_LowPower_Sleep();
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("shut down mode sleep \n");
    DelayMs(2);
    LowPower_Shutdown(0); // All power off, reset after wake-up
    /* Reset will be performed after this mode wakes up, so the following code will not run.
     Be careful to make sure that the system sleeps and wakes up before wakes up, otherwise it may become IDLE level wake-up. */
    HSECFG_Current(HSE_RCur_100); // Reduced to rated current (HSE bias current is increased in low power consumption function)
    PRINT("wake.. \n");
    DelayMs(500);
#endif

    while(1)
        ;
}

/* ***************************************************************************
 * @fn LowPowerGapProcess
 *
 * @brief Execution during unstable external clock, which can be used to perform processing that requires low clock requirements
 *
 * @return none */
__HIGH_CODE
void LowPowerGapProcess()
{
    PRINT("LowPowerGapProcess.. \n");
}

/* ***************************************************************************
 * @fn PM_LowPower_Sleep
 *
 * @brief GPIOA interrupt function
 *
 * @return none */
__HIGH_CODE
void PM_LowPower_Sleep(void)
{
    uint32_t t;
    uint8_t wake_ctrl;
    unsigned long irq_status;

    // Switch the internal clock
    sys_safe_access_enable();
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
    R16_CLK_SYS_CFG &= ~RB_OSC32M_SEL;
    sys_safe_access_disable();
    LowPower_Sleep(RB_PWR_RAM96K | RB_PWR_RAM32K ); // Only 96+32K SRAM power supply is retained
    SYS_DisableAllIrq(&irq_status);
    wake_ctrl = R8_SLP_WAKE_CTRL;
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = RB_WAKE_EV_MODE | RB_SLP_RTC_WAKE; // RTC wake up
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;  // Trigger mode
    sys_safe_access_disable();
    t = RTC_GetCycle32k() + US_TO_RTC(1600);
    if(t > RTC_MAX_COUNT)
    {
        t -= RTC_MAX_COUNT;
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();
    LowPowerGapProcess();
    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; // flash close

    PFIC->SCTLR &= ~(1 << 2); // sleep
    __WFE();
    __nop();
    __nop();
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = wake_ctrl;
    sys_safe_access_disable();
    HSECFG_Current(HSE_RCur_100); // Reduced to rated current (HSE bias current is increased in low power consumption function)
    // Switch external clock
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    SYS_RecoverIrq(irq_status);

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
