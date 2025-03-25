/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_pwr.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* ***************************************************************************
 * @fn PWR_DCDCCfg
 *
 * @brief Enable internal DC/DC power supply to save system power consumption
 *
 * @param s - Whether to turn on DCDC power
 *
 * @return none */
void PWR_DCDCCfg(FunctionalState s)
{
    uint16_t adj = R16_AUX_POWER_ADJ;
    uint16_t plan = R16_POWER_PLAN;

    if(s == DISABLE)
    {
        
        adj &= ~RB_DCDC_CHARGE;
        plan &= ~(RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE); // Bypass DC/DC
        sys_safe_access_enable();
        R16_AUX_POWER_ADJ = adj;
        R16_POWER_PLAN = plan;
        sys_safe_access_disable();
    }
    else
    {
        uint32_t HW_Data[2];
        FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, ROM_CFG_ADR_HW, HW_Data, 0);
        if((HW_Data[0]) & (1 << 13))
        {
            return;
        }
        adj |= RB_DCDC_CHARGE;
        plan |= RB_PWR_DCDC_PRE;
        sys_safe_access_enable();
        R16_AUX_POWER_ADJ = adj;
        R16_POWER_PLAN = plan;
        sys_safe_access_disable();
        DelayUs(10);
        sys_safe_access_enable();
        R16_POWER_PLAN |= RB_PWR_DCDC_EN;
        sys_safe_access_disable();
    }
}

/* ***************************************************************************
 * @fn PWR_UnitModCfg
 *
 * @brief Power control of controllable unit module
 *
 * @param s - Whether to power on
 * @param unit - please refer to unit of controlled power supply
 *
 * @return none */
void PWR_UnitModCfg(FunctionalState s, uint8_t unit)
{
    uint8_t ck32k_cfg = R8_CK32K_CONFIG;

    if(s == DISABLE) // closure
    {
        ck32k_cfg &= ~(unit & 0x03);
    }
    else // Open
    {
        ck32k_cfg |= (unit & 0x03);
    }

    sys_safe_access_enable();
    R8_CK32K_CONFIG = ck32k_cfg;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn PWR_SafeClkCfg
 *
 * @brief Safe Access Clock Control Bit
 *
 * @param s - Whether to turn on the corresponding peripheral clock
 * @param perph - please refer to SAFE CLK control bit define
 *
 * @return none */
void PWR_SafeClkCfg(FunctionalState s, uint16_t perph)
{
    uint32_t sleep_ctrl = R8_SAFE_CLK_CTRL;

    if(s == DISABLE)
    {
        sleep_ctrl |= perph;
    }
    else
    {
        sleep_ctrl &= ~perph;
    }

    sys_safe_access_enable();
    R8_SAFE_CLK_CTRL = sleep_ctrl;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn PWR_PeriphClkCfg
 *
 * @brief Peripheral clock control bit
 *
 * @param s - Whether to turn on the corresponding peripheral clock
 * @param perph - please refer to Peripher CLK control bit define
 *
 * @return none */
void PWR_PeriphClkCfg(FunctionalState s, uint16_t perph)
{
    uint32_t sleep_ctrl = R32_SLEEP_CONTROL;

    if(s == DISABLE)
    {
        sleep_ctrl |= perph;
    }
    else
    {
        sleep_ctrl &= ~perph;
    }

    sys_safe_access_enable();
    R32_SLEEP_CONTROL = sleep_ctrl;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn PWR_PeriphWakeUpCfg
 *
 * @brief Sleep wake source configuration
 *
 * @param s - Whether to turn on the sleep wake-up function of this peripheral
 * @param perph - Wake source that needs to be set
 * RB_SLP_USB_WAKE - USBFS is the wake-up source
 * RB_SLP_USB2_WAKE - USBHS is the wake-up source
 * RB_SLP_RTC_WAKE - RTC is the wake-up source
 * RB_SLP_GPIO_WAKE - GPIO is the wake-up source
 * RB_SLP_BAT_WAKE - BAT is the wake-up source
 * RB_GPIO_EDGE_WAKE - GPIO can wake up regardless of the upper or lower edges
 * @param mode - refer to WakeUP_ModeypeDef
 *
 * @return none */
void PWR_PeriphWakeUpCfg(FunctionalState s, uint8_t perph, WakeUP_ModeypeDef mode)
{
    uint8_t m;

    if(s == DISABLE)
    {
        sys_safe_access_enable();
        R8_SLP_WAKE_CTRL &= ~perph;
        sys_safe_access_disable();
    }
    else
    {
        switch(mode)
        {
            case Short_Delay:
                m = 0x01;
                break;

            case Long_Delay:
                m = 0x00;
                break;

            default:
                m = 0x01;
                break;
        }

        sys_safe_access_enable();
        R8_SLP_WAKE_CTRL |= RB_WAKE_EV_MODE | perph;
        sys_safe_access_disable();
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
        sys_safe_access_disable();
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= m;
        sys_safe_access_disable();
    }
}

/* ***************************************************************************
 * @fn PowerMonitor
 *
 * @brief Power Monitoring
 *
 * @param s - Whether to turn on this feature
 * @param vl - refer to VolM_LevelypeDef
 *
 * @return none */
void PowerMonitor(FunctionalState s, VolM_LevelypeDef vl)
{
    uint8_t ctrl = R8_BAT_DET_CTRL;
    uint8_t cfg = R8_BAT_DET_CFG;

    if(s == DISABLE)
    {
        sys_safe_access_enable();
        R8_BAT_DET_CTRL = 0;
        sys_safe_access_disable();
    }
    else
    {
        if(vl & 0x80)
        {
            cfg = vl & 0x03;
            ctrl = RB_BAT_MON_EN | ((vl >> 2) & 1);
        }
        else
        {
            
            cfg = vl & 0x03;
            ctrl = RB_BAT_DET_EN;
        }
        sys_safe_access_enable();
        R8_BAT_DET_CTRL = ctrl;
        R8_BAT_DET_CFG = cfg;
        sys_safe_access_disable();

        mDelayuS(1);
        sys_safe_access_enable();
        R8_BAT_DET_CTRL |= RB_BAT_LOW_IE | RB_BAT_LOWER_IE;
        sys_safe_access_disable();
    }
}

/* ***************************************************************************
 * @fn LowPower_Idle
 *
 * @brief low power consumption - Idle mode
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
void LowPower_Idle(void)
{
    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; // flash close

    PFIC->SCTLR &= ~(1 << 2); // sleep
    __WFI();
    __nop();
    __nop();
}

/* ***************************************************************************
 * @fn LowPower_Halt
 *
 * @brief Low power consumption - Halt mode, this low power consumption cuts to the HSI/5 clock operation, and after wake-up, the user needs to re-select the system clock source by himself
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
void LowPower_Halt(void)
{
    uint8_t x32Mpw;

    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; // flash close
    x32Mpw = R8_XT32M_TUNE;
    if(!(R8_HFCK_PWR_CTRL&RB_CLK_XT32M_KEEP))
    {
        x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150% rated current
    }

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // Turn off voltage monitoring
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_PLL_CONFIG |= (1 << 5);
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep
    __WFI();
    __nop();
    __nop();
    sys_safe_access_enable();
    R8_PLL_CONFIG &= ~(1 << 5);
    sys_safe_access_disable();
}

/* *********************************************************************************************
* Function Name: LowPower_Sleep
* Description: Low power consumption-Sleep mode.
* Input : rm:
                    RB_PWR_RAM32K - 32K retention SRAM power supply
                    RB_PWR_RAM96K - 96K main SRAM powered
                    RB_PWR_EXTEND - USB and BLE units reserved area power supply
                    RB_PWR_XROM - FlashROM Powered
                   NULL - All of the above units are powered off
* Return : None
********************************************************************************************* */
__HIGH_CODE
void LowPower_Sleep(uint16_t rm)
{
    __attribute__((aligned(4))) uint8_t MacAddr[6] = {0};
    uint8_t x32Mpw;
    uint16_t power_plan;
    uint16_t clk_sys_cfg;
    uint16_t hfck_pwr_ctrl;

    GetMACAddress(MacAddr);

    clk_sys_cfg = R16_CLK_SYS_CFG;
    hfck_pwr_ctrl = R8_HFCK_PWR_CTRL;
    x32Mpw = R8_XT32M_TUNE;
    x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150% rated current

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // Turn off voltage monitoring
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();

    sys_safe_access_enable();
    R16_POWER_PLAN &= ~RB_XT_PRE_EN;
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep

    power_plan = R16_POWER_PLAN & (RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE);
    power_plan |= RB_PWR_PLAN_EN | RB_PWR_CORE | rm | (2<<11);

    sys_safe_access_enable();
    if(rm & RB_XT_PRE_EN)
    {
        R8_SLP_POWER_CTRL |= 0x41;
    }
    else
    {
        R8_SLP_POWER_CTRL |= 0x40;
    }
    R16_POWER_PLAN = power_plan;
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;   // Sleeping requires turning on the internal HSI before sleeping
    sys_safe_access_disable();
    if((R16_CLK_SYS_CFG & RB_CLK_SYS_MOD) == 0x40)
    {
        sys_safe_access_enable();
        R16_CLK_SYS_CFG = (R16_CLK_SYS_CFG&(~RB_CLK_PLL_DIV))|24;
        sys_safe_access_disable();
    }
//    sys_safe_access_enable();
//    R8_PLL_CONFIG |= (1 << 5);
//    sys_safe_access_disable();

    __WFI();
    __nop();
    __nop();

    sys_safe_access_enable();
    R16_CLK_SYS_CFG = clk_sys_cfg;
    R8_HFCK_PWR_CTRL = hfck_pwr_ctrl;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R16_POWER_PLAN &= ~RB_PWR_PLAN_EN;
    sys_safe_access_disable();

    sys_safe_access_enable();
    R16_POWER_PLAN &= ~RB_XT_PRE_EN;
    sys_safe_access_disable();

//    sys_safe_access_enable();
//    R8_PLL_CONFIG &= ~(1 << 5);
//    sys_safe_access_disable();
    DelayUs(40);
}

/* ***************************************************************************
 * @fn LowPower_Shutdown
 *
 * @brief Low power consumption - Shutdown mode, this low power consumption cuts to the HSI/5 clock operation, and after wake-up, the user needs to re-select the system clock source by himself
 * @note Note: Call this function, the DCDC function is forced to be closed, and it can be manually turned on again after wake-up.
 *
 * @param rm - Power supply module selection
 * RB_PWR_RAM32K - 32K retention SRAM powered
 * RB_PWR_RAM96K - 96K main SRAM powered
 * RB_PWR_EXTEND - Reserved area power supply for USB and BLE units
 * NULL - All the above units are powered off
 *
 * @return none */
__HIGH_CODE
void LowPower_Shutdown(uint16_t rm)
{
    uint8_t x32Kpw, x32Mpw;

    FLASH_ROM_SW_RESET();
    x32Kpw = R8_XT32K_TUNE;
    x32Mpw = R8_XT32M_TUNE;
    x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150% rated current
    x32Kpw = (x32Kpw & 0xfc) | 0x01; // LSE drive current is reduced to rated current

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // Turn off voltage monitoring
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32K_TUNE = x32Kpw;
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep

    sys_safe_access_enable();
    R8_SLP_POWER_CTRL |= 0x40;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R16_POWER_PLAN = RB_PWR_PLAN_EN | rm;
    sys_safe_access_disable();
    __WFI();
    __nop();
    __nop();
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}
