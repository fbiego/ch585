/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_clk.c
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
 * @fn LClk32K_Select
 *
 * @brief 32K low frequency clock source
 *
 * @param hc - Choose 32K to use internal or external
 *
 * @return none */
void LClk32K_Select(LClk32KTypeDef hc)
{
    uint8_t cfg = R8_CK32K_CONFIG;

    if(hc == Clk32K_LSI)
    {
        cfg &= ~RB_CLK_OSC32K_XT;
        LSECFG_Current(LSE_RCur_100);
    }
    else
    {
        cfg |= RB_CLK_OSC32K_XT;
    }

    sys_safe_access_enable();
    R8_CK32K_CONFIG = cfg;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn LClk32K_Cfg
 *
 * @brief 32K low frequency clock power supply configuration
 *
 * @param hc - Choose internal 32K or external 32K
 * @param s - Whether to power on
 *
 * @return none */
void LClk32K_Cfg(LClk32KTypeDef hc, FunctionalState s)
{
    uint8_t cfg = R8_CK32K_CONFIG;

    if(hc == Clk32K_LSI)
    {
        if(s == DISABLE)
        {
            cfg &= ~RB_CLK_INT32K_PON;
        }
        else
        {
            cfg |= RB_CLK_INT32K_PON;
        }
    }
    else
    {
        if(s == DISABLE)
        {
            cfg &= ~RB_CLK_XT32K_PON;
        }
        else
        {
            cfg |= RB_CLK_XT32K_PON;
        }
    }

    sys_safe_access_enable();
    R8_CK32K_CONFIG = cfg;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn HSECFG_Current
 *
 * @brief HSE crystal Bias current configuration
 *
 * @param c - 75%, 100%, 125%, 150%
 *
 * @return none */
void HSECFG_Current(HSECurrentTypeDef c)
{
    uint8_t x32M_c;

    x32M_c = R8_XT32M_TUNE;
    x32M_c = (x32M_c & 0xfc) | (c & 0x03);

    sys_safe_access_enable();
    R8_XT32M_TUNE = x32M_c;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn HSECFG_Capacitance
 *
 * @brief HSE crystal Load capacitor configuration
 *
 * @param c - refer to HSECapTypeDef
 *
 * @return none */
void HSECFG_Capacitance(HSECapTypeDef c)
{
    uint8_t x32M_c;

    x32M_c = R8_XT32M_TUNE;
    x32M_c = (x32M_c & 0x0f) | (c << 4);

    sys_safe_access_enable();
    R8_XT32M_TUNE = x32M_c;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn LSICFG_Current
 *
 * @brief LSI crystal Bias current configuration
 *
 * @param c - 70%, 100%, 140%, 200%
 *
 * @return none */
void LSICFG_Current(LSICurrentTypeDef c)
{
    uint8_t x32K_c;

    x32K_c = R8_XT32K_TUNE;
    x32K_c = (x32K_c & 0xf3) | ((c<<2) & 0x0C);

    sys_safe_access_enable();
    R8_XT32K_TUNE = x32K_c;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn LSECFG_Current
 *
 * @brief LSE crystal Bias current configuration
 *
 * @param c - 70%, 100%, 140%, 200%
 *
 * @return none */
void LSECFG_Current(LSECurrentTypeDef c)
{
    uint8_t x32K_c;

    x32K_c = R8_XT32K_TUNE;
    x32K_c = (x32K_c & 0xfc) | (c & 0x03);

    sys_safe_access_enable();
    R8_XT32K_TUNE = x32K_c;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn LSECFG_Capacitance
 *
 * @brief LSE crystal Load capacitor configuration
 *
 * @param c - refer to LSECapTypeDef
 *
 * @return none */
void LSECFG_Capacitance(LSECapTypeDef c)
{
    uint8_t x32K_c;

    x32K_c = R8_XT32K_TUNE;
    x32K_c = (x32K_c & 0x0f) | (c << 4);

    sys_safe_access_enable();
    R8_XT32K_TUNE = x32K_c;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn Calibration_LSI
 *
 * @brief Calibrate internal 32K clock
 *
 * @param cali_Lv - Calibration level selection Level_32: 2.4ms 1000ppm (32M main frequency) 1100ppm (60M main frequency)
 * Level_64: 4.4ms 800ppm (32M main frequency) 1000ppm (60M main frequency)
 * Level_128: 8.4ms 600ppm (32M main frequency) 800ppm (60M main frequency)
 *
 * @return none */
void Calibration_LSI(Cali_LevelTypeDef cali_Lv)
{
    UINT64 i;
    long long cnt_offset;
    UINT8  retry = 0;
    UINT8  retry_all = 0;
    INT32  freq_sys;
    UINT32 cnt_32k = 0;
    UINT32 irqv = 0;

    freq_sys = GetSysClock();

    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~RB_CLK_OSC32K_FILT;
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_FILT;
    sys_safe_access_disable();

    while(1)
    {
        // Coarse adjustment
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_TOTAL;
        R8_OSC_CAL_CTRL |= 1;
        sys_safe_access_disable();

        while(1)
        {
            sys_safe_access_enable();
            R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
            R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
            R16_OSC_CAL_CNT |= RB_OSC_CAL_IF;
            sys_safe_access_disable();
            while( (R8_OSC_CAL_CTRL & RB_OSC_CNT_EN) == 0 )
            {
                sys_safe_access_enable();
                R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
                sys_safe_access_disable();
            }

            while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT)); // For discarding

            SYS_DisableAllIrq(&irqv);
            sys_safe_access_enable();
            R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_EN;
            R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
            R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
            R16_OSC_CAL_CNT |= RB_OSC_CAL_IF;
            sys_safe_access_disable();
            while( (R8_OSC_CAL_CTRL & RB_OSC_CNT_EN) == 0 )
            {
                sys_safe_access_enable();
                R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
                sys_safe_access_disable();
            }

            while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
            cnt_32k = RTC_GetCycle32k();
            while(RTC_GetCycle32k() == cnt_32k);
            R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
            SYS_RecoverIrq(irqv);
            while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
            i = R16_OSC_CAL_CNT; // Sampling values ​​after real-time calibration
            cnt_offset = (i & 0x3FFF) + R8_OSC_CAL_OV_CNT * 0x3FFF - 2000 * (freq_sys / 1000) / CAB_LSIFQ;
            if(((cnt_offset > -35 * (freq_sys / 1000) / 60000) && (cnt_offset < 35 * (freq_sys / 1000) / 60000)) || retry > 2)
            {
                if(retry)
                {
                    break;
                }
            }
            retry++;
            cnt_offset = (cnt_offset > 0) ? (((cnt_offset * 2) / (70 * (freq_sys/1000) / 60000)) + 1) / 2 : (((cnt_offset * 2) / (70 * (freq_sys/1000) / 60000 )) - 1) / 2;
            sys_safe_access_enable();
            R16_INT32K_TUNE += cnt_offset;
            sys_safe_access_disable();
        }

        // Fine adjustment
        // After configuring the parameters carefully, discard the capture value twice (software behavior) and judge that it has been once, only once is left here.
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_TOTAL;
        R8_OSC_CAL_CTRL |= cali_Lv;
        sys_safe_access_disable();
        while( (R8_OSC_CAL_CTRL & RB_OSC_CNT_TOTAL) != cali_Lv )
        {
            sys_safe_access_enable();
            R8_OSC_CAL_CTRL |= cali_Lv;
            sys_safe_access_disable();
        }

        sys_safe_access_enable();
        R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_EN;
        R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
        R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
        R16_OSC_CAL_CNT |= RB_OSC_CAL_IF;
        sys_safe_access_disable();
        while( (R8_OSC_CAL_CTRL & RB_OSC_CNT_EN) == 0 )
        {
            sys_safe_access_enable();
            R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
            sys_safe_access_disable();
        }

        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT)); // For discarding

        SYS_DisableAllIrq(&irqv);
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_EN;
        R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
        R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
        R16_OSC_CAL_CNT |= RB_OSC_CAL_IF;
        sys_safe_access_disable();
        while( (R8_OSC_CAL_CTRL & RB_OSC_CNT_EN) == 0 )
        {
            sys_safe_access_enable();
            R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
            sys_safe_access_disable();
        }

        while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
        cnt_32k = RTC_GetCycle32k();
        while(RTC_GetCycle32k() == cnt_32k);
        R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
        SYS_RecoverIrq(irqv);
        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_EN;
        sys_safe_access_disable();
        i = R16_OSC_CAL_CNT; // Sampling values ​​after real-time calibration
        cnt_offset = (i & 0x3FFF) + R8_OSC_CAL_OV_CNT * 0x3FFF -  4000 * (1 << ((cali_Lv<=Level_128)?cali_Lv:(cali_Lv+2))) * (freq_sys / 100000) / 256 * 100/(CAB_LSIFQ/256);
        cnt_offset = (cnt_offset > 0) ? ((((cnt_offset * 2*(100 )) / (1141 * ((1 << ((cali_Lv<=Level_128)?cali_Lv:(cali_Lv+2)))/8) * (freq_sys/1000) / 60000)) + 1) / 2) : ((((cnt_offset * 2*(100)) / (1141 * ((1 << ((cali_Lv<=Level_128)?cali_Lv:(cali_Lv+2)))/8) * (freq_sys/1000) / 60000)) - 1) / 2);
        if((cnt_offset > 0)&&(((R16_INT32K_TUNE>>5)+cnt_offset)>0xFF))
        {
            if(retry_all>2)
            {
                sys_safe_access_enable();
                R16_INT32K_TUNE |= (0xFF<<5);
                sys_safe_access_disable();
                return;
            }
            else
            {
                sys_safe_access_enable();
                R16_INT32K_TUNE = (R16_INT32K_TUNE&0x1F)|(0x7F<<5);
                sys_safe_access_disable();
            }
        }
        else if((cnt_offset < 0)&&((R16_INT32K_TUNE>>5)<(-cnt_offset)))
        {
            if(retry_all>2)
            {
                sys_safe_access_enable();
                R16_INT32K_TUNE &= 0x1F;
                sys_safe_access_disable();
                return;
            }
            else
            {
                sys_safe_access_enable();
                R16_INT32K_TUNE = (R16_INT32K_TUNE&0x1F)|(0x7F<<5);
                sys_safe_access_disable();
            }
        }
        else
        {
            sys_safe_access_enable();
            R16_INT32K_TUNE += (cnt_offset<<5);
            sys_safe_access_disable();
            return;
        }
        retry_all++;

    }
}

/* ***************************************************************************
 * @fn RTCInitTime
 *
 * @brief RTC clock initialization current time
 *
 * @param y - Configuration year, MAX_Y = BEGYEAR + 44
 * @param mon - Configure month, MAX_MON = 12
 * @param d - Configuration day, MAX_D = 31
 * @param h - Configuration hours, MAX_H = 23
 * @param m - Configuration minutes, MAX_M = 59
 * @param s - Configure seconds, MAX_S = 59
 *
 * @return none */
void RTC_InitTime(uint16_t y, uint16_t mon, uint16_t d, uint16_t h, uint16_t m, uint16_t s)
{
    uint32_t         t;
    uint16_t         year, month, day, sec2, t32k;
    volatile uint8_t clk_pin;

    year = y;
    month = mon;
    day = 0;
    while(year > BEGYEAR)
    {
        day += YearLength(year - 1);
        year--;
    }
    while(month > 1)
    {
        day += monthLength(IsLeapYear(y), month - 2);
        month--;
    }

    day += d - 1;
    sec2 = (h % 24) * 1800 + m * 30 + s / 2;
    t32k = (s & 1) ? (0x8000) : (0);
    t = sec2;
    t = t << 16 | t32k;

    do
    {
        clk_pin = (R8_CK32K_CONFIG & RB_32K_CLK_PIN);
    } while(clk_pin != (R8_CK32K_CONFIG & RB_32K_CLK_PIN));
    if(!clk_pin)
    {
        while(!clk_pin)
        {
            do
            {
                clk_pin = (R8_CK32K_CONFIG & RB_32K_CLK_PIN);
            } while(clk_pin != (R8_CK32K_CONFIG & RB_32K_CLK_PIN));
        }
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = day;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_HI;
    sys_safe_access_disable();
    while((R32_RTC_TRIG & 0x3FFF) != (R32_RTC_CNT_DAY & 0x3FFF));
    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn RTC_GetTime
 *
 * @brief Get the current time
 *
 * @param py - year obtained, MAX_Y = BEGYEAR + 44
 * @param pmon - month obtained, MAX_MON = 12
 * @param pd - the day obtained, MAX_D = 31
 * @param ph - The hour obtained, MAX_H = 23
 * @param pm - minute obtained, MAX_M = 59
 * @param ps - the seconds obtained, MAX_S = 59
 *
 * @return none */
void RTC_GetTime(uint16_t *py, uint16_t *pmon, uint16_t *pd, uint16_t *ph, uint16_t *pm, uint16_t *ps)
{
    uint32_t t;
    uint16_t day, sec2, t32k;

    day = R32_RTC_CNT_DAY & 0x3FFF;
    sec2 = R16_RTC_CNT_2S;
    t32k = R16_RTC_CNT_32K;

    t = sec2 * 2 + ((t32k < 0x8000) ? 0 : 1);

    *py = BEGYEAR;
    while(day >= YearLength(*py))
    {
        day -= YearLength(*py);
        (*py)++;
    }

    *pmon = 0;
    while(day >= monthLength(IsLeapYear(*py), *pmon))
    {
        day -= monthLength(IsLeapYear(*py), *pmon);
        (*pmon)++;
    }
    (*pmon)++;
    *pd = day + 1;
    *ph = t / 3600;
    *pm = t % 3600 / 60;
    *ps = t % 60;
}

/* ***************************************************************************
 * @fn RTC_SetCycle32k
 *
 * @brief Configure the current number of RTC cycles based on the LSE/LSI clock
 *
 * @param cyc - Configure the initial value of the cycle count, MAX_CYC = 0xA8BFFFF = 2831155199
 *
 * @return none */
void RTC_SetCycle32k(uint32_t cyc)
{
    volatile uint8_t clk_pin;

    do
    {
        clk_pin = (R8_CK32K_CONFIG & RB_32K_CLK_PIN);
    } while((clk_pin != (R8_CK32K_CONFIG & RB_32K_CLK_PIN)) || (!clk_pin));

    sys_safe_access_enable();
    R32_RTC_TRIG = cyc;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn RTC_GetCycle32k
 *
 * @brief Get the current number of RTC cycles based on the LSE/LSI clock
 *
 * @param none
 *
 * @return Current cycle number, MAX_CYC = 0xA8BFFFF = 2831155199 */
__HIGH_CODE
uint32_t RTC_GetCycle32k(void)
{
    volatile uint32_t i;

    do
    {
        i = R32_RTC_CNT_32K;
    } while(i != R32_RTC_CNT_32K);

    return (i);
}
/* ***************************************************************************
 * @fn RTC_TMRFunCfg
 *
 * @brief RTC timing mode configuration (note that the timing reference is fixed to 32768Hz)
 *
 * @param t - refer to RTC_TMRCycTypeDef
 *
 * @return none */
void RTC_TMRFunCfg(RTC_TMRCycTypeDef t)
{
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL &= ~(RB_RTC_TMR_EN | RB_RTC_TMR_MODE);
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TMR_EN | (t);
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn RTC_TRIGFunCfg
 *
 * @brief RTC time trigger mode configuration
 *
 * @param cyc - Trigger interval time relative to the current time, based on the number of LSE/LSI clock cycles
 *
 * @return none */
void RTC_TRIGFunCfg(uint32_t cyc)
{
    uint32_t t;

    t = RTC_GetCycle32k() + cyc;
    if(t > RTC_MAX_COUNT)
    {
        t -= RTC_MAX_COUNT;
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn RTC_ModeFunDisable
 *
 * @brief RTC mode function is turned off
 *
 * @param m - Current mode that needs to be turned off
 *
 * @return none */
void RTC_ModeFunDisable(RTC_MODETypeDef m)
{
    uint8_t i = 0;

    if(m == RTC_TRIG_MODE)
    {
        i |= RB_RTC_TRIG_EN;
    }
    else if(m == RTC_TMR_MODE)
    {
        i |= RB_RTC_TMR_EN;
    }

    sys_safe_access_enable();
    R8_RTC_MODE_CTRL &= ~(i);
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn RTC_GetITFlag
 *
 * @brief Get RTC interrupt flag
 *
 * @param f - refer to RTC_EVENTTypeDef
 *
 * @return Interrupt flag status */
uint8_t RTC_GetITFlag(RTC_EVENTTypeDef f)
{
    if(f == RTC_TRIG_EVENT)
    {
        return (R8_RTC_FLAG_CTRL & RB_RTC_TRIG_FLAG);
    }
    else
    {
        return (R8_RTC_FLAG_CTRL & RB_RTC_TMR_FLAG);
    }
}

/* ***************************************************************************
 * @fn RTC_ClearITFlag
 *
 * @brief Clear RTC interrupt flag
 *
 * @param f - refer to RTC_EVENTTypeDef
 *
 * @return none */
void RTC_ClearITFlag(RTC_EVENTTypeDef f)
{
    switch(f)
    {
        case RTC_TRIG_EVENT:
            R8_RTC_FLAG_CTRL = RB_RTC_TRIG_CLR;
            break;
        case RTC_TMR_EVENT:
            R8_RTC_FLAG_CTRL = RB_RTC_TMR_CLR;
            break;
        default:
            break;
    }
}
