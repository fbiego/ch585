/* ********************************* (C) COPYRIGHT ***************************
 * File Name: RTC.c
 * Author: WCH
 * Version: V1.2
 * Date: 2022/01/18
 * Description: RTC configuration and its initialization
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/******************************************************************************/
/* The header file contains */
#include "HAL.h"

/*********************************************************************
 * CONSTANTS
 */
#define RTC_INIT_TIME_HOUR      0
#define RTC_INIT_TIME_MINUTE    0
#define RTC_INIT_TIME_SECEND    0

/***************************************************
 * Global variables
 */
volatile uint32_t RTCTigFlag;

#if RF_8K

/* ***************************************************************************
 * @fn TMR3_IRQHandler
 *
 * @brief TMR0 interrupt function
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
void TMR3_IRQHandler(void) // TMR3
{
    uint32_t trig_time;

    TMR3_ClearITFlag(TMR0_3_IT_CYC_END); // Clear the interrupt flag
    if( !TMOS_TimerIRQHandler( &trig_time )  )
    {
        if( trig_time )
        {
            R32_TMR3_CNT_END = trig_time;
            R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;
            R8_TMR3_CTRL_MOD = RB_TMR_COUNT_EN;
        }
        else
        {
            PRINT("!!!!!!!!!!!!!!!!!! warn \n");
        }
    }
}

/*
 * @brief
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
__HIGH_CODE
static uint32_t SYS_GetClock1Value(void)
{
    return SysTick->CNT;
}
__HIGH_CODE
static void SYS_SetClock1PendingIRQ(void)
{
    PFIC_SetPendingIRQ( TMR3_IRQn );
}
__HIGH_CODE
static void SYS_SetTignOffest( int32_t val )
{
    R32_TMR3_CNT_END += (val);
}

#endif

/* *********************************************************************************************
 * @fn RTC_SetTignTime
 *
 * @brief Configure RTC trigger time
 *
 * @param time - Trigger time.
 *
 * @return None. */
void RTC_SetTignTime(uint32_t time)
{
    sys_safe_access_enable();
    R32_RTC_TRIG = time;
    sys_safe_access_disable();
    RTCTigFlag = 0;
}

/* *********************************************************************************************
 * @fn RTC_IRQHandler
 *
 * @brief RTC interrupt handling
 *
 * @param None.
 *
 * @return None. */
__INTERRUPT
__HIGH_CODE
void RTC_IRQHandler(void)
{
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    RTCTigFlag = 1;
}

/* *********************************************************************************************
 * @fn SYS_GetClockValue
 *
 * @brief Get the current count value of RTC
 *
 * @param None.
 *
 * @return None. */
__HIGH_CODE
static uint32_t SYS_GetClockValue(void)
{
    volatile uint32_t i;

    do
    {
        i = R32_RTC_CNT_32K;
    } while(i != R32_RTC_CNT_32K);

    return (i);
}
__HIGH_CODE
static void SYS_SetPendingIRQ(void)
{
    PFIC_SetPendingIRQ( RTC_IRQn );
}

/* *********************************************************************************************
 * @fn HAL_Time0Init
 *
 * @brief System timer initialization
 *
 * @param None.
 *
 * @return None. */
void HAL_TimeInit(void)
{
    bleClockConfig_t conf;
#if(CLK_OSC32K)
    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON);
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    sys_safe_access_disable();
    LSECFG_Current(LSE_RCur_100);
    Lib_Calibration_LSI();
#else
    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~RB_CLK_INT32K_PON;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON;
    sys_safe_access_disable();
#endif
    RTC_InitTime(2020, 1, 1, 0, 0, 0); // RTC clock initialization current time

    tmos_memset( &conf, 0, sizeof(bleClockConfig_t) );
    conf.ClockAccuracy = CLK_OSC32K ? 1000 : 50;
    conf.ClockFrequency = CAB_LSIFQ;
    conf.ClockMaxCount = RTC_MAX_COUNT;
    conf.getClockValue = SYS_GetClockValue;
    conf.SetPendingIRQ = SYS_SetPendingIRQ;

#if RF_8K
    // rf-8k communication time related configuration
    conf.Clock1Frequency = GetSysClock( )/1000;
    conf.getClock1Value = SYS_GetClock1Value;
    conf.SetClock1PendingIRQ = SYS_SetClock1PendingIRQ;
    conf.SetTign = SYS_SetTignOffest;
    TMR3_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // Turn on interrupt
    PFIC_EnableIRQ(TMR3_IRQn);
#endif

    TMOS_TimerInit( &conf );

}

/******************************** endfile @ time ******************************/
