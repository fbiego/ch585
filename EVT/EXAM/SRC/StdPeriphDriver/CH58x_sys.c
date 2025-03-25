/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_SYS.c
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

volatile MachineMode_Call_func gs_machine_mode_func;

extern uint32_t _vector_base[];

/* ***************************************************************************
 * @fn SetSysClock
 *
 * @brief Configure the system running clock
 *
 * @param sc - System clock source selection refer to SYS_CLKTypeDef
 *
 * @return none */
__HIGH_CODE
void SetSysClock(SYS_CLKTypeDef sc)
{
    uint16_t clk_sys_cfg;
    uint8_t i;
    uint8_t x32M_c;
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    if(sc == RB_CLK_SYS_MOD)  // 32KHz
    {
        R16_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
    }

    else
    {
        if(sc & RB_OSC32M_SEL)
        {
            if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
            {
                x32M_c = R8_XT32M_TUNE;
                R8_XT32M_TUNE |= 0x03;
                R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
                clk_sys_cfg = R16_CLK_SYS_CFG;
                R16_CLK_SYS_CFG |= 0xC0;
                for(i=0; i<9; i++)
                {
                    __nop();
                }
                R16_CLK_SYS_CFG = clk_sys_cfg;
                R8_XT32M_TUNE = x32M_c;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
        }

        if(sc & RB_XROM_SCLK_SEL) // PLL div
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
            if((sc&0x1F) == 0 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK|(1<<4);
                R8_FLASH_CFG = 0X07;
            }
            else if((sc&0x1F) < 10)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X01;
            }
            else if((sc&0x1F) < 16)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X02;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK|(1<<4);
                R8_FLASH_CFG = 0X03;
            }
        }
        else
        {
            if((sc&0x1F) < 8 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X51;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X57;
            }
        }

        R16_CLK_SYS_CFG = sc|0xC0;
        R16_CLK_SYS_CFG = sc;

        if(sc & RB_OSC32M_SEL)
        {
            if((!((R8_GLOB_CFG_INFO & RB_CFG_DEBUG_EN)|(R8_GLOB_CFG_INFO & RB_CFG_ROM_READ ))) && (R8_SAFE_DEBUG_CTRL & RB_DEBUG_DIS))
            {
                R8_HFCK_PWR_CTRL &= ~RB_CLK_RC16M_PON;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL &= ~RB_CLK_XT32M_PON;
        }

    }
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn highcode_init
 *
 * @brief transfer highcode code
 *
 * @param none
 *
 * @return none */
__attribute__((section(".highcode_init")))
void highcode_init(void)
{
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    R32_MISC_CTRL |= 5; //
    R8_PLL_CONFIG &= ~(1 << 5); //
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON | RB_CLK_PLL_PON;
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_PLL_62_4MHz;
    R8_FLASH_CFG = 0X02;
    R8_XT32M_TUNE = (R8_XT32M_TUNE&(~0x03))|0x01;
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn MachineMode_Call_IRQ
 *
 * @brief Interrupts used by mechanical mode call functions
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
__INTERRUPT
void MachineMode_Call_IRQ(void)
{
    if(gs_machine_mode_func != NULL)
    {
        gs_machine_mode_func();
        gs_machine_mode_func = NULL;
    }
}

/* ***************************************************************************
 * @fn MachineMode_Call
 *
 * @brief Register the mechanical mode execution function and call it in mechanical mode
 *
 * @param func - Function for execution in mechanical mode
 *
 * @return none */
__HIGH_CODE
void MachineMode_Call(MachineMode_Call_func func)
{
    uint8_t i;
    uint32_t sw_vtf, sw_irqtable;
    uint32_t irqv;

    /* Close all interrupts here */
    irqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    /* If the user uses table-free interrupts for SW interrupts, all comments of this function need to be cancelled */
//    for(i = 0; i < 4; i++)
//    {
//        if(PFIC->VTFIDR[i] == SWI_IRQn)
//        {
// /* Found the SW interrupt used by the user and close it */
//            sw_vtf = PFIC->VTFADDR[i];
//            PFIC->VTFADDR[i] = (sw_vtf & 0xFFFFFFFE);
//            break;
//        }
//    }

    sw_irqtable = _vector_base[SWI_IRQn];
    _vector_base[SWI_IRQn] = (uint32_t)MachineMode_Call_IRQ;
    gs_machine_mode_func = func;

    /* Only open SWI_IRQn */
    PFIC_EnableIRQ(SWI_IRQn);
    /* Enter SWI_IRQn interrupt handling function */
    PFIC_SetPendingIRQ(SWI_IRQn);

    /* Wait for processing to end */
    while(gs_machine_mode_func != NULL);

    PFIC_DisableIRQ(SWI_IRQn);

//    if(i != 4)
//    {
// /* Restore the original SW table-free interrupt */
//        PFIC->VTFADDR[i] = sw_vtf;
//    }

    /* Restore the original interrupt enable configuration here */
    PFIC->IENR[0] = (irqv << 8);
    PFIC->IENR[1] = (irqv >> 24);
}

/* ***************************************************************************
 * @fn SetPI_func
 *
 * @brief Enable prefetch instruction function for mechanical mode calls
 *
 * @param none
 *
 * @return none */void SetPI_func()
{
    write_csr(0xbc0, 0x25);
}

/* ***************************************************************************
 * @fn SYS_EnablePI
 *
 * @brief enables the prefetch command function
 *
 * @param none
 *
 * @return null */
void SYS_EnablePI()
{
    MachineMode_Call(SetPI_func);
}

/* ***************************************************************************
 * @fn GetSysClock
 *
 * @brief Get the current system clock
 *
 * @param none
 *
 * @return Hz */
uint32_t GetSysClock(void)
{
    if((R16_CLK_SYS_CFG & RB_CLK_SYS_MOD) == RB_CLK_SYS_MOD)
    { // 32K is the main frequency
        return (CAB_LSIFQ);
    }
    else if(R16_CLK_SYS_CFG & RB_XROM_SCLK_SEL)
    {
        if(!(R16_CLK_SYS_CFG & 0x1f))
        {
            return ((R16_CLK_SYS_CFG & RB_OSC32M_SEL)?32000000:16000000);
        }
        else
        {   // PLL for frequency division
            return (312000000 / (R16_CLK_SYS_CFG & 0x1f));
        }
    }
    else if(R16_CLK_SYS_CFG & RB_OSC32M_SEL)
    { // 32M for frequency division
        return (32000000 / (R16_CLK_SYS_CFG & 0x1f));
    }
    else
    { // 16M for frequency division
        return (16000000 / (R16_CLK_SYS_CFG & 0x1f));
    }
}

/* ***************************************************************************
 * @fn SYS_GetInfoSta
 *
 * @brief Get the current system information status
 *
 * @param i - refer to SYS_InfoStaTypeDef
 *
 * @return is enabled */
uint8_t SYS_GetInfoSta(SYS_InfoStaTypeDef i)
{
    if(i == STA_SAFEACC_ACT)
    {
        return (R8_SAFE_ACCESS_SIG & RB_SAFE_ACC_ACT);
    }
    else
    {
        return (R8_GLOB_CFG_INFO & (1 << i));
    }
}

/* ***************************************************************************
 * @fn SYS_ResetExecute
 *
 * @brief Execute system software reset
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
void SYS_ResetExecute(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn SYS_DisableAllIrq
 *
 * @brief Close all interrupts and keep the current interrupt value
 *
 * @param pirqv - Currently reserved interrupt value
 *
 * @return none */
void SYS_DisableAllIrq(uint32_t *pirqv)
{
    *pirqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;
}

/* ***************************************************************************
 * @fn SYS_RecoverIrq
 *
 * @brief restores the interrupt value that was closed before
 *
 * @param irq_status - currently retained interrupt value
 *
 * @return none */
void SYS_RecoverIrq(uint32_t irq_status)
{
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}

/* ***************************************************************************
 * @fn SYS_GetSysTickCnt
 *
 * @brief Get the current system (SYSTICK) count value
 *
 * @param none
 *
 * @return Current count value */
uint32_t SYS_GetSysTickCnt(void)
{
    return SysTick->CNTL;
}

/* ***************************************************************************
 * @fn WWDG_ITCfg
 *
 * @brief Watchdog timer overflow interrupt enable
 *
 * @param s - whether the overflow is interrupted
 *
 * @return none */
void WWDG_ITCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_INT_EN;
    }
    else
    {
        ctrl |= RB_WDOG_INT_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn WWDG_ResetCfg
 *
 * @brief Watchdog timer reset function
 *
 * @param s - Whether to reset overflow
 *
 * @return none */
void WWDG_ResetCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_RST_EN;
    }
    else
    {
        ctrl |= RB_WDOG_RST_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn WWDG_ClearFlag
 *
 * @brief Clear the watchdog interrupt flag, and reload the count value can also be cleared
 *
 * @param none
 *
 * @return none */
void WWDG_ClearFlag(void)
{
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_WDOG_INT_FLAG;
    sys_safe_access_disable();
}

/* ***************************************************************************
 * @fn HardFault_Handler
 *
 * @brief The hardware error interrupts, and the reset is power-on reset after entering.
 *
 * @param none
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void HardFault_Handler(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R16_INT32K_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
    while(1);
}

/* ***************************************************************************
 * @fn mDelayuS
 *
 * @brief uS Delay
 *
 * @param t - Time parameters
 *
 * @return none */
__HIGH_CODE
void mDelayuS(uint16_t t)
{
    uint32_t i;
#if(FREQ_SYS == 78000000)
    i = t * 13;
#elif(FREQ_SYS == 62400000)
    i = ((uint32_t)t * 78) / 5;
#elif(FREQ_SYS == 52000000)
    i = t * 13;
#elif(FREQ_SYS == 39000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 26000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 24000000)
    i = t * 6;
#elif(FREQ_SYS == 19500000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 32000000)
    i = t << 3;
#elif(FREQ_SYS == 16000000)
    i = t << 2;
#elif(FREQ_SYS == 13000000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 8000000)
    i = t << 1;
#elif(FREQ_SYS == 4000000)
    i = t;
#elif(FREQ_SYS == 2000000)
    i = t >> 1;
#elif(FREQ_SYS == 1000000)
    i = t >> 2;
#else
    i = t;
#endif

#if((FREQ_SYS == 78000000)||\
    (FREQ_SYS == 39000000)||\
    (FREQ_SYS == 19500000))
    while(--i)
    {
        __nop();
    };
#else
    do
    {
        __nop();
    }while(--i);
#endif
}

/* ***************************************************************************
 * @fn mDelaymS
 *
 * @brief mS Delay
 *
 * @param t - Time parameters
 *
 * @return none */
__HIGH_CODE
void mDelaymS(uint16_t t)
{
    do
    {
        mDelayuS(1000);
    }while(--t);
}

#ifdef DEBUG
int _write(int fd, char *buf, int size)
{
    int i;
    for(i = 0; i < size; i++)
    {
#if DEBUG == Debug_UART0
        while(R8_UART0_TFC == UART_FIFO_SIZE);                  /* Wait for data to be sent */
        R8_UART0_THR = *buf++; /* Send data */
#elif DEBUG == Debug_UART1
        while(R8_UART1_TFC == UART_FIFO_SIZE);                  /* Wait for data to be sent */
        R8_UART1_THR = *buf++; /* Send data */
#elif DEBUG == Debug_UART2
        while(R8_UART2_TFC == UART_FIFO_SIZE);                  /* Wait for data to be sent */
        R8_UART2_THR = *buf++; /* Send data */
#elif DEBUG == Debug_UART3       
        while(R8_UART3_TFC == UART_FIFO_SIZE);                  /* Wait for data to be sent */
        R8_UART3_THR = *buf++; /* Send data */
#endif
    }
    return size;
}

#endif

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || ((uint32_t)curbrk + incr > (__get_SP() - 64)))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

/*********************************************************************
 * @fn      __wrap_memcpy
 *
 * @brief   wrap memcpy
 *
 * @return  dst
 */
__HIGH_CODE
void *__wrap_memcpy(void *dst, void *src, size_t size)
{
    __MCPY(dst, src, (void *)((uint32_t)src+size));
    return dst;
}
