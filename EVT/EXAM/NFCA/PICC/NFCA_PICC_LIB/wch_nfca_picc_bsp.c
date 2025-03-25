/* ********************************* (C) COPYRIGHT ***************************
 * File Name: wch_nfca_picc_bsp.c
 * Author: WCH
 * Version: V1.0
 * Date: 2024/10/29
 * Description: nfc picc sending and receiving control layer
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */
#include "wch_nfca_picc.h"
#include "wch_nfca_picc_bsp.h"

/* Each file has a separate debug print switch, setting 0 can prohibit internal printing of this file. */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

/* * NFC PICC function will occupy TIMER0 and TIMER3 for data capture and sending.
 * So when using the PICC function, TIMER0 and TIMER3 cannot be used by other functions. */

static uint32_t gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_data_buf[PICC_DATA_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_parity_buf[PICC_DATA_BUF_LEN];

void nfca_picc_init(void)
{
    nfca_picc_config_t cfg;

    /* For EVT board NFC CTR pin, if the time-sharing card and card reader are used, the NFC CTR pin and PA7 are shorted, the pin needs to be set as an analog input before starting PICC. */
    /* If you only use card mode, the NFC CTR pin does not need to be shorted to PA7, then there is no need to initialize the pin, comment the code below. */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_7);

    /* PA9 and PB9 of CH585F and CH584F are shorted inside the chip, so the F series needs to uncomment the following PA9 code. */
//    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeIN_Floating);
//    GPIOADigitalCfg(DISABLE, GPIO_Pin_9);

    GPIOB_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_16 | GPIO_Pin_17, GPIO_ModeIN_Floating);

    /* Turn off GPIO digital input function */
    R32_PIN_IN_DIS |= ((GPIO_Pin_8 | GPIO_Pin_9) << 16);        /* Turn off the digital input functions of GPIO_Pin_8 and GPIO_Pin_9 in GPIOB */
    R16_PIN_CONFIG |= ((GPIO_Pin_16 | GPIO_Pin_17) >> 8);       /* Turn off the digital input functions of GPIO_Pin_16 and GPIO_Pin_17 in GPIOB */

    cfg.signal_buf = gs_picc_signal_buf;
    cfg.signal_buf_len = PICC_SIGNAL_BUF_LEN;

    cfg.parity_buf = g_picc_parity_buf;
    cfg.data_buf = g_picc_data_buf;
    cfg.data_buf_len = PICC_DATA_BUF_LEN;

    nfca_picc_lib_init(&cfg);
}

void nfca_picc_start(void)
{
    nfca_picc_lib_start();

    /* TIMER3 Send PWM data Send using DMA */
    R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

#if NFCA_PICC_RSP_POLAR != 0
    R8_TMR3_CTRL_MOD = (High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#else
    R8_TMR3_CTRL_MOD = (Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#endif

    R32_TMR3_CNT_END = TMR3_NFCA_PICC_CNT_END;
    R32_TMR3_FIFO = 0;
    R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
    R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END;
    R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

#if NFCA_PICC_RSP_POLAR != 0
    R8_TMR3_CTRL_MOD = ((High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#else
    R8_TMR3_CTRL_MOD = ((Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#endif

    /* TIMER0 Captures received data */
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56;
    R32_TMR0_CNT_END = TMR0_NFCA_PICC_CNT_END;

    /* Clear the interrupt flag bit */
    R32_TMR0_DMA_END = R32_TMR0_DMA_NOW + 0x100;
    R8_TMR0_INT_FLAG = RB_TMR_IF_DMA_END;

    /* Configure the DMA buffer address */
    R32_TMR0_DMA_BEG = (uint32_t)gs_picc_signal_buf;
    R32_TMR0_DMA_END = (uint32_t)&(gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN]);
    /* DMA uses loop mode */
    R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;

    /* Enable interrupt, only DATA_ACT interrupt is enabled first. When an interrupt is entered, valid data must be received. */
    R8_TMR0_INT_FLAG = RB_TMR_IF_DATA_ACT;
    R8_TMR0_INTER_EN = RB_TMR_IE_DATA_ACT;

    /* Start counting */
    R8_TMR0_CTRL_MOD = (RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);

    PFIC_EnableIRQ(TMR3_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
}

void nfca_picc_stop(void)
{
    PFIC_DisableIRQ(TMR0_IRQn);
    PFIC_DisableIRQ(TMR3_IRQn);

    R8_TMR0_CTRL_DMA = 0;
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR3_CTRL_DMA = 0;
    R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

    nfca_picc_lib_stop();
}

/* ***************************************************************************
 * @fn nfca_picc_rand
 *
 * @brief nfc-a picc card reader random number generation function
 *
 * @param none
 *
 * @return Random number */
__attribute__((section(".highcode")))
uint32_t nfca_picc_rand(void)
{
    /* Need to implement a callback that generates random numbers by yourself */
    /* When used with Bluetooth, you can use tmos_rand() to return */
    return 0;
}

/* The TIMER0 interrupt function must be of the highest priority and cannot be interrupted. The interrupts closed during operation cannot exceed 100us. */
/* Since the NFC protocol stipulates that the ATQA reply signal at the beginning of the communication must be around 90us after the card reader sends, so if the shutdown interrupt exceeds 90us,
 * It may cause the device to fail to communicate successfully. Some card reader devices have low time requirements, so there is no problem. Generally, mobile phones have high card reading requirements. */
/* Due to frequent signals and complex function processing flow, the interrupt function will remain in the interrupt during the reception of data.
 * After processing data and starting to send data, exit the interrupt and stop receiving. Receive will be re-opened until TMR3 is interrupted. */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR0_IRQHandler(void)
{
    nfca_picc_rx_irq_handler();
}

/* The TIMER3 interrupt function must be of the highest priority and will not be interrupted. The interrupts closed during operation cannot exceed 100us. */
/* Since the NFC protocol stipulates that after the slave reply is completed, the host may have a next reply at the minimum time of about 80us. Therefore, if the shutdown interruption exceeds 80us and the card reader sends a signal very quickly, it will not be able to successfully receive it. */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR3_IRQHandler(void)
{
    nfca_picc_tx_irq_handler();
}
