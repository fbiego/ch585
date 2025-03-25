/* ********************************* (C) COPYRIGHT ***************************
 * File Name: wch_nfca_pcd_bsp.c
 * Author: WCH
 * Version: V1.1
 * Date: 2024/11/14
 * Description: NFC-A PCD BSP underlying interface
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#include "wch_nfca_pcd_bsp.h"

/* Each file has a separate debug print switch, setting 0 can prohibit internal printing of this file. */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
#define PRINTF(...) PRINT(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

__attribute__((aligned(4))) static uint16_t gs_nfca_pcd_data_buf[NFCA_PCD_DATA_BUF_SIZE];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];

static uint16_t gs_lpcd_adc_base_value;
static uint16_t gs_lpcd_adc_filter_buf[8];
uint16_t g_nfca_pcd_recv_buf_len;
uint32_t g_nfca_pcd_recv_bits;

#if NFCA_PCD_USE_NFC_CTR_PIN

void nfca_pcd_ctr_init(void)
{
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    GPIOA_ResetBits(GPIO_Pin_7);
    nfca_pcd_set_lp_ctrl(NFCA_PCD_LP_CTRL_0_5_VDD);
}

__always_inline static inline void nfca_pcd_ctr_on(void)
{
    /* Output enable, output is low, output cannot be high, antenna peak-to-peak voltage divided by one third */
    R32_PA_DIR |= (GPIO_Pin_7);
}

__always_inline static inline void nfca_pcd_ctr_off(void)
{
    /* Output is disabled, analog input, antenna peak-to-peak value is almost indistinguishable */
    R32_PA_DIR &= ~(GPIO_Pin_7);
}

void nfca_pcd_ctr_handle(void)
{
    if(nfca_pcd_get_lp_status())
    {
        /* Peak-to-peak value is too low */
        PRINTF("LP\n");
        nfca_pcd_ctr_off();
    }
    else
    {
        nfca_pcd_ctr_on();
    }
}

#endif

/* ***************************************************************************
 * @fn nfca_pcd_init
 *
 * @brief nfc-a pcd card reader initialization
 *
 * @param none
 *
 * @return none */
void nfca_pcd_init(void)
{
    nfca_pcd_config_t cfg;

    /* NFC pin is initialized to analog input mode */
    GPIOB_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_16 | GPIO_Pin_17, GPIO_ModeIN_Floating);

    /* Turn off GPIO digital input function */
    R32_PIN_IN_DIS |= ((GPIO_Pin_8 | GPIO_Pin_9) << 16);        /* Turn off the digital input functions of GPIO_Pin_8 and GPIO_Pin_9 in GPIOB */
    R16_PIN_CONFIG |= ((GPIO_Pin_16 | GPIO_Pin_17) >> 8);       /* Turn off the digital input functions of GPIO_Pin_16 and GPIO_Pin_17 in GPIOB */

    /* CH584F and CH585F are shorted internally, and PA9 needs to be set to analog input and turn off the digital function. M package comments the following two sentences of code, F package cancels the annotation */
//    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeIN_Floating);
//    R32_PIN_IN_DIS |= GPIO_Pin_9;

    cfg.data_buf = gs_nfca_pcd_data_buf;
    cfg.data_buf_size = NFCA_PCD_DATA_BUF_SIZE;

    cfg.send_buf = g_nfca_pcd_send_buf;
    cfg.send_buf_size = NFCA_PCD_MAX_SEND_NUM;

    cfg.recv_buf = g_nfca_pcd_recv_buf;
    cfg.recv_buf_size = NFCA_PCD_MAX_RECV_NUM;

    cfg.parity_buf = g_nfca_pcd_parity_buf;
    cfg.parity_buf_size = NFCA_PCD_MAX_PARITY_NUM;

    /* Pass the data area pointer to the BUFFER pointer in the NFC library */
    nfca_pcd_lib_init(&cfg);

#if NFCA_PCD_USE_NFC_CTR_PIN
    nfca_pcd_ctr_init();
#endif

}

/* ***************************************************************************
 * @fn nfca_pcd_start
 *
 * @brief nfc-a pcd card reader function starts running
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
void nfca_pcd_start(void)
{
#if NFCA_PCD_USE_NFC_CTR_PIN
    nfca_pcd_ctr_on();
#endif
    nfca_pcd_lib_start();
    PFIC_ClearPendingIRQ(NFC_IRQn);
    PFIC_EnableIRQ(NFC_IRQn);
}

/* ***************************************************************************
 * @fn nfca_pcd_stop
 *
 * @brief nfc-a pcd reader function stops running
 *
 * @param none
 *
 * @return none */
__HIGH_CODE
void nfca_pcd_stop(void)
{
    nfca_pcd_lib_stop();
    PFIC_DisableIRQ(NFC_IRQn);
}

/* ***************************************************************************
 * @fn nfca_pcd_wait_communicate_end
 *
 * @brief nfc-a pcd card reader waiting for communication to end
 *
 * @param none
 *
 * @return nfca_pcd_controller_state_t Communication status */
nfca_pcd_controller_state_t nfca_pcd_wait_communicate_end(void)
{
    nfca_pcd_controller_state_t status;
    uint32_t overtimes;

    overtimes = 0;

    /* This function can be rewrite in a Bluetooth task, query the end status with the task, the example is dead, etc. */
    while (1)
    {
        status = nfca_pcd_get_communicate_status();
        if (status != 0)
        {
            break;
        }

        if (overtimes > (NFCA_PCD_WAIT_MAX_MS * 10))
        {
            /* Software timeout time, */
            break;
        }

        mDelayuS(100);
        overtimes++;
    }

    g_nfca_pcd_recv_buf_len = nfca_pcd_get_recv_data_len();
    g_nfca_pcd_recv_bits = nfca_pcd_get_recv_bits();

    return status;
}

/* ***************************************************************************
 * @fn nfca_pcd_rand
 *
 * @brief nfc-a pcd reader random number generation function
 *
 * @param none
 *
 * @return nfca_pcd_controller_state_t Communication status */
uint32_t nfca_pcd_rand(void)
{
    /* Need to implement a callback that generates random numbers by yourself */
    /* When used with Bluetooth, you can use tmos_rand() to return */
    return 0;
}

/* ***************************************************************************
 * @fn nfca_adc_get_ant_signal
 *
 * @brief nfca Detect antenna signal strength
 *
 * @param none
 *
 * @return Detected ADC value */
__HIGH_CODE
uint16_t nfca_adc_get_ant_signal(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;
    uint32_t adc_data_all;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB sampling ADC_PGA_1_4 */
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT &= ~(3 << 4);  /* 4ä¸ªTadc */

    adc_data_all = 0;

    for(uint8_t i = 0; i < 2; i++)
    {
        R8_ADC_CONVERT |= RB_ADC_START;
        while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
        adc_data_all = adc_data_all + R16_ADC_DATA;
    }

    adc_data = adc_data_all / 2;

    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;
    return (adc_data);
}

/* ***************************************************************************
 * @fn nfca_pcd_lpcd_calibration
 *
 * @brief nfca pcd low power card calibration
 *
 * @param none
 *
 * @return none */
void nfca_pcd_lpcd_calibration(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint32_t adc_all;
    uint16_t adc_max, adc_min, adc_value;
    uint8_t i;

    /* Median filtering */
    adc_all = 0;
    adc_max = 0;
    adc_min = 0xffff;

    nfca_pcd_start();
    mDelayuS(200);      /* Internal signal establishment takes 200us to stabilize */

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    /* Adc configuration save */
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB sampling ADC_PGA_1_4 */
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT &= ~(3 << 4);  /* 4ä¸ªTadc */

    for(i = 0; i < 10; i++)
    {
        R8_ADC_CONVERT |= RB_ADC_START;
        while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
        adc_value = R16_ADC_DATA;

        if(adc_value > adc_max)
        {
            adc_max = adc_value;
        }
        if(adc_value < adc_min)
        {
            adc_min = adc_value;
        }
        adc_all = adc_all + adc_value;
    }

    /* Adc configuration recovery */
    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;

    adc_all = adc_all - adc_max - adc_min;

    gs_lpcd_adc_base_value = adc_all >> 3;

    PRINTF("gs_lpcd_adc_base_value:%d\n", gs_lpcd_adc_base_value);

    for(i = 0; i < 8; i++)
    {
        gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_base_value;
    }

    nfca_pcd_stop();
}

/* ***************************************************************************
 * @fn nfca_pcd_lpcd_adc_filter_buf_add
 *
 * @brief nfca pcd low power detection card adc value filter processing
 *
 * @param lpcd_adc - Adc value that needs to be added
 *
 * @return uint16_t - ADC threshold reference for new low power check cards */
__HIGH_CODE
static uint16_t nfca_pcd_lpcd_adc_filter_buf_add(uint16_t lpcd_adc)
{
    uint32_t lpcd_adc_all = 0;
    uint8_t i;
    for(uint8_t i = 0; i < 7; i++)
    {
        gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_filter_buf[i + 1];
        lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[i];
    }
    gs_lpcd_adc_filter_buf[7] = lpcd_adc;
    lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[7];
    lpcd_adc_all = (lpcd_adc_all >> 3);
    return (uint16_t)lpcd_adc_all;
}

/* *********************************************************************************************
 * @fn nfca_pcd_lpcd_check
 *
 * @brief nfc-a pcd lpcd ADC check card
 *
 * @param None.
 *
 * @return 1 has a card, 0 has no card. */
__HIGH_CODE
uint8_t nfca_pcd_lpcd_check(void)
{
    uint32_t adc_value_diff;
    uint16_t adc_value;
    uint8_t res = 0;

    adc_value = nfca_adc_get_ant_signal();
    PRINTF("adc_value:%d\n", adc_value);
    if(adc_value > gs_lpcd_adc_base_value)
    {
        adc_value_diff = adc_value - gs_lpcd_adc_base_value;

    }
    else
    {
        adc_value_diff = gs_lpcd_adc_base_value - adc_value;
    }
    adc_value_diff = (adc_value_diff * 1000) / gs_lpcd_adc_base_value;

    if(adc_value > gs_lpcd_adc_base_value)
    {

#if 0
        /* * Here you can consider that it will trigger if it does not judge that it will increase.
         * When the card reader and card device are close to each other, the value will increase.
         * When the phone detects that the antenna has a waveform, switching to card mode will reduce the antenna signal. */
        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_PERMIL)
        {
            res = 1;
        }
#endif

        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL)
        {
            /* The threshold value should be updated slowly. The value should be modified as appropriate according to the detection interval. The error is basically within two thousandths. */
            adc_value = ((uint32_t)gs_lpcd_adc_base_value * (1000 + NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL) / 1000);
        }
    }
    else
    {
        if(adc_value_diff >= NFCA_PCD_LPCD_THRESHOLD_PERMIL)
        {
            res = 1;
        }
#if 1
        /* Each limit is 1 */
        if(adc_value < gs_lpcd_adc_base_value)
        {
            adc_value = gs_lpcd_adc_base_value - 1;
        }
#else
        /* Limit the range by a thousandth */
        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL)
        {
            /* The threshold value should be updated slowly. The value should be modified as appropriate according to the detection interval. The error is basically within two thousandths. */
            adc_value = ((uint32_t)gs_lpcd_adc_base_value * (1000 - NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL) / 1000);
        }
#endif
    }

    gs_lpcd_adc_base_value = nfca_pcd_lpcd_adc_filter_buf_add(adc_value);
    return res;
}

/* *********************************************************************************************
 * @fn NFC_IRQHandler
 *
 * @brief nfc-a interrupt function, in PCD mode, one interrupt can be entered within 800us
 *
 * @param None.
 *
 * @return 1 has a card, 0 has no card. */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void NFC_IRQHandler(void)
{
    NFC_IRQLibHandler();
}
