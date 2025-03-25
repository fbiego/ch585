/* ********************************* (C) COPYRIGHT ***************************
 * File Name: Main.c
 * Author: WCH
 * Version: V1.0
 * Date: 2021/03/09
 * Description: Adc sampling examples, including temperature detection, single channel detection, differential channel detection, TouchKey detection, interrupt method sampling, and scanning functions.
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#include "CH58x_common.h"

/* Note that when using the ADC scanning function, the default print pin of the routine is changed from PA14 to PB7. */
#define ADC_SCAN_MODE_EXAM      0
#define ADC_SCAN_CH_NUM            14

#define ADC_SCAN1_CH_IDX_1 (0)
#define ADC_SCAN1_CH_IDX_2 (4)
#define ADC_SCAN1_CH_IDX_3 (8)
#define ADC_SCAN1_CH_IDX_4 (12)
#define ADC_SCAN1_CH_IDX_5 (16)
#define ADC_SCAN1_CH_IDX_6 (20)
#define ADC_SCAN1_CH_IDX_7 (24)
#define ADC_SCAN1_CH_IDX_8 (28)
#define ADC_SCAN2_CH_IDX_9 (0)
#define ADC_SCAN2_CH_IDX_10 (4)
#define ADC_SCAN2_CH_IDX_11 (8)
#define ADC_SCAN2_CH_IDX_12 (12)
#define ADC_SCAN2_CH_IDX_13 (16)


const uint32_t ADC_PIN[14] = {
    0x00000010,                                           // PA4,AIN0
    0x00000020,                                           // PA5,AIN1
    0x00001000,                                           // PA12,AIN2
    0x00002000,                                           // PA13,AIN3
    0x00004000,                                           // PA14,AIN4
    0x00008000,                                           // PA15,AIN5
    0x00000008,                                           // PA3,AIN6
    0x00000004,                                           // PA2,AIN7
    0x00000002,                                           // PA1,AIN8
    0x00000001,                                           // PA0,AIN9
    0x00000040,                                           // PA6,AIN10
    0x00000080,                                           // PA7,AIN11
    0x00000100,                                           // PA8,AIN12
    0x00000200                                            // PA9,AIN13
};

const uint8_t adc_ch_seq[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

uint16_t adcBuff[40];

volatile uint8_t adclen;
volatile uint8_t DMA_end = 0;


/* When it is necessary to measure the absolute value of high accuracy in applications, it is recommended to use differential mode, with one end grounded */


/* ***************************************************************************
 * @fn DebugInit
 *
 * @brief debug initialization
 *
 * @return none */
void DebugInit(void)
{
#if(!ADC_SCAN_MODE_EXAM)
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#else
    GPIOB_SetBits(GPIO_Pin_7);
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif
}

/* ***************************************************************************
 * @fn main
 *
 * @brief main function
 *
 * @return none */
int main()
{

    uint8_t      i;
    signed short RoughCalib_Value = 0; // ADC coarse adjustment deviation value
    uint32_t temp = 0;
    uint8_t adcchidx = 0;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

    /* Configure serial debugging */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if(!ADC_SCAN_MODE_EXAM)

    /* Temperature sampling and output */
    PRINT("\n1.Temperature sampling...\n");
    ADC_InterTSSampInit();

    ADC_ExcutSingleConver();// When time is enough, it is recommended to convert and discard the first ADC data again.
    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver(); // Sampling 20 times in a row
    }
    for(i = 0; i < 20; i++)
    {
        uint32_t C25 = 0;
        C25 = (*((PUINT32)ROM_CFG_TMP_25C));
        PRINT("%d %d %d \n", adc_to_temperature_celsius(adcBuff[i]),adcBuff[i],C25);
    }

    /* Single channel sampling: select adc channel 0 for sampling, corresponding to PA4 pin, with data calibration function */
    PRINT("\n2.Single channel sampling...\n");
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_Floating);
    // The highest sampling rate is 8M
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);

    RoughCalib_Value = ADC_DataCalib_Rough(); // Used to calculate the internal deviation of the ADC and record it in the global variable RoughCalib_Value
    PRINT("RoughCalib_Value =%d \n", RoughCalib_Value);

    ADC_ChannelCfg(0);
    ADC_ExcutSingleConver();// When time is enough, it is recommended to convert and discard the first ADC data again.

    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver() + RoughCalib_Value; // Sampling 20 times in a row
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]); // Note: Due to the existence of internal deviation of ADC, data overflow may occur when the sampling voltage is near the selected gain range limit.
    }

    /* DMA single channel sampling: select adc channel 0 for sampling, corresponding to PA4 pin */
    PRINT("\n3.Single channel DMA sampling...\n");
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_Floating);
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    ADC_ChannelCfg(0);
    ADC_ExcutSingleConver();// When time is enough, it is recommended to convert and discard the first ADC data again.
    ADC_AutoConverCycle(192); // The sampling period is (256-192)*16 system clocks
    ADC_DMACfg(ENABLE, (uint32_t)&adcBuff[0], (uint32_t)&adcBuff[40], ADC_Mode_Single);
    PFIC_EnableIRQ(ADC_IRQn);
    ADC_StartAutoDMA();
    while(!DMA_end);
    DMA_end = 0;
    ADC_DMACfg(DISABLE, 0, 0, 0);

    for(i = 0; i < 40; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    /* Differential channel sampling: select adc channel 0 for sampling, corresponding to PA4 (AIN0), PA12 (AIN2) */
    PRINT("\n4.Diff channel sampling...\n");
    GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_12, GPIO_ModeIN_Floating);
    ADC_ExtDiffChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    ADC_ChannelCfg(0);
    ADC_ExcutSingleConver();// When time is enough, it is recommended to convert and discard the first ADC data again.
    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver(); // Sampling 20 times in a row
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    /* TouchKey sampling: select adc channel 2 for sampling, corresponding to PA12 */
    PRINT("\n5.TouchKey sampling...\n");
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_Floating);
    TouchKey_ChSampInit();
    ADC_ChannelCfg(2);

    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = TouchKey_ExcutSingleConver(0x10, 0); // Sampling 20 times in a row
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    /* Single channel sampling: interrupt mode, select adc channel 1 for sampling, corresponding to PA5 pin, without data calibration function */
    PRINT("\n6.Single channel sampling in interrupt mode...\n");
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_Floating);
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    ADC_ChannelCfg(1);
    ADC_ExcutSingleConver();// When time is enough, it is recommended to convert and discard the first ADC data again.
    adclen = 0;
    ADC_ClearITFlag();
    PFIC_EnableIRQ(ADC_IRQn);
    ADC_StartUp();
    while(adclen < 20);
    PFIC_DisableIRQ(ADC_IRQn);
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    while(1);
#else
    GPIOA_ModeCfg(ADC_PIN[adc_ch_seq[0]] |
                      ADC_PIN[adc_ch_seq[1]] |
                      ADC_PIN[adc_ch_seq[2]] |
                      ADC_PIN[adc_ch_seq[3]] |
                      ADC_PIN[adc_ch_seq[4]] |
                      ADC_PIN[adc_ch_seq[5]] |
                      ADC_PIN[adc_ch_seq[6]] |
                      ADC_PIN[adc_ch_seq[7]] |
                      ADC_PIN[adc_ch_seq[8]] |
                      ADC_PIN[adc_ch_seq[9]] |
                      ADC_PIN[adc_ch_seq[10]] |
                      ADC_PIN[adc_ch_seq[11]] |
                      ADC_PIN[adc_ch_seq[12]] |
                      ADC_PIN[adc_ch_seq[13]],
                  GPIO_ModeIN_Floating);

    ADC_ExtSingleChSampInit(SampleFreq_4_or_2, ADC_PGA_0);     // 4M sampling frequency, 0dB gain
    R8_ADC_CONVERT |= RB_ADC_SAMPLE_TIME;                      // 7 sampling cycles
    ADC_ChannelCfg(adc_ch_seq[0]);

    temp = (adc_ch_seq[1] << ADC_SCAN1_CH_IDX_1) |
           (adc_ch_seq[2] << ADC_SCAN1_CH_IDX_2) |
           (adc_ch_seq[3] << ADC_SCAN1_CH_IDX_3) |
           (adc_ch_seq[4] << ADC_SCAN1_CH_IDX_4) |
           (adc_ch_seq[5] << ADC_SCAN1_CH_IDX_5) |
           (adc_ch_seq[6] << ADC_SCAN1_CH_IDX_6) |
           (adc_ch_seq[7] << ADC_SCAN1_CH_IDX_7) |
           (adc_ch_seq[8] << ADC_SCAN1_CH_IDX_8);
    R32_ADC_SCAN_CFG1 = temp;
    //      PRINT("R32_ADC_SCAN_CH_CFG1:%08X;\r\n temp:%08X\n", R32_ADC_SCAN_CH_CFG1,temp);
    temp = 0;
    temp = (adc_ch_seq[9] << ADC_SCAN2_CH_IDX_9) |
           (adc_ch_seq[10] << ADC_SCAN2_CH_IDX_10) |
           (adc_ch_seq[11] << ADC_SCAN2_CH_IDX_11) |
           (adc_ch_seq[12] << ADC_SCAN2_CH_IDX_12) |
           (adc_ch_seq[13] << ADC_SCAN2_CH_IDX_13);
    temp |= RB_ADC_SCAN_SEL | RB_ADC_IE_SCAN_END | ((ADC_SCAN_CH_NUM - 1) << 24);  // RB_ADC_SCAN_NUM =  ADC_SCAN_CH_NUM - 1
    R32_ADC_SCAN_CFG2 = temp;

    R32_ADC_DMA_BEG = (uint32_t)&adcBuff[0] & 0x1FFFF;
    R32_ADC_DMA_END = (uint32_t)&adcBuff[40] & 0x1FFFF;
    R8_ADC_CTRL_DMA |= RB_ADC_DMA_ENABLE | RB_SCAN_AUTO_TYPE;          //| RB_ADC_IE_EOC| RB_ADC_IE_DMA_END |
    PFIC_EnableIRQ(ADC_IRQn);
    R8_ADC_CONVERT = RB_ADC_START;


    while (!DMA_end);
    DMA_end = 0;

    for (i = 0; i < ADC_SCAN_CH_NUM; i++)
    {
        PRINT("%d:%d ",i, adcBuff[i]);
        PRINT("\n");
        adcBuff[i] = 0;
    }
    PRINT("\n");

    while(1);
#endif
}

/* ***************************************************************************
 * @fn ADC_IRQHandler
 *
 * @brief ADC interrupt function
 *
 * @return none */
__INTERRUPT
__HIGH_CODE
void ADC_IRQHandler(void) // adc interrupt service program
{
#if(!ADC_SCAN_MODE_EXAM)
    if(ADC_GetDMAStatus())
    {
        ADC_StopAutoDMA();
        R32_ADC_DMA_BEG = ((uint32_t)adcBuff) & 0x1ffff;
        ADC_ClearDMAFlag();
        DMA_end = 1;
    }
    if(ADC_GetITStatus())
    {
        ADC_ClearITFlag();
        if(adclen < 20)
        {
            adcBuff[adclen] = ADC_ReadConverValue();
            ADC_StartUp(); // Use to clear the interrupt flag and start a new round of sampling
        }
        adclen++;
    }
#else
    if (R8_ADC_INT_FLAG & RB_ADC_IF_EOC)
    {
        uint8_t i = 0;
        i = R8_ADC_CONVERT;
        R8_ADC_CONVERT = i;
    }

    if (R32_ADC_SCAN_CFG2 & RB_ADC_IF_SCAN_END)
    {
        R32_ADC_SCAN_CFG2 |= RB_ADC_IF_SCAN_END;
        R32_ADC_DMA_BEG = (uint32_t)adcBuff & 0x1ffff;
        DMA_end = 1;
    }


#endif
}
