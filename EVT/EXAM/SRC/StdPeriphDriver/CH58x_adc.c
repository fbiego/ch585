/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_adc.c
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
 * @fn ADC_DataCalib_Rough
 *
 * @brief sample data to roughly tune, get the deviation value, you must first configure the ADC and then call this function to get the calibration value
 *
 * @param none
 *
 * @return deviation */
signed short ADC_DataCalib_Rough(void) // Sampling data roughly tuned to obtain deviation value
{
    uint16_t i;
    uint32_t sum = 0;
    uint8_t  ch = 0;   // Backup channel
    uint8_t  cfg = 0;   // Backup

    ch = R8_ADC_CHANNEL;
    cfg = R8_ADC_CFG;

    R8_ADC_CFG |= RB_ADC_OFS_TEST; // Enter test mode
    R8_ADC_CFG &= ~RB_ADC_DIFF_EN; // Close the difference

    R8_ADC_CONVERT |= RB_ADC_START;
    while(R8_ADC_CONVERT & RB_ADC_START);
    for(i = 0; i < 16; i++)
    {
        R8_ADC_CONVERT |= RB_ADC_START;
        while(R8_ADC_CONVERT & RB_ADC_START);
        sum += (~R16_ADC_DATA) & RB_ADC_DATA;
    }
    sum = (sum + 8) >> 4;

    R8_ADC_CFG = cfg;  // Restore configuration values
    R8_ADC_CHANNEL = ch;

    return (2048 - sum);
}

/* ***************************************************************************
 * @fn ADC_ExtSingleChSampInit
 *
 * @brief External signal single channel sampling initialization
 *
 * @param sp - refer to ADC_SampClkTypeDef
 * @param ga - refer to ADC_SignalPGATypeDef
 *
 * @return none */
void ADC_ExtSingleChSampInit(ADC_SampClkTypeDef sp, ADC_SignalPGATypeDef ga)
{
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (sp << 6) | ((ga&0xF) << 4);
    if( ga & ADC_PGA_2_ )
    {
        R8_ADC_CONVERT |= RB_ADC_PGA_GAIN2;
    }
    else
    {
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    }
}

/* ***************************************************************************
 * @fn ADC_ExtDiffChSampInit
 *
 * @brief External signal differential channel sampling initialization
 *
 * @param sp - refer to ADC_SampClkTypeDef
 * @param ga - refer to ADC_SignalPGATypeDef
 *
 * @return none */
void ADC_ExtDiffChSampInit(ADC_SampClkTypeDef sp, ADC_SignalPGATypeDef ga)
{
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_DIFF_EN | (sp << 6) | ((ga&0xF) << 4);
    if( ga & ADC_PGA_2_ )
    {
        R8_ADC_CONVERT |= RB_ADC_PGA_GAIN2;
    }
    else
    {
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    }
}

/* ***************************************************************************
 * @fn ADC_InterTSSampInit
 *
 * @brief Built-in temperature sensor sampling initialization
 *
 * @param none
 *
 * @return none */
void ADC_InterTSSampInit(void)
{
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_TEM_SENSOR = RB_TEM_SEN_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_VTEMP;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_DIFF_EN | (3 << 4);
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
}

/* ***************************************************************************
 * @fn ADC_InterBATSampInit
 *
 * @brief Built-in battery voltage sampling initialization
 *
 * @param none
 *
 * @return none */
void ADC_InterBATSampInit(void)
{
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_VBAT;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0 << 4); // Use -12dB mode
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
}

/* ***************************************************************************
 * @fn TouchKey_ChSampInit
 *
 * @brief Touch key channel sampling initialization
 *
 * @param none
 *
 * @return none */
void TouchKey_ChSampInit(void)
{
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (ADC_PGA_0 << 4) | (SampleFreq_8_or_4 << 6);
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_TKEY_CFG = RB_TKEY_PWR_ON | RB_TKEY_CURRENT;
}

/* ***************************************************************************
 * @fn ADC_ExcutSingleConver
 *
 * @brief ADC performs a single conversion
 *
 * @param none
 *
 * @return ADC converted data */
uint16_t ADC_ExcutSingleConver(void)
{
    R8_ADC_CONVERT |= RB_ADC_START;
    while(R8_ADC_CONVERT & RB_ADC_START);

    return (R16_ADC_DATA & RB_ADC_DATA);
}

/* ***************************************************************************
 * @fn TouchKey_ExcutSingleConver
 *
 * @brief TouchKey converted data
 *
 * @param charg - Touchkey charging time, 5bits valid, t=charg*Tadc
 * @param disch - Touchkey discharge time, 3bits valid, t=disch*Tadc
 *
 * @return Current TouchKey equivalent data */
uint16_t TouchKey_ExcutSingleConver(uint8_t charg, uint8_t disch)
{
    R8_TKEY_COUNT = (disch << 5) | (charg & 0x1f);
    R8_TKEY_CONVERT = RB_TKEY_START;
    while(R8_TKEY_CONVERT & RB_TKEY_START);
    return (R16_ADC_DATA & RB_ADC_DATA);
}

/* ***************************************************************************
 * @fn ADC_AutoConverCycle
 *
 * @brief Sets the cycle of continuous ADC
 *
 * @param cycle - The sampling period calculation method is (256-cycle)*16*Tsys
 *
 * @return none */
void ADC_AutoConverCycle(uint8_t cycle)
{
    R8_ADC_AUTO_CYCLE = cycle;
}

/* ***************************************************************************
 * @fn ADC_DMACfg
 *
 * @brief Configure DMA function
 *
 * @param s - Whether to turn on the DMA function
 * @param startAddr - DMA Start Address
 * @param endAddr - DMA end address
 * @param m - Configure DMA mode
 *
 * @return none */
void ADC_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, ADC_DMAModeTypeDef m)
{
    if(s == DISABLE)
    {
        R8_ADC_CTRL_DMA &= ~(RB_ADC_DMA_ENABLE | RB_ADC_IE_DMA_END);
    }
    else
    {
        R32_ADC_DMA_BEG = startAddr&0x1FFFF;
        R32_ADC_DMA_END = endAddr&0x1FFFF;
        if(m)
        {
            R8_ADC_CTRL_DMA |= RB_ADC_DMA_LOOP | RB_ADC_IE_DMA_END | RB_ADC_DMA_ENABLE;
        }
        else
        {
            R8_ADC_CTRL_DMA &= ~RB_ADC_DMA_LOOP;
            R8_ADC_CTRL_DMA |= RB_ADC_IE_DMA_END | RB_ADC_DMA_ENABLE;
        }
    }
}

/*********************************************************************
 * @fn      adc_to_temperature_celsius
 *
 * @brief   Convert ADC value to temperature(Celsius)
 *
 * @param   adc_val - adc value
 *
 * @return  temperature (Celsius)
 */

int adc_to_temperature_celsius(uint16_t adc_val)
{
    uint32_t C25 = 0;
    int      temp;

    C25 = (*((PUINT32)ROM_CFG_TMP_25C));

    /* current temperature = standard temperature + (adc deviation * adc linearity coefficient) */ 
    temp = (((C25 >> 16) & 0xFFFF) ? ((C25 >> 16) & 0xFFFF) : 25) + \
        (adc_val - ((int)(C25 & 0xFFFF))) * 100 / 283;

    return (temp);
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_MINUS_12dB
 *
 * @brief -12dB gain ADC single-ended sampling value is converted to voltage (mV)
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_MINUS_12dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+256)/512 - 3*1050);
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_MINUS_6dB
 *
 * @brief -6dB gain ADC single-ended sampling value is converted to voltage (mV)
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_MINUS_6dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+512)/1024 - 1*1050);
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_0dB
 *
 * @brief The ADC single-ended sampling value is converted to voltage (mV) at 0dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_0dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+1024)/2048);
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_6dB
 *
 * @brief The ADC single-ended sample value is converted to voltage (mV) at 6dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_6dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+2048)/4096 + 525);
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_12dB
 *
 * @brief The ADC single-ended sampling value is converted to voltage (mV) at 12dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_12dB(uint16_t adc_data)
{
    return (((int)adc_data*1050)/8192 + 788);  //787.5
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_18dB
 *
 * @brief The ADC single-ended sample value is converted to voltage (mV) at 18dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_18dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+4096)/16384 + 919);  //918.75
}

/* ***************************************************************************
 * @fn ADC_VoltConverSignalPGA_24dB
 *
 * @brief The ADC single-ended sample value is converted to voltage (mV) at 24dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverSignalPGA_24dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+28672)/32768 + 984);  //984.375
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_MINUS_12dB
 *
 * @brief - ADC differential sampling value is converted to voltage (mV) at 12dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_MINUS_12dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+256)/512 - 4*1050);
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_MINUS_6dB
 *
 * @brief -6dB gain ADC differential sampling value is converted to voltage (mV)
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_MINUS_6dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+512)/1024 - 2*1050);
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_0dB
 *
 * @brief The ADC differential sampling value is converted to voltage (mV) at 0dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_0dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+1024)/2048 - 1*1050);
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_6dB
 *
 * @brief The ADC differential sampling value is converted to voltage (mV) at 6dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_6dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+2048)/4096 - 525);
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_12dB
 *
 * @brief The ADC differential sampling value is converted to voltage (mV) at 12dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_12dB(uint16_t adc_data)
{
    return (((int)adc_data*1050)/8192 - 262);  //262.5
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_18dB
 *
 * @brief The ADC differential sampling value is converted to voltage (mV) at 18dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_18dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+4096)/16384 - 131);  //131.25
}

/* ***************************************************************************
 * @fn ADC_VoltConverDiffPGA_24dB
 *
 * @brief The ADC differential sampling value is converted to voltage (mV) at 24dB gain
 *
 * @param adc_data - ADC sampling value
 *
 * @return voltage (mV) */
int ADC_VoltConverDiffPGA_24dB(uint16_t adc_data)
{
    return (((int)adc_data*1050+28672)/32768 - 66);  //65.625
}
