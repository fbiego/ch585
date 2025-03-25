/* ********************************* (C) COPYRIGHT ***************************
 * File Name : main.c
 * Author: WCH
 * Version: V1.1
 * Date: 2024/11/14
 * Description: NFC PCD Mifare Classic Test Routine
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************************* */

/******************************************************************************/
/* The header file contains */
#include "CH58x_common.h"
#include "wch_nfca_mifare_classic.h"
#include "wch_nfca_pcd_bsp.h"

/* Each file has a separate debug print switch, setting 0 can prohibit internal printing of this file. */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
uint8_t default_key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t picc_uid[4];

/* ***************************************************************************
 * @fn sys_get_vdd
 *
 * @brief System voltage detection
 *
 * @param none
 *
 * @return Detected ADC value */
uint16_t sys_get_vdd(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_VBAT;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0 << 4);    /* Use -12dB mode */
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT |= (3 << 4);                                 /* 7ä¸ªTadc */
    R8_ADC_CONVERT |= RB_ADC_START;
    while (R8_ADC_CONVERT & RB_ADC_START);
    adc_data = R16_ADC_DATA;

    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;
    return (adc_data);
}

/* ***************************************************************************
 * @fn nfca_pcd_test
 *
 * @brief nfc-a pcd card reader test function
 *
 * @param none
 *
 * @return none */
void nfca_pcd_test(void)
{
    uint16_t res;
    uint16_t adc_vdd;
    int vdd_value;

    adc_vdd = sys_get_vdd();
    vdd_value = ADC_VoltConverSignalPGA_MINUS_12dB(adc_vdd);
    PRINTF("vdd_value: %d\n", vdd_value);
    if(vdd_value > 3400)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL0);
        PRINTF("LV0\n");
    }
    else if(vdd_value > 3000)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL1);
        PRINTF("LV1\n");
    }
    else if(vdd_value > 2600)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL2);
        PRINTF("LV2\n");
    }
    else
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL3);
        PRINTF("LV3\n");
    }

    while(1)
    {
        nfca_pcd_start();

#if 1   /* Set the ultra-low power card first to perform the ultra-low power card detection. Devices with little impact on the antenna signal amplitude may not be awakened. */
        if(nfca_pcd_lpcd_check() == 0)
        {
            PRINTF("NO CARD\n");
            goto next_loop;
        }
        PRINTF("CARD DETECT\n");
#endif
        mDelaymS(5);   /* Mobile phones and other analog card devices require long-term continuous wave wake-up of their card function, and ordinary physical cards can be 1 ms. */

#if NFCA_PCD_USE_NFC_CTR_PIN
        nfca_pcd_ctr_handle();  /* Detect the antenna signal and control the amplitude using the NFC CTR pin */
#endif

        res = PcdRequest(PICC_REQALL);
        if(res == 0x0004)
        {
            res = PcdAnticoll(PICC_ANTICOLL1);
            if (res == PCD_NO_ERROR)
            {
                picc_uid[0] = g_nfca_pcd_recv_buf[0];
                picc_uid[1] = g_nfca_pcd_recv_buf[1];
                picc_uid[2] = g_nfca_pcd_recv_buf[2];
                picc_uid[3] = g_nfca_pcd_recv_buf[3];
                PRINTF("uid: %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3]);

                res = PcdSelect(PICC_ANTICOLL1, picc_uid);
                if (res == PCD_NO_ERROR)
                {
                    PRINTF("\nselect OK, SAK:%02x\n", g_nfca_pcd_recv_buf[0]);

#if 1   /* Read the first 4 blocks of data test */
                    res = PcdAuthState(PICC_AUTHENT1A, 0, default_key, picc_uid);
                    if (res != PCD_NO_ERROR)
                    {
                        goto nfc_exit;
                    }

                    for (uint8_t i = 0; i < 4; i++)
                    {
                        res = PcdRead(i);
                        if (res != PCD_NO_ERROR)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }
                        PRINTF("block %02d: ", i);
                        for (uint8_t j = 0; j < 16; j++)
                        {
                            PRINTF("%02x ", g_nfca_pcd_recv_buf[j]);
                        }
                        PRINTF("\n");
                    }

#if 0   /* Value block reading and initialization test */

                    res = PcdReadValueBlock(1);
                    if (res == PCD_VALUE_BLOCK_INVALID)
                    {
                        PRINTF("not a value block, init it.");
                        uint32_t vdata = 100;
                        res = PcdInitValueBlock(1, (uint8_t *)&vdata, 2);
                        if (res != PCD_NO_ERROR)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }
                    }
                    else if (res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n", res);
                        goto nfc_exit;
                    }
                    else
                    {
                        PRINTF("value:%d, adr:%d\n", PU32_BUF(g_nfca_pcd_recv_buf)[0], g_nfca_pcd_recv_buf[12]);
                    }

#endif  /* Value block reading and initialization test */

#if 0   /* Value block deduction and backup test */
                    PRINTF("PcdValue\n");
                    uint32_t di_data = 1;
                    res = PcdValue(PICC_DECREMENT, 1, (uint8_t *)&di_data);
                    if(res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n",res);
                        goto nfc_exit;
                    }
                    PRINTF("PcdBakValue\n");
                    res = PcdBakValue(1,2);
                    if(res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n",res);
                        goto nfc_exit;
                    }

#endif  /* Value block deduction and backup test */

#endif  /* Read the first 4 blocks of data test */

#if 1   /* All sectors read test */
                    for (uint8_t l = 1; l < 16; l++)
                    {
                        res = PcdAuthState(PICC_AUTHENT1A, 4 * l, default_key, picc_uid);
                        if (res)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }

                        PRINTF("read:\n");
                        for (uint8_t i = 0; i < 3; i++)
                        {
                            res = PcdRead(i + 4 * l);
                            if (res)
                            {
                                PRINTF("ERR: 0x%x\n", res);
                                goto nfc_exit;
                            }
                            PRINTF("block %02d: ", i + 4 * l);
                            for (uint8_t j = 0; j < 16; j++)
                            {
                                PRINTF("%02x ", g_nfca_pcd_recv_buf[j]);
                            }
                            PRINTF("\n");
                        }
                    }
#endif  /* All sectors read test */

nfc_exit:
                    PcdHalt();
                }
            }
        }
next_loop:
        nfca_pcd_stop();
        mDelaymS(500);
    }
}

/* ***************************************************************************
 * @fn main
 *
 * @brief main function
 *
 * @return none */
int main(void)
{
    UINT16 x;
    PWR_DCDCCfg(ENABLE);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif

    PRINT("NFCA PCD START\n");

    nfca_pcd_init();

    nfca_pcd_lpcd_calibration();    /* The ADC value calibration of the low-power inspection card is related to antennas, etc., and can be detected during production. */

    nfca_pcd_test();

    while(1);
}

/******************************** endfile @ main ******************************/
