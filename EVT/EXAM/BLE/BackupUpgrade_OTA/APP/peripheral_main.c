/* ********************************* (C) COPYRIGHT ***************************
 * File Name : main.c
 * Author: WCH
 * Version: V1.1
 * Date: 2019/11/05
 * Description: Peripheral slave application master function and task system initialization
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/******************************************************************************/
/* The header file contains */
#include "CONFIG.h"
#include "HAL.h"
#include "GATTprofile.h"
#include "Peripheral.h"
#include "OTA.h"
#include "OTAprofile.h"

/* Record the current Image */
unsigned char CurrImageFlag = 0xff;

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] =
    {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/* Used to determine the validity of files on the APP */
const uint32_t Address = 0xFFFFFFFF;

__attribute__((aligned(4))) uint32_t Image_Flag __attribute__((section(".ImageFlag"))) = (uint32_t)&Address;

/* Note: The flash operation must be executed first after the program is upgraded, and no interrupts must be enabled to prevent interruptions and failures of operations. */
/* ***************************************************************************
 * @fn ReadImageFlag
 *
 * @brief Reads the Image flag of the current program. If DataFlash is empty, it is ImageA by default.
 *
 * @return none */
void ReadImageFlag(void)
{
    OTADataFlashInfo_t p_image_flash;

    EEPROM_READ(OTA_DATAFLASH_ADD, &p_image_flash, 4);
    CurrImageFlag = p_image_flash.ImageFlag;

    /* The program is executed for the first time, or has not been updated, and the DataFlash is erased after the update is updated. */
    if((CurrImageFlag != IMAGE_A_FLAG) && (CurrImageFlag != IMAGE_B_FLAG))
    {
        CurrImageFlag = IMAGE_A_FLAG;
    }
}

/* ***************************************************************************
 * @fn Main_Circulation
 *
 * @brief main loop
 *
 * @return none */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
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
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif
#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif
    PRINT("%s\n", VER_LIB);
    ReadImageFlag();
    CH58x_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    Peripheral_Init();
    Main_Circulation();
}

/******************************** endfile @ main ******************************/
