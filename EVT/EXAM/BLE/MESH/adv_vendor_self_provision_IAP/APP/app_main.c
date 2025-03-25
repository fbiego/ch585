/* ********************************* (C) COPYRIGHT ***************************
 * File Name : main.c
 * Author: WCH
 * Version: V1.1
 * Date: 2019/11/05
 * Description: Judgment flags and transfer codes to the APP code area
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/******************************************************************************/
/* The header file contains */
#include "CH58x_common.h"
#include "app.h"
void dbg_printf(const char* format, ...);

/* Record the current Image */
unsigned char CurrImageFlag = 0xff;

/* Flash's data temporary storage */
__attribute__((aligned(8))) uint8_t block_buf[16];

#define jumpApp    ((void (*)(void))((int *)IMAGE_A_START_ADD))

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/* ***************************************************************************
 * @fn SwitchImageFlag
 *
 * @brief Toggle ImageFlag in dataflash
 *
 * @param new_flag - ImageFlag toggle
 *
 * @return none */
void SwitchImageFlag(uint8_t new_flag)
{
    uint16_t i;
    uint32_t ver_flag;

    /* Read the first block */
    EEPROM_READ(OTA_DATAFLASH_ADD, (uint32_t *)&block_buf[0], 4);

    /* Erase the first piece */
    EEPROM_ERASE(OTA_DATAFLASH_ADD, EEPROM_PAGE_SIZE);

    /* Update Image Information */
    block_buf[0] = new_flag;

    /* Programming DataFlash */
    EEPROM_WRITE(OTA_DATAFLASH_ADD, (uint32_t *)&block_buf[0], 4);
}

/* ***************************************************************************
 * @fn jump_APP
 *
 * @brief Switch APP program
 *
 * @return none */
void jump_APP(void)
{
    if(CurrImageFlag == IMAGE_IAP_FLAG)
    {
        __attribute__((aligned(8))) uint8_t flash_Data[1024];

        uint8_t i;
        FLASH_ROM_ERASE(IMAGE_A_START_ADD, IMAGE_A_SIZE);
        for(i = 0; i < IMAGE_A_SIZE / 1024; i++)
        {
            FLASH_ROM_READ(IMAGE_B_START_ADD + (i * 1024), flash_Data, 1024);
            FLASH_ROM_WRITE(IMAGE_A_START_ADD + (i * 1024), flash_Data, 1024);
        }
        SwitchImageFlag(IMAGE_A_FLAG);
        // Destroy the backup code
        FLASH_ROM_ERASE(IMAGE_B_START_ADD, IMAGE_A_SIZE);
    }
    jumpApp();
}

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
    if((CurrImageFlag != IMAGE_A_FLAG) && (CurrImageFlag != IMAGE_B_FLAG) && (CurrImageFlag != IMAGE_IAP_FLAG))
    {
        CurrImageFlag = IMAGE_A_FLAG;
    }

#ifdef DEBUG
    dbg_printf("Image Flag %02x\n", CurrImageFlag);
#endif
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
    ReadImageFlag();
    jump_APP();
}

/******************************** endfile @ main ******************************/
