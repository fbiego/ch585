/* ********************************* (C) COPYRIGHT ***************************
 * File Name: EXAM11.C
 * Author: WCH
 * Version: V1.0
 * Date: 2020/08/11
 * Description: CH573 C language USB directory file enumeration program
 * Support: FAT12/FAT16/FAT32
 * Note that CHRV3UFI.LIB/USBHOST.C/DEBUG.C is included
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/* * Do not use the U disk file system library, need to modify DISK_LIB_ENABLE=0 in precompilation of engineering properties. */
/* * When the USB drive is mounted under USBhub, you need to modify DISK_WITHOUT_USB_HUB=0 in the precompilation of engineering properties. */

#include "CH58x_common.h"
#include "CHRV3UFI.H"

__attribute__((aligned(4))) uint8_t RxBuffer[MAX_PACKET_SIZE]; // IN, must even address
__attribute__((aligned(4))) uint8_t TxBuffer[MAX_PACKET_SIZE]; // OUT, must even address

/* ***************************************************************************
 * @fn mStopIfError
 *
 * @brief Check operation status, if it is wrong, the error code will be displayed and the operation will be shut down
 *
 * @param iError - Error code
 *
 * @return none */
void mStopIfError(uint8_t iError)
{
    if(iError == ERR_SUCCESS)
    {
        return; /* Operation is successful */
    }
    PRINT("Error: %02X\n", (uint16_t)iError); /* Display error */
    /* After encountering an error, the error code and CHRV3DiskStatus status should be analyzed. For example, call CHRV3DiskReady to check whether the current USB disk is connected. If the USB disk is disconnected, wait for the USB disk to be plugged in before operation.
     Recommended steps to deal with errors:
     1. Call CHRV3DiskReady once, and continue to operate successfully, such as Open, Read/Write, etc.
     2. If CHRV3DiskReady fails, then the operation will be forced to start from scratch (waiting for the USB drive connection, CHRV3DiskReady, etc.) */
    while(1)
    {
    }
}

/* ***************************************************************************
 * @fn main
 *
 * @brief main function
 *
 * @return none */
int main()
{
    uint8_t  s, i;
    uint8_t *pCodeStr;
    uint16_t j;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
    PRINT("Start @ChipID=%02X \n", R8_CHIP_ID);

    pHOST_RX_RAM_Addr = RxBuffer;
    pHOST_TX_RAM_Addr = TxBuffer;
    USB_HostInit();
    CHRV3LibInit(); // Initialize the USB disk library to support USB disk files

    FoundNewDev = 0;
    PRINT("Wait Device In\n");
    while(1)
    {
        s = ERR_SUCCESS;
        if(R8_USB_INT_FG & RB_UIF_DETECT) // If there is a USB host detection interrupt, it will be handled
        {
            R8_USB_INT_FG = RB_UIF_DETECT; // Clear connection interrupt flag
            s = AnalyzeRootHub();          // Analyze ROOT-HUB status
            if(s == ERR_USB_CONNECT)
            {
                FoundNewDev = 1;
            }
        }

        if(FoundNewDev || s == ERR_USB_CONNECT)
        {
            // There is a new USB device plugged in
            FoundNewDev = 0;
            mDelaymS(200);        // Since the USB device has just been plugged in, it is not stable yet, so wait for the USB device to be hundreds of milliseconds to eliminate plug-in and unplug jitter
            s = InitRootDevice(); // Initialize USB device
            if(s == ERR_SUCCESS)
            {
                PRINT("Start UDISK_demo @CHRV3UFI library\n");
                // USB drive operation process: USB bus reset, USB drive connection, obtain device descriptors and set USB address, optional obtain configuration descriptors, and then arrive here, and the CHRV3 subroutine library continues to complete the subsequent work.
                CHRV3DiskStatus = DISK_USB_ADDR;
                for(i = 0; i != 10; i++)
                {
                    PRINT("Wait DiskReady\n");
                    s = CHRV3DiskReady();
                    if(s == ERR_SUCCESS)
                    {
                        break;
                    }
                    mDelaymS(50);
                }
                if(CHRV3DiskStatus >= DISK_MOUNTED) // The USB drive is ready
                {
                    /* Read the file */
                    PRINT("Open\n");
                    strcpy((uint8_t *)mCmdParam.Open.mPathName, "/C51/CHRV3HFT.C"); // Set the file name and path to operate
                    s = CHRV3FileOpen();                                            // Open the file
                    if(s == ERR_MISS_DIR)
                    {
                        PRINT("不存在该文件夹则列出根目录所有文件\n");
                        pCodeStr = (uint8_t *)"/* ";
                    }
                    else
                    {
                        pCodeStr = (uint8_t *)"/C51/*"; //List the files under the \C51 subdirectory
                    }

                    PRINT("List file %s\n", pCodeStr);
                    for(j = 0; j < 10000; j++) // Limited to 10000 files, there is actually no limit
                    {
                        strcpy((uint8_t *)mCmdParam.Open.mPathName, (const uint8_t *)pCodeStr); //Search for file name, * is a wildcard character, applicable to all files or subdirectories
                        i = strlen((uint8_t *)mCmdParam.Open.mPathName);
                        mCmdParam.Open.mPathName[i] = 0xFF; //Replace the ending character with the search sequence number according to the length of the string, from 0 to 254. If it is 0xFF, that is, 255, it means that the search sequence number is in the CHRV3vFileSize variable
                        CHRV3vFileSize = j; //Specify the sequence number of search/enum
                        i = CHRV3FileOpen(); //Open the file. If the file name contains wildcard characters*, it will not open it to search for the file.
                        /* The only difference between CHRV3FileEnum and CHRV3FileOpen is that when the latter returns ERR_FOUND_NAME, it returns ERR_SUCCESS corresponding to the former */
                        if(i == ERR_MISS_FILE)
                        {
                            break; // No matching file can be searched anymore, no matching file name is available
                        }
                        if(i == ERR_FOUND_NAME) // Search for file names matching wildcard characters, file names and their full paths in the command buffer
                        {
                            PRINT("  match file %04d#: %s\n", (unsigned int)j, mCmdParam.Open.mPathName); /* Display the serial number and the searched matching file name or subdirectory name */
                            continue;                                                                      /* Continue to search for the next matching file name, and the next time you search, the serial number will be added 1 */
                        }
                        else // An error occurred
                        {
                            mStopIfError(i);
                            break;
                        }
                    }
                    i = CHRV3FileClose(); // Close the file
                    PRINT("U盘演示完成\n");
                }
                else
                {
                    PRINT("U盘没有准备好 ERR =%02X\n", (uint16_t)s);
                }
            }
            else
            {
                PRINT("初始化U盘失败，请拔下U盘重试\n");
            }
        }
        mDelaymS(100);  // Simulate a microcontroller to do other things
        SetUsbSpeed(1); // Default is full speed
    }
}
