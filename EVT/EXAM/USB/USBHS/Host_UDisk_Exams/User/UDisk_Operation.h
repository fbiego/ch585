/********************************** (C) COPYRIGHT  *******************************
* File Name          : UDisk_Operation.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : This file contains all the functions prototypes for the Udisk 
                       host operation.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef USER_UDISK_OPERATION_H_
#define USER_UDISK_OPERATION_H_

#include "CH58x_common.h"
#include "stdio.h"
#include "string.h"
#include "CHRV3UFI.h"
#include "ch585_usbhs_host.h"
#include "usb_host_config.h"

/*******************************************************************************/
/* Public Extern Variables */
extern volatile uint8_t          UDisk_Opeation_Flag;
extern struct   _ROOT_HUB_DEVICE RootHubDev[ DEF_TOTAL_ROOT_HUB ];
extern struct   __HOST_CTL       HostCtl[ DEF_TOTAL_ROOT_HUB * DEF_ONE_USB_SUP_DEV_TOTAL ];
extern volatile uint8_t          UDisk_Opeation_Flag;
extern uint8_t  *pCodeStr;

extern __attribute__((aligned(4)))  uint8_t  Com_Buffer[ DEF_COM_BUF_LEN ];     // even address , used for host enumcation and udisk operation
extern __attribute__((aligned(4)))  uint8_t  DevDesc_Buf[ 18 ];                 // Device Descriptor Buffer

/*******************************************************************************/
/* Macro definitions related to long file names are extremely global variables */
// Long filename buffer from (0 to 20)*26
#define     LONG_NAME_BUF_LEN       (20*26)
#define     UNICODE_ENDIAN          0           // 1 is UNICDOE big endian encoding 0 is small endian
// Long file name storage buffer (Unicode encoding)
extern uint8_t LongNameBuf[ ];
// Long file name (Unicode encoding)
extern uint8_t LongName[ ];
#define     LongName_Len            124
#define     TRUE                    1
#define     FALSE                   0

// Function returns
#define     ERR_NO_NAME             0X44        // This short file name has no long file name or wrong long file
#define     ERR_BUF_OVER            0X45        // Long file buffer overflow
#define     ERR_LONG_NAME           0X46        // Incorrect long file name
#define     ERR_NAME_EXIST          0X47        // This short file name exists

/*******************************************************************************/
/* Extern UDisk Operation Functions */
extern void mStopIfError( uint8_t iError );
extern void Udisk_USBH_Initialization( void );
extern uint8_t Udisk_USBH_EnumRootDevice( uint8_t usb_port );
extern uint8_t UDisk_USBH_PreDeal( void );
extern uint8_t UDisk_USBH_DiskReady( void );

/* Extern Long-name Operation Functions */
extern void UDisk_USBH_Longname( void );
extern uint8_t CHRV3GetLongName( void );
extern uint8_t GetUpSectorData( uint32_t *NowSector );
extern uint8_t CHRV3CreateLongName( void );
extern uint8_t AnalyzeLongName( void );
extern uint8_t CheckNameSum( uint8_t *p );

/* Extern Creating Directory Functions */
extern void UDisk_USBH_CreatDirectory( void );
extern uint8_t CreateDirectory( void );

/* Extern Byte/Sector Open/Read/Modify/Delete and File-Enumeration Functions */
extern void UDisk_USBH_ByteOperation( void );
extern void UDisk_USBH_SectorOperation( void );
extern void UDisk_USBH_EnumFiles( void );

#endif /* USER_UDISK_OPERATION_H_ */
