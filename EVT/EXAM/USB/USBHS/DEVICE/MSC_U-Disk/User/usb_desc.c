/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : usb device descriptor,configuration descriptor,
 *                      string descriptors and other descriptors.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "usb_desc.h"

/* Device Descriptor */
const uint8_t  MyDevDescr[ ] =
{
    0x12,                                                                        // bLength
    0x01,       // bDescriptorType (Device)
    0x00, 0x02,                                                                  // bcdUSB 2.00
    0x00,                                                                        // bDeviceClass (Use class information in the Interface Descriptors)
    0x00,                                                                        // bDeviceSubClass
    0x00,                                                                        // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,                                                          // bMaxPacketSize0 64
    (uint8_t)DEF_USB_VID, (uint8_t)(DEF_USB_VID >> 8),                           // idVendor  0x1A86
    (uint8_t)DEF_USB_PID, (uint8_t)(DEF_USB_PID >> 8),                           // idProduct 0xFE10
    DEF_IC_PRG_VER, 0x00,                                                        // bcdDevice 0.01
    0x01,                                                                        // iManufacturer (String Index)
    0x02,                                                                        // iProduct (String Index)
    0x00,                                                                        // iSerialNumber (String Index)
    0x01,                                                                        // bNumConfigurations 1
};

/* USB≈‰÷√√Ë ˆ∑˚(∏ﬂÀŸ) */
const uint8_t  MyCfgDescr_HS[ ] =
{
    /* Configuration Descriptor */
    0x09,                                                                        // bLength
    0x02,                                                                        // bDescriptorType (Configuration)
    0x20, 0x00,                                                                  // wTotalLength 32
    0x01,                                                                        // bNumInterfaces 1
    0x01,                                                                        // bConfigurationValue
    0x00,                                                                        // iConfiguration (String Index)
    0xC0,                                                                        // bmAttributes Self Powered
    0x32,                                                                        // bMaxPower 100mA

    /*****************************************************************/
    /* Interface Descriptor(UDisk) */
    0x09,                                                                        // bLength
    0x04,                                                                        // bDescriptorType (Interface)
    0x00,                                                                        // bInterfaceNumber 0
    0x00,                                                                        // bAlternateSetting
    0x02,                                                                        // bNumEndpoints 2
    0x08,                                                                        // bInterfaceClass
    0x06,                                                                        // bInterfaceSubClass
    0x50,                                                                        // bInterfaceProtocol
    0x00,                                                                        // iInterface (String Index)

    /* Endpoint Descriptor */
    0x07,                                                                        // bLength
    0x05,                                                                        // bDescriptorType (Endpoint)
    0x82,                                                                        // bEndpointAddress (IN/D2H)
    0x02,                                                                        // bmAttributes (Bulk)
    0x00, 0x02,                                                                  // wMaxPacketSize 512
    0x00,                                                                        // bInterval 0 (unit depends on device speed)

    /* Endpoint Descriptor */
    0x07,                                                                        // bLength
    0x05,                                                                        // bDescriptorType (Endpoint)
    0x03,                                                                        // bEndpointAddress (OUT/H2D)
    0x02,                                                                        // bmAttributes (Bulk)
    0x00, 0x02,                                                                  // wMaxPacketSize 512
    0x00,                                                                        // bInterval 0 (unit depends on device speed)

};

/* Configuration Descriptor */
const uint8_t  MyCfgDescr_FS[ ] =
{
   /* Configuration Descriptor */
    0x09,                                                                        // bLength
    0x02,                                                                        // bDescriptorType (Configuration)
    0x20, 0x00,                                                                  // wTotalLength 32
    0x01,                                                                        // bNumInterfaces 1
    0x01,                                                                        // bConfigurationValue
    0x00,                                                                        // iConfiguration (String Index)
    0xC0,                                                                        // bmAttributes Self Powered
    0x32,                                                                        // bMaxPower 100mA

    /*****************************************************************/
    /* Interface Descriptor(UDisk) */
    0x09,                                                                        // bLength
    0x04,                                                                        // bDescriptorType (Interface)
    0x00,                                                                        // bInterfaceNumber 0
    0x00,                                                                        // bAlternateSetting
    0x02,                                                                        // bNumEndpoints 2
    0x08,                                                                        // bInterfaceClass
    0x06,                                                                        // bInterfaceSubClass
    0x50,                                                                        // bInterfaceProtocol
    0x00,                                                                        // iInterface (String Index)

    /* Endpoint Descriptor */
    0x07,                                                                        // bLength
    0x05,                                                                        // bDescriptorType (Endpoint)
    0x82,                                                                        // bEndpointAddress (IN/D2H)
    0x02,                                                                        // bmAttributes (Bulk)
    0x40, 0x00,                                                                  // wMaxPacketSize 64
    0x00,                                                                        // bInterval 0 (unit depends on device speed)

    /* Endpoint Descriptor */
    0x07,                                                                        // bLength
    0x05,                                                                        // bDescriptorType (Endpoint)
    0x03,                                                                        // bEndpointAddress (OUT/H2D)
    0x02,                                                                        // bmAttributes (Bulk)
    0x40, 0x00,                                                                  // wMaxPacketSize 64
    0x00,                                                                        // bInterval 0 (unit depends on device speed)

};

/* Language Descriptor */
const uint8_t  MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  MyManuInfo[] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const uint8_t  MyProdInfo[] =
{
        0x16, 0x03, 'C', 0, 'H', 0, '5', 0, '8', 0, '5', 0, 'U', 0
                  , 'D', 0, 'i', 0, 's', 0, 'k', 0,
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[] =
{
    0x16, 0x03, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0
              , '6', 0, '7', 0, '8', 0, '9', 0
};

/* Device Qualified Descriptor */
const uint8_t MyQuaDesc[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x40, 0x01, 0x00,
};

/* Device BOS Descriptor */
const uint8_t MyBOSDesc[ ] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[ sizeof(MyCfgDescr_HS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

/* USB High-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_HS_OSC_DESC[ sizeof(MyCfgDescr_FS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};
