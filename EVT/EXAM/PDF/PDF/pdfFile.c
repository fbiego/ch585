/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2023/07/26
 * Description        : pdfFile
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* Header Files */
#include "string.h"
#include "pdfFile.h"
#include "SW_UDISK.h"
#include "Internal_Flash.h"
#include "CHRV3UFI.h"
#include "usbfs_device.h"


uint32_t data_len;
uint32_t data_tollen;
__attribute__ ((aligned(4)))  uint8_t data_buf[PDF_TMP_BUF_LEN_MAX+PDF_TMP_BUF_LEN_EXT];

const uint8_t sec0[] ={
    0xeb,0x3c,0x90,0x4d,0x53,0x44,0x4f,0x53,0x35,0x2e,0x30,0x00,0x02,0x01,0x04,0x00,
    0x02,0x00,0x02,0x80,0x02,0xf8,0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x80,0x00,0x29,0xff,0x32,0xc6,0x0c,0x4e,0x4f,0x20,0x4e,0x41,
    0x4d,0x45,0x20,0x20,0x20,0x20,0x46,0x41,0x54,0x31,0x32,0x20,0x20,0x20,0x33,0xc9,
    0x8e,0xd1,0xbc,0xf0,0x7b,0x8e,0xd9,0xb8,0x00,0x20,0x8e,0xc0,0xfc,0xbd,0x00,0x7c,
    0x38,0x4e,0x24,0x7d,0x24,0x8b,0xc1,0x99,0xe8,0x3c,0x01,0x72,0x1c,0x83,0xeb,0x3a,
    0x66,0xa1,0x1c,0x7c,0x26,0x66,0x3b,0x07,0x26,0x8a,0x57,0xfc,0x75,0x06,0x80,0xca,
    0x02,0x88,0x56,0x02,0x80,0xc3,0x10,0x73,0xeb,0x33,0xc9,0x8a,0x46,0x10,0x98,0xf7,
    0x66,0x16,0x03,0x46,0x1c,0x13,0x56,0x1e,0x03,0x46,0x0e,0x13,0xd1,0x8b,0x76,0x11,
    0x60,0x89,0x46,0xfc,0x89,0x56,0xfe,0xb8,0x20,0x00,0xf7,0xe6,0x8b,0x5e,0x0b,0x03,
    0xc3,0x48,0xf7,0xf3,0x01,0x46,0xfc,0x11,0x4e,0xfe,0x61,0xbf,0x00,0x00,0xe8,0xe6,
    0x00,0x72,0x39,0x26,0x38,0x2d,0x74,0x17,0x60,0xb1,0x0b,0xbe,0xa1,0x7d,0xf3,0xa6,
    0x61,0x74,0x32,0x4e,0x74,0x09,0x83,0xc7,0x20,0x3b,0xfb,0x72,0xe6,0xeb,0xdc,0xa0,
    0xfb,0x7d,0xb4,0x7d,0x8b,0xf0,0xac,0x98,0x40,0x74,0x0c,0x48,0x74,0x13,0xb4,0x0e,
    0xbb,0x07,0x00,0xcd,0x10,0xeb,0xef,0xa0,0xfd,0x7d,0xeb,0xe6,0xa0,0xfc,0x7d,0xeb,
    0xe1,0xcd,0x16,0xcd,0x19,0x26,0x8b,0x55,0x1a,0x52,0xb0,0x01,0xbb,0x00,0x00,0xe8,
    0x3b,0x00,0x72,0xe8,0x5b,0x8a,0x56,0x24,0xbe,0x0b,0x7c,0x8b,0xfc,0xc7,0x46,0xf0,
    0x3d,0x7d,0xc7,0x46,0xf4,0x29,0x7d,0x8c,0xd9,0x89,0x4e,0xf2,0x89,0x4e,0xf6,0xc6,
    0x06,0x96,0x7d,0xcb,0xea,0x03,0x00,0x00,0x20,0x0f,0xb6,0xc8,0x66,0x8b,0x46,0xf8,
    0x66,0x03,0x46,0x1c,0x66,0x8b,0xd0,0x66,0xc1,0xea,0x10,0xeb,0x5e,0x0f,0xb6,0xc8,
    0x4a,0x4a,0x8a,0x46,0x0d,0x32,0xe4,0xf7,0xe2,0x03,0x46,0xfc,0x13,0x56,0xfe,0xeb,
    0x4a,0x52,0x50,0x06,0x53,0x6a,0x01,0x6a,0x10,0x91,0x8b,0x46,0x18,0x96,0x92,0x33,
    0xd2,0xf7,0xf6,0x91,0xf7,0xf6,0x42,0x87,0xca,0xf7,0x76,0x1a,0x8a,0xf2,0x8a,0xe8,
    0xc0,0xcc,0x02,0x0a,0xcc,0xb8,0x01,0x02,0x80,0x7e,0x02,0x0e,0x75,0x04,0xb4,0x42,
    0x8b,0xf4,0x8a,0x56,0x24,0xcd,0x13,0x61,0x61,0x72,0x0b,0x40,0x75,0x01,0x42,0x03,
    0x5e,0x0b,0x49,0x75,0x06,0xf8,0xc3,0x41,0xbb,0x00,0x00,0x60,0x66,0x6a,0x00,0xeb,
    0xb0,0x42,0x4f,0x4f,0x54,0x4d,0x47,0x52,0x20,0x20,0x20,0x20,0x0d,0x0a,0x52,0x65,
    0x6d,0x6f,0x76,0x65,0x20,0x64,0x69,0x73,0x6b,0x73,0x20,0x6f,0x72,0x20,0x6f,0x74,
    0x68,0x65,0x72,0x20,0x6d,0x65,0x64,0x69,0x61,0x2e,0xff,0x0d,0x0a,0x44,0x69,0x73,
    0x6b,0x20,0x65,0x72,0x72,0x6f,0x72,0xff,0x0d,0x0a,0x50,0x72,0x65,0x73,0x73,0x20,
    0x61,0x6e,0x79,0x20,0x6b,0x65,0x79,0x20,0x74,0x6f,0x20,0x72,0x65,0x73,0x74,0x61,
    0x72,0x74,0x0d,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xac,0xcb,0xd8,0x55,0xaa,
};
// SEC4/6
const uint8_t  BPB_Media[] =
{
    0xf8,0xff,0xff,0xff,0xff,0xff
};

/* ***************************************************************************
 * @fn pdf_memcpy
 *
 * @brief Data copying
 *
 * @param pDst - Destination address
            pSrc - Original address
            len - Number of bytes to copy
 *
 * @return none */
__HIGH_CODE
void pdf_memcpy( void *pDst, const void *pSrc, uint32_t len )
{
  uint8_t *dst;
  const uint8_t *src;
  if( pDst == 0 || pSrc == 0 || len == 0 )
  {
     return;
  }
  src = (uint8_t *)pSrc;
  dst = (uint8_t *)pDst;
  do
  {
    *dst++ = *src++;
  }
  while( --len );
}

/*********************************************************************
 * @fn      udisk_enable
 *
 * @brief   Enable USB_Device and Udisk
 *
 * @param   none
 *
 * @return  none
 */
void udisk_enable( void )
{
#if  FUN_UDISK
    /* USBOTG_FS device init */
    USBFS_Device_Init( );
#endif
}

/*********************************************************************
 * @fn      udisk_format
 *
 * @brief   Udisk format
 *
 * @param   none
 *
 * @return  none
 */
void udisk_format( void )
{
    uint32_t addr = IFLASH_UDISK_START_ADDR;

    for( int sec=0;sec<40;sec++ )
    {
        memset(pDISK_FAT_BUF,0,DEF_UDISK_SECTOR_SIZE);
        if( sec == 0 )
        {
            pdf_memcpy( pDISK_FAT_BUF,sec0,sizeof(sec0) );
        }
        else if( sec == 4 || sec == 6 )
        {
            pdf_memcpy( pDISK_FAT_BUF,BPB_Media,sizeof(BPB_Media) );
        }
        IFlash_Prog_512( addr,(uint32_t*)pDISK_FAT_BUF);
        addr += DEF_UDISK_SECTOR_SIZE;
    }
}

/* ***************************************************************************
 * @fn mStopIfError
 *
 * @brief Checking the operation status, displaying the error code and stopping if there is an error
 * input : iError - Error code input
 *
 * @param iError - Error code
 *
 * @return none */
#if FUN_FILE_CREATE
static void mStopIfError( uint8_t iError )
{
    if ( iError == ERR_SUCCESS )
    {
        /* operation success, return */
        return;
    }
    /* Display the errors */
    printf( "Error:%02x\r\n", iError );
    /* After encountering an error, you should analyze the error code and CH103DiskReday status, for example,
     * call CH103DiskReday to check whether the current USB disk is connected or not,
     * if the disk is disconnected then wait for the disk to be plugged in again and operate again,
     * Suggested steps to follow after an error:
     *     1,call CH103DiskReday once, if successful, then continue the operation, such as Open, Read/Write, etc.
     *     2,If CH103DiskReday is not successful, then the operation will be forced to start from the beginning.
     */
    while(1);
}
#endif

/* ***************************************************************************
 * @fn open_file
 *
 * @brief Try opening or enumerating files, and if the USB drive is not formatted, format the USB drive first.
 * If the file is not found, create a new file in the root directory
 *
 * @param filename - filename
 *
 * @return none */
void open_file( char *filename )
{
#if FUN_FILE_CREATE
  uint8_t s;

  printf( "Open\r\n" );
  strcpy( mCmdParam.Open.mPathName,filename );
  s = CHRV3FileOpen( );                       // Open file
  printf("open file. s=%x\n",s);
  if( s == 0x91 )
  {
      udisk_format( );
      s = ERR_MISS_FILE;
  }
  if ( s == ERR_MISS_DIR || s == ERR_MISS_FILE )// File not found
  {
      printf( "Create new file\n" );
      strcpy( mCmdParam.Create.mPathName, filename );// New file name, in the root directory, Chinese file name
      s = CHRV3FileCreate( );          // Create a new file and open it. If the file already exists, delete it before creating a new one
      mStopIfError( s );
  }
  else
  {
      udisk_enable();
      while(1);
  }
#endif
  data_len = 0;
  data_tollen = 0;
}

/* ***************************************************************************
 * @fn write_file
 *
 * @brief Write data to file
 *
 * @param pData - File Buffer
 *
 * @return none */
void write_file( uint8_t *pData )
{
#if FUN_FILE_CREATE
  uint8_t s;

  mCmdParam.Write.mSectorCount = 1;       // Write data to all sectors
  mCmdParam.Write.mDataBuffer = pData;    // Pointing to the starting address of the file data buffer
  s = CHRV3FileWrite( );                  // Write data to file
  mStopIfError( s );
#endif
}

/*********************************************************************
 * @fn      close_file
 *
 * @brief   Query the current file information, add file time and other information. Finally, close the current file
 *
 * @param   none
 *
 * @return  none
 */
void close_file( void )
{
  printf( "Close\r\n" );
#if FUN_FILE_CREATE
  uint8_t s;
  uint32_t file_len;

  if( data_len )
  {
      write_file( data_buf );
      s = CHRV3FileQuery( );                         // Query the information of the current file
      mStopIfError( s );
      file_len = CHRV3vFileSize - CHRV3vSectorSize + data_len;
      printf( "Modify\r\n" );
      mCmdParam.Modify.mFileAttr = ATTR_READ_ONLY;   // Input parameter: New file attribute, not modified if it is 0FFH
      mCmdParam.Modify.mFileTime = MAKE_FILE_TIME( 8,8,8 );   // Input parameter: New file time. If it is 0FFFFH, it will not be modified and the system's default time will be used
      mCmdParam.Modify.mFileDate = MAKE_FILE_DATE( 2023, 8, 1 );  // Input parameter: New file date: August 1st, 2023
      mCmdParam.Modify.mFileSize = file_len;         // Input parameter: The new file length, written in bytes, should be automatically updated by the program library
                                                     // when the file is closed, so it is not modified here
      s = CHRV3FileModify( );                        // Modify the information of the current file, modification date
      mStopIfError( s );
  }
  mCmdParam.Close.mUpdateLen = 0;
  s = CHRV3FileClose( );                             // close current file
  mStopIfError( s );
  udisk_enable( );
#endif
}

/* ***************************************************************************
 * @fn pdf_data_processes
 *
 * @brief This function is used to process PDF data. The function takes a pointer to the data buffer and the length of the data as parameters
 *
 * @param buf - PDF file data buffer
 * length - data length
 *
 * @return 0 - Write successfully
 * 1 - The data length is greater than PDF_TMP_BUF_LEN_EXT
 * 2 - data_tollen and length are added to PDF_FILE_MAX_LEN
 * */
uint32_t pdf_data_proces( void *buf, uint32_t length )
{
  if( length > PDF_TMP_BUF_LEN_EXT )
  {
    printf("max length error.len= %d > %d\n",length,PDF_TMP_BUF_LEN_MAX);

    return 1;
  }
  if( data_tollen + length > PDF_FILE_MAX_LEN )
  {
    printf( "file error len=%d\r\n",data_tollen + length );

    return 2;
  }
  pdf_memcpy( &data_buf[data_len], buf, length );
  data_len += length;
  if( data_len >= PDF_TMP_BUF_LEN_MAX )
  {
    data_len -= PDF_TMP_BUF_LEN_MAX;
    write_file( data_buf );
    if( data_len )
    {
        pdf_memcpy( data_buf,&data_buf[PDF_TMP_BUF_LEN_MAX],data_len);
    }
  }
  data_tollen += length;

  return 0;
}
