/********************************** (C) COPYRIGHT *******************************
* File Name          : UDisk_Func_CreatDir.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : USB full-speed port host operation functions.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*******************************************************************************/
/* Header File */
#include "Udisk_Operation.h"

/* *********************************************************************************************
* Function Name: CreateDirectory
* Description: Create a new directory and open it. If the directory already exists, open it directly. The directory name is in mCmdParam.Create.mPathName, which is the same as the file name rules.
* Input:
* Output: None
* Return : ERR_SUCCESS = The directory is opened successfully or the directory is created successfully,
                   ERR_FOUND_NAME = The file with the same name already exists,
                   ERR_MISS_DIR = The path name is invalid or the previous directory does not exist
********************************************************************************************* */
uint8_t CreateDirectory( void )
{
    uint8_t   i, j;
    uint32_t  UpDirCluster;
    uint8_t * DirXramBuf;
    uint8_t  *DirConstData;
    j = 0xFF;
    for ( i = 0; i != sizeof( mCmdParam.Create.mPathName ); i ++ )    // Check directory path
    {
        if ( mCmdParam.Create.mPathName[ i ] == 0 )
        {
            break;
        }
        if ( mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR1 || mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR2 )
        {
            j = i;                                                     // Record the previous directory
        }
    }
    i = ERR_SUCCESS;
    if ( j == 0 || (j == 2 && mCmdParam.Create.mPathName[1] == ':') )
    {
        UpDirCluster = 0;                                              // Create a subdirectory in the root directory
    }
    else
    {
        if ( j != 0xFF )                                               // For the absolute path, the starting cluster number of the upper directory should be obtained.
        {
            mCmdParam.Create.mPathName[ j ] = 0;
            i = CHRV3FileOpen( );                                      // Open the previous directory
            if ( i == ERR_SUCCESS )
            {
                i = ERR_MISS_DIR;                                      // It's a file, not a directory
            }
            else if ( i == ERR_OPEN_DIR )
            {
                i = ERR_SUCCESS;                                       // Successfully opened the previous directory
            }
            mCmdParam.Create.mPathName[ j ] = PATH_SEPAR_CHAR1;        // Restore directory separator
        }
        UpDirCluster = CHRV3vStartCluster;                             // Save the starting cluster number of the previous directory
    }
    if ( i == ERR_SUCCESS )                                            // Successfully obtain the starting cluster number of the previous directory
    {
        i = CHRV3FileOpen( );                                          // Open the subdirectory of this level
        if ( i == ERR_SUCCESS )
        {
            i = ERR_FOUND_NAME;                                        // It's a file, not a directory
        }
        else if ( i == ERR_OPEN_DIR )
        {
            i = ERR_SUCCESS;                                           // The directory already exists
        }
        else if ( i == ERR_MISS_FILE )                                 // The directory does not exist, can be created
        {
            i = CHRV3FileCreate( );                                    // Create a directory by creating a file
            if ( i == ERR_SUCCESS )
            {
                if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                {
                    memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);     // If FILE_DATA_BUF is used in conjunction with DISK_BASE_BUF, the disk buffer must be cleared
                }
                DirXramBuf = pDISK_FAT_BUF;                            // File data buffer
                DirConstData = ".          \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0..         \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0";
                for ( i = 0x40; i != 0; i -- )                         // The directory retention unit points to itself and the previous directory respectively
                {
                    *DirXramBuf = *DirConstData;
                    DirXramBuf ++;
                    DirConstData ++;
                }
                *(pDISK_FAT_BUF+0x1A) = ( (uint8_t *)&CHRV3vStartCluster )[3];// Its own starting cluster number
                *(pDISK_FAT_BUF+0x1B) = ( (uint8_t *)&CHRV3vStartCluster )[2];
                *(pDISK_FAT_BUF+0x14) = ( (uint8_t *)&CHRV3vStartCluster )[1];
                *(pDISK_FAT_BUF+0x15) = ( (uint8_t *)&CHRV3vStartCluster )[0];
                *(pDISK_FAT_BUF+0x20+0x1A) = ( (uint8_t *)&UpDirCluster )[3];// The starting cluster number of the upper directory
                *(pDISK_FAT_BUF+0x20+0x1B) = ( (uint8_t *)&UpDirCluster )[2];
                *(pDISK_FAT_BUF+0x20+0x14) = ( (uint8_t *)&UpDirCluster )[1];
                *(pDISK_FAT_BUF+0x20+0x15) = ( (uint8_t *)&UpDirCluster )[0];
// for ( count = 0x40; count != CHRV3vSectorSizeH*256; count ++ ) { /* Clear the rest of the directory area */
//                  *DirXramBuf = 0;
//                  DirXramBuf ++;
//              }
                mCmdParam.Write.mSectorCount = 1;
                mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;                // Point to the start address of the file data buffer
                i = CHRV3FileWrite( );                                      // Write data to a file
                if ( i == ERR_SUCCESS )
                {
                    DirXramBuf = pDISK_FAT_BUF;
                    for ( i = 0x40; i != 0; i -- )                          // Clear the directory area
                    {
                        *DirXramBuf = 0;
                        DirXramBuf ++;
                    }
                    for ( j = 1; j != CHRV3vSecPerClus; j ++ )
                    {
                        if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                        {
                            memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);   // If FILE_DATA_BUF is used in conjunction with DISK_BASE_BUF, the disk buffer must be cleared
                        }
                        mCmdParam.Write.mSectorCount = 1;
                        mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;         // Point to the start address of the file data buffer
                        i = CHRV3FileWrite( );                               // Clear the remaining sectors of the directory
                        if ( i != ERR_SUCCESS )
                        {
                            break;
                        }
                    }
                    if ( j == CHRV3vSecPerClus )                              // Clear the directory successfully
                    {
                        mCmdParam.Modify.mFileSize = 0;                       // The length of the directory is always 0
                        mCmdParam.Modify.mFileDate = 0xFFFF;
                        mCmdParam.Modify.mFileTime = 0xFFFF;
                        mCmdParam.Modify.mFileAttr = 0x10;                    // Set directory properties
                        i = CHRV3FileModify( );                               // Modify file information into a directory
                    }
                }
            }
        }
    }
    return( i );
}

/*********************************************************************
 * @fn      UDisk_USBH_CreatDirectory
 *
 * @brief   Demo Function For UDisk Create Directory (EXAM9)
 *
 * @return  none
 */
void UDisk_USBH_CreatDirectory( void )
{
    uint8_t  i;
    uint8_t  ret;

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        printf("CHRV3DiskStatus:%02x\r\n",CHRV3DiskStatus);
        printf( "Create Level 1 Directory /YEAR2004 \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004" );             // Directory name, this directory is built in the root directory
        ret = CreateDirectory( );                                      // Create or open a directory
        mStopIfError( ret );
        /* The directory is created or opened successfully. Create a new demonstration file in this subdirectory below. */
        printf( "Create New File /YEAR2004/DEMO2004.TXT \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/DEMO2004.TXT" );// file name
        ret = CHRV3FileCreate( );                                      // Create a new file and open it. If the file already exists, delete it first and then create it.
        mStopIfError( ret );
        printf( "Write some data to file DEMO2004.TXT \r\n" );
        i = sprintf( Com_Buffer, "演示文件\xd\xa" );
        mCmdParam.ByteWrite.mByteCount = i;                            // Specify the number of bytes written this time, the length of a single read and write cannot exceed MAX_BYTE_IO
        mCmdParam.ByteWrite.mByteBuffer = Com_Buffer;                  // Point to the buffer
        ret = CHRV3ByteWrite( );                                       // Write data to a file in units of bytes, the length of a single read and write cannot exceed MAX_BYTE_IO
        mStopIfError( ret );
        printf( "Close file DEMO2004.TXT \r\n" );
        mCmdParam.Close.mUpdateLen = 1;                                // Automatically calculate file length and write files in bytes. It is recommended that the program library close the file so that the file length can be automatically updated.
        ret = CHRV3FileClose( );
        mStopIfError( ret );
        /* The following creates a new secondary subdirectory, and the method is exactly the same as the previous primary subdirectory */
        printf( "Create Level 2 Directory /YEAR2004/MONTH05 \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/MONTH05" );    // Directory name, this directory is built in the YEAR2004 subdirectory, the YEAR2004 directory must exist in advance
        ret = CreateDirectory( );                                     // Create or open a directory
        mStopIfError( ret );
        printf( "Close\r\n" );
        mCmdParam.Close.mUpdateLen = 0;                               // For directories, no automatic update of file length is required
        ret = CHRV3FileClose( );                                      // Close the directory, the directory does not need to be closed, it is just to prevent the following errors
        mStopIfError( ret );
    }
}


