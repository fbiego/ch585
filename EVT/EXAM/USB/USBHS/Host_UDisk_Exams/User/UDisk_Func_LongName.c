/********************************** (C) COPYRIGHT *******************************
* File Name          : Udisk_Func_longname.c
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

/*******************************************************************************/
/* Variable Definition */
uint8_t    LongNameBuf[ LONG_NAME_BUF_LEN ];
/* Example of long file name (the size end of UNICODE encoding must be the same as the UNICODE_ENDIAN definition)
The following is the coded content in LongName:
Create a long file name and enter two parameters: 1. Use (unicode big endian), and end with two 0 at the end of the string; 2. ANSI encode short file name.TXT */
uint8_t LongName[ ] =
#if UNICODE_ENDIAN == 1
{
    0x5E, 0xFA, 0x7A, 0xCB, 0x95, 0x7F, 0x65, 0x87, 0x4E, 0xF6, 0x54, 0x0D, 0xFF, 0x0C, 0x8F, 0x93,
    0x51, 0x65, 0x4E, 0x24, 0x4E, 0x2A, 0x53, 0xC2, 0x65, 0x70, 0xFF, 0x1A, 0x00, 0x20, 0x00, 0x31,
    0x00, 0x2E, 0x91, 0xC7, 0x75, 0x28, 0x00, 0x28, 0x00, 0x75, 0x00, 0x6E, 0x00, 0x69, 0x00, 0x63,
    0x00, 0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x59, 0x27, 0x7A, 0xEF, 0x00, 0x29, 0xFF, 0x0C,
    0x5B, 0x57, 0x7B, 0x26, 0x4E, 0x32, 0x67, 0x2B, 0x5C, 0x3E, 0x75, 0x28, 0x4E, 0x24, 0x4E, 0x2A,
    0x00, 0x30, 0x88, 0x68, 0x79, 0x3A, 0x7E, 0xD3, 0x67, 0x5F, 0x00, 0x3B, 0x00, 0x32, 0x00, 0x2E,
    0x00, 0x41, 0x00, 0x4E, 0x00, 0x53, 0x00, 0x49, 0x7F, 0x16, 0x78, 0x01, 0x77, 0xED, 0x65, 0x87,
    0x4E, 0xF6, 0x54, 0x0D, 0x00, 0x2E, 0x00, 0x54, 0x00, 0x58, 0x00, 0x54
};
#else
{
    0xFA, 0x5E, 0xCB, 0x7A, 0x7F, 0x95, 0x87, 0x65, 0xF6, 0x4E, 0x0D, 0x54, 0x0C, 0xFF, 0x93, 0x8F,
    0x65, 0x51, 0x24, 0x4E, 0x2A, 0x4E, 0xC2, 0x53, 0x70, 0x65, 0x1A, 0xFF, 0x20, 0x00, 0x31, 0x00,
    0x2E, 0x00, 0xC7, 0x91, 0x28, 0x75, 0x28, 0x00, 0x75, 0x00, 0x6E, 0x00, 0x69, 0x00, 0x63, 0x00,
    0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x00, 0x27, 0x59, 0xEF, 0x7A, 0x29, 0x00, 0x0C, 0xFF,
    0x57, 0x5B, 0x26, 0x7B, 0x32, 0x4E, 0x2B, 0x67, 0x3E, 0x5C, 0x28, 0x75, 0x24, 0x4E, 0x2A, 0x4E,
    0x30, 0x00, 0x68, 0x88, 0x3A, 0x79, 0xD3, 0x7E, 0x5F, 0x67, 0x3B, 0x00, 0x32, 0x00, 0x2E, 0x00,
    0x41, 0x00, 0x4E, 0x00, 0x53, 0x00, 0x49, 0x00, 0x16, 0x7F, 0x01, 0x78, 0xED, 0x77, 0x87, 0x65,
    0xF6, 0x4E, 0x0D, 0x54, 0x2E, 0x00, 0x54, 0x00, 0x58, 0x00, 0x54, 0x00
};
#endif

/*********************************************************************
 * @fn      UDisk_USBH_Longname
 *
 * @brief   Demo Function For UDisk long-name Operation(EXAM 13)
 *
 * @return  none
 */
void UDisk_USBH_Longname( void )
{
    uint8_t  ret, i, len;
    uint16_t j;

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        /* ========================= The following demonstration creates and reads long file names ===================================== */
        // Copy the long file name (UNICODE big endian) into LongNameBuf
        len = LongName_Len;
        memcpy( LongNameBuf, LongName, len );
        // The end is represented by two 0s
        LongNameBuf[len] = 0x00;
        LongNameBuf[len + 1] = 0x00;
        // The ANSI encoded short file name of the long file name (8+3 format)
        strcpy( mCmdParam.Create.mPathName, "\\长文件名.TXT" );
        i = CHRV3CreateLongName( );
        if( i == ERR_SUCCESS )
        {
            PRINT( "Created Long Name OK!\r\n" );
        }
        else
        {
            /* Error code defined in "udisk_operaion.h" */
            PRINT( "Error Code: %02X\r\n", (uint16_t)i );
        }

        PRINT( "Get long Name#\r\n" );
        strcpy( mCmdParam.Open.mPathName, "\\长文件名.TXT" );
        // The complete path to the file name is required above
        i = CHRV3GetLongName( );
        if( i == ERR_SUCCESS )
        {
            // The long file name is collected and encoded in UNICODE (defined according to UNICODE_ENDIAN)
            // Store it in LongNameBuf buffer, and the long file name ends with two 0s.
            // The following shows all data in the buffer
            PRINT( "LongNameBuf: " );
            for( j=0; j!=LONG_NAME_BUF_LEN; j++ )
            {
                PRINT( "%02X ", (uint16_t)LongNameBuf[j] );
            }
            PRINT( "\r\n" );
        }
        else
        {
            /* Error code defined in "udisk_operaion.h" */
            PRINT( "Error Code: %02X\r\n", (uint16_t)i );
        }
    }
}

/* ***************************************************************************
 * @fn CheckNameSum
 *
 * @brief Check the short file name check sum of long file names
 *
 * @return Calculated checksum */
uint8_t CheckNameSum( uint8_t *p )
{
uint8_t FcbNameLen;
uint8_t Sum;

    Sum = 0;
    for (FcbNameLen=0; FcbNameLen!=11; FcbNameLen++)
        Sum = ((Sum & 1) ? 0x80 : 0) + (Sum >> 1) + *p++;
    return Sum;
}

/* ***************************************************************************
 * @fn AnalyzeLongName
 *
 * @brief sorting out long file names Returns how many 26 lengths there are
 *
 * @return Return how many 26 lengths there are */
uint8_t AnalyzeLongName( void )
{
uint8_t   i, j;
uint16_t  index;

    i = FALSE;
    for( index=0; index!=LONG_NAME_BUF_LEN; index = index + 2 )
    {
        if( ( LongNameBuf[index] == 0 ) && ( LongNameBuf[index+1] == 0 ) )
        {
            i = TRUE;
            break;
        }
    }
    if( ( i == FALSE ) || ( index == 0) )
        return 0;                   // Returns 0 for error long file name

    i = index % 26;
    if( i != 0 )
    {
        index += 2;
        if( index % 26 != 0 )       // Adding 0 just ends
        {
            for( j=i+2; j!=26; j++ )// Fill in the remaining data as 0XFF
                LongNameBuf[index++] = 0xff;
        }
    }
    return  (index / 26);
}

/* ***************************************************************************
 * @fn CHRV3CreateLongName
 *
 * @brief To create a long file name, you need to enter the complete path of the short file name
 *
 * @return Operation status */
uint8_t CHRV3CreateLongName( void )
{
// Analysis Keep file path Create an empty file Get FDT offset and sector Delete file
// Offset sector backwards may fail. If FAT12/16 is at the root directory, create the file again after filling.
uint8_t   i;
uint8_t   len;                                // The length of the storage path
uint16_t  index;                              // Long file offset index
uint16_t  indexBak;                           // Long file offset index backup
uint32_t  Secoffset;                          // Sector Offset

uint8_t   Fbit;                               // Entering the writing sector for the first time
uint8_t   Mult;                               // Long file name length multiples 26
uint8_t   MultBak;                            // Backup of multiples of long file name length 26

uint16_t  Backoffset;                         // Save file offset backup
uint16_t  BackoffsetBak;                      // Save a backup of offset backup
uint32_t  BackFdtSector;                      // Pre-size offset from the previous sector
uint8_t   sum;                                // Save the checksum of long file names

uint8_t   BackPathBuf[MAX_PATH_LEN];    // Save file path

    Mult = AnalyzeLongName( );              // Save long file name multiples of 26
    if( Mult == 0 )
        return ERR_LONG_NAME;
    MultBak = Mult;

    i = CHRV3FileOpen();                    // If a short file name exists, an error will be returned
    if( i == ERR_SUCCESS )
        return ERR_NAME_EXIST;

    i = CHRV3FileCreate( );
    if( i == ERR_SUCCESS )
    {
        Backoffset = CHRV3vFdtOffset;
        BackoffsetBak = Backoffset;
        BackFdtSector = CHRV3vFdtLba;
        sum = CheckNameSum( &DISK_BASE_BUF[Backoffset ] );
        for( i=0; i!=MAX_PATH_LEN; i++ )    // Backup file paths
            BackPathBuf[i] = mCmdParam.Open.mPathName[i];
        CHRV3FileErase( );                  // Delete this file

        Secoffset   = 0;                    // Offset starting from 0
        index       = Mult*26;              // Get the length of the long file name
        indexBak    = index;
        Fbit        = FALSE;                // No entry by default
        // Open the previous level to fill data
        P_RETRY:
        for(len=0; len!=MAX_PATH_LEN; len++)
        {
            if(mCmdParam.Open.mPathName[len] == 0)
                break;                      // Get the string length
        }

        for(i=len-1; i!=0xff; i--)          // Get the location of the previous directory
        {
            if((mCmdParam.Open.mPathName[i] == '\\') || (mCmdParam.Open.mPathName[i] == '/'))
                break;
        }
        mCmdParam.Open.mPathName[i] = 0x00;

        if( i==0 )                          // Note: Special situations at the beginning of the root directory
        {
            mCmdParam.Open.mPathName[0] = '/';
            mCmdParam.Open.mPathName[1] = 0;
        }

        i = CHRV3FileOpen();                // Open the previous directory
        if( i == ERR_OPEN_DIR )
        {
            while( 1 )                      // Fill in until completed
            {
                mCmdParam.Locate.mSectorOffset = Secoffset;
                i = CHRV3FileLocate( );
                if( i == ERR_SUCCESS )
                {
                    if( Fbit )             // Enter the second write sector
                    {
                        if( mCmdParam.Locate.mSectorOffset != 0x0FFFFFFFF )
                        {
                            BackFdtSector = mCmdParam.Locate.mSectorOffset;
                            Backoffset = 0;
                        }
                        else
                        {
                            for( i=0; i!=MAX_PATH_LEN; i++ )// Restore file path
                                mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                            i = CHRV3FileCreate( );         // Do space expansion
                            if( i != ERR_SUCCESS )
                                return i;
                            CHRV3FileErase( );
                            goto P_RETRY;                   // Reopen the previous directory
                        }
                    }

                    if( BackFdtSector == mCmdParam.Locate.mSectorOffset )
                    {
                        mCmdParam.Read.mSectorCount = 1;   // Read a sector to disk buffer
                        mCmdParam.Read.mDataBuffer = &DISK_BASE_BUF[0];
                        i = CHRV3FileRead( );
                        CHRV3DirtyBuffer( );                // Clear the disk buffer
                        if( i!= ERR_SUCCESS )
                            return i;

                        i = ( CHRV3vSectorSize - Backoffset ) / 32;
                        if( Mult > i )
                            Mult = Mult - i;                // The remaining multiples
                        else
                        {
                            i = Mult;
                            Mult = 0;
                        }

                        for( len=i; len!=0; len-- )
                        {
                            indexBak -= 26;
                            index = indexBak;
                            for( i=0; i!=5; i++)            // 1-5 characters of a long file name
                            {                               // UNICODE is stored in small-endian way on disk
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + i*2 + 2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + i*2 + 2 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            for( i =0; i!=6; i++)           // 6-11 characters of long file name
                            {
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + 14 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 14 + i*2 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + 14 + i*2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 14 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            for( i=0; i!=2; i++)            // 12-13 characters of long file name
                            {
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + 28 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 28 + i*2 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + 28 + i*2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 28 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            DISK_BASE_BUF[Backoffset + 0x0b] = 0x0f;
                            DISK_BASE_BUF[Backoffset + 0x0c] = 0x00;
                            DISK_BASE_BUF[Backoffset + 0x0d] = sum;
                            DISK_BASE_BUF[Backoffset + 0x1a] = 0x00;
                            DISK_BASE_BUF[Backoffset + 0x1b] = 0x00;
                            DISK_BASE_BUF[Backoffset] = MultBak--;
                            Backoffset += 32;
                        }

                        if( !Fbit )
                        {
                            Fbit = TRUE;
                            DISK_BASE_BUF[ BackoffsetBak ] |= 0x40;
                        }
                        CHRV3vLbaCurrent = BackFdtSector;
                        i = CHRV3WriteSector( 1, DISK_BASE_BUF );
                        if( i!= ERR_SUCCESS )
                            return i;

                        if( Mult==0 )
                        {   // Restore file path
					        CHRV3FileClose( );
                            for( i=0; i!=MAX_PATH_LEN; i++ )
                                mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                            i = CHRV3FileCreate( );
                            return i;
                        }
                    }
                }
                else
                    return i;
                Secoffset++;
            }
        }
    }
    return i;
}

/* ***************************************************************************
 * @fn GetUpSectorData
 *
 * @brief Get the data of the previous sector from the current sector and place it in the disk buffer
 *
 * @return Operation status */
uint8_t GetUpSectorData( uint32_t *NowSector )
{
uint8_t  i;
uint8_t  len;             // The length of the storage path
uint32_t index;           // Number of sector offset sectors

    index = 0;
    for(len=0; len!=MAX_PATH_LEN; len++)
    {
        if(mCmdParam.Open.mPathName[len] == 0)          // Get the string length
            break;
    }

    for(i=len-1; i!=0xff; i--)                          // Get the location of the previous directory
    {
        if((mCmdParam.Open.mPathName[i] == '\\') || (mCmdParam.Open.mPathName[i] == '/'))
            break;
    }
    mCmdParam.Open.mPathName[i] = 0x00;

    if( i==0 )  // Note: Special situations at the beginning of the root directory
    {
        mCmdParam.Open.mPathName[0] = '/';
        mCmdParam.Open.mPathName[1] = 0;
        i = CHRV3FileOpen();
        if ( i == ERR_OPEN_DIR )
            goto P_NEXT0;
    }
    else
    {
        i = CHRV3FileOpen();
        if ( i == ERR_OPEN_DIR )
        {
            while( 1 )
            {
                P_NEXT0:
                mCmdParam.Locate.mSectorOffset = index;
                i = CHRV3FileLocate( );
                if( i == ERR_SUCCESS )
                {
                    if( *NowSector == mCmdParam.Locate.mSectorOffset )
                    {
                        if( index==0 )                          // At the beginning of the root sector
                            return ERR_NO_NAME;
                        mCmdParam.Locate.mSectorOffset = --index;
                        i = CHRV3FileLocate( );                 // Read data from the previous sector
                        if( i == ERR_SUCCESS )
                        {                                       // The following saves the current number of sectors
                            *NowSector = mCmdParam.Locate.mSectorOffset;
                            mCmdParam.Read.mSectorCount = 1;   // Read a sector to disk buffer
                            mCmdParam.Read.mDataBuffer = &DISK_BASE_BUF[0];
                            i = CHRV3FileRead( );
                            CHRV3DirtyBuffer( );                // Clear the disk buffer
                            return i;
                        }
                        else
                            return i;
                    }
                }
                else
                    return i;
                index++;
            }
        }
    }
    return i;
}

/* ***************************************************************************
 * @fn CHRV3GetLongName
 *
 * @brief Get the corresponding long file name from the complete short file name path (can be a file or a folder)
 *
 * @return Operation status */
uint8_t CHRV3GetLongName( void )
{
// Variable sector size required
// Step 1: Open the file to find the file, analyze whether the file exists, and get the offset of the FDT sector and the sector in which it is located
// Step 2: Analyze the above information to see if there is a long file name and whether it is at the beginning of the first sector of the directory.
// Step 3: Implement a sector offset backwards? Read the long file name (U disk with 512-byte sector)
uint8_t   i;
uint16_t  index;          // Index in long filename buffer
uint32_t  BackFdtSector;  // Pre-size offset from the previous sector
uint8_t   sum;            // Save the checksum of long file names
// uint16_t Backoffset; // Save file offset backup
uint16_t  offset;         // File offset in sector 32 times
uint8_t   FirstBit;       // Long file name spans two sector flag bits
uint8_t   BackPathBuf[MAX_PATH_LEN]; // Save file path

    i = CHRV3FileOpen( );
    if( ( i == ERR_SUCCESS ) || ( i == ERR_OPEN_DIR ) )
    {
        for( i=0; i!=MAX_PATH_LEN; i++ )
            BackPathBuf[i] = mCmdParam.Open.mPathName[i];
        // The above completes the backup of the path

        sum = CheckNameSum( &DISK_BASE_BUF[CHRV3vFdtOffset] );
        index = 0;
        FirstBit = FALSE;
//        Backoffset = CHRV3vFdtOffset;
        BackFdtSector = CHRV3vFdtLba;
        if( CHRV3vFdtOffset == 0 )
        {
            // First determine whether it is at the beginning of a sector or whether it is at the beginning of the root directory, otherwise it will be offset backwards.
            if( FirstBit == FALSE )
                FirstBit = TRUE;
            i = GetUpSectorData( &BackFdtSector );
            if( i == ERR_SUCCESS )
            {
                CHRV3vFdtOffset = CHRV3vSectorSize;
                goto P_NEXT1;
            }
        }
        else
        {
            // Read the offset data until it ends. If not enough, shift backward
            P_NEXT1:
            offset = CHRV3vFdtOffset;
            while( 1 )
            {
                if( offset != 0 )
                {
                    offset = offset - 32;
                    if( ( DISK_BASE_BUF[offset + 11] == ATTR_LONG_NAME )
                        && (  DISK_BASE_BUF[offset + 13] == sum ) )
                    {
                        if( (index + 26) > LONG_NAME_BUF_LEN )
                            return ERR_BUF_OVER;

                        for( i=0; i!=5; i++)            // 1-5 characters of a long file name
                        {                               // UNICODE is stored in small-endian way on disk
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 2];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 1];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 2];
                            #endif
                        }

                        for( i =0; i!=6; i++)           // 6-11 characters of long file name
                        {
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 14 + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + + 14 + i*2 ];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + + 14 + i*2 ];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 14 + i*2 + 1];
                            #endif

                        }

                        for( i=0; i!=2; i++)            // 12-13 characters of long file name
                        {
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 ];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 ];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 + 1];
                            #endif
                        }

                        if( DISK_BASE_BUF[offset] & 0X40 )
                        {
                            if( ! (((LongNameBuf[index -1] ==0x00)
                                && (LongNameBuf[index -2] ==0x00))
                                || ((LongNameBuf[index -1] ==0xFF)
                                && (LongNameBuf[index -2 ] ==0xFF))))
                            {                           // Process file names that are exactly 26 bytes long
                                if(index + 52 >LONG_NAME_BUF_LEN )
                                    return ERR_BUF_OVER;
                                LongNameBuf[ index ] = 0x00;
                                LongNameBuf[ index + 1] = 0x00;
                            }
                            return ERR_SUCCESS;         // Successfully completed the collection of long file names
                        }
                    }
                    else
                        return ERR_NO_NAME;             // The wrong long file name, the program returns
                }
                else
                {
                    if( FirstBit == FALSE )
                        FirstBit = TRUE;
                    else                                // Otherwise, the second entry
                    {
                        for( i=0; i!=MAX_PATH_LEN; i++ )// Restore the path
                            mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                    }
                    i = GetUpSectorData( &BackFdtSector );
                    if( i == ERR_SUCCESS )
                    {
                        CHRV3vFdtOffset = CHRV3vSectorSize;
                        goto P_NEXT1;
                    }
                    else
                        return i;
                    // Offset sector backwards
                }
            }
        }
    }
    return i;                // Return an error
}

