/* ********************************* (C) COPYRIGHT ***************************
 * File Name: EXAM1.C
 * Author: WCH
 * Version: V1.0
 * Date: 2020/08/11
 * Description: C language USB drive creation long file name routine
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

typedef struct __attribute__((packed)) _LONG_NAME
{                               // Byte Alignment
    uint8_t  LDIR_Ord;          /* The group number of the long file name, if it is 0X40, it means the last group */
    uint16_t LDIR_Name1[5];     /* The first 5 bytes of a long file name */
    uint8_t  LDIR_Attr;         /* The attribute must be ATTR_LONG_NAME */
    uint8_t  LDIR_Type;         /* A child with a long file name of 0 */
    uint8_t  LDIR_Chksum;       /* Checksum of short file names */
    uint16_t LDIR_Name2[6];     /* 6-11 characters of long file name */
    uint8_t  LDIR_FstClusLO[2]; /* ä¸º0 */
    uint16_t LDIR_Name3[2];     /* Long file names 12-13 are each. character */

} F_LONG_NAME; /* Define long file names */

typedef F_LONG_NAME *P_LONG_NAME;

#define MAX_LONG_NAME        4
#define FILE_LONG_NAME       MAX_LONG_NAME * 13 + 1
#define DATA_BASE_BUF_LEN    512

uint8_t DATA_BASE_BUF0[DATA_BASE_BUF_LEN];
uint8_t DATA_BASE_BUF1[DATA_BASE_BUF_LEN];

uint16_t LongFileName[FILE_LONG_NAME]; /* Long file namespace only stores file names but not paths */

/* ***************************************************************************
 * @fn ChkSum
 *
 * @brief calculates the checksum of short file names
 *
 * @param pDir1 - File directory information in the FAT data area
 *
 * @return Checksum */
unsigned char ChkSum(PX_FAT_DIR_INFO pDir1)
{
    unsigned char FcbNameLen;
    unsigned char Sum;
    Sum = 0;
    for(FcbNameLen = 0; FcbNameLen != 11; FcbNameLen++)
    {
        //if(pDir1->DIR_Name[FcbNameLen]==0x20)continue;
        Sum = ((Sum & 1) ? 0x80 : 0) + (Sum >> 1) + pDir1->DIR_Name[FcbNameLen];
    }
    return (Sum);
}

/* ***************************************************************************
 * @fn mLDirCheck
 *
 * @brief analyzes whether the directory entry and long file names of the buffer are the same
 *
 * @param pDir2 - File directory information in the FAT data area
 * @param pLdir1 - long file name
 *
 * @return Return 00-15 is to find a file with the same long file name. 00-15 indicates the corresponding short file name in the directory entry.
 * Return 0X80-8F to indicate that the analysis reaches the end of the directory item. After that, all directory items are unused. Return 0FF to indicate that there is no matching directory item in this sector. */
uint8_t mLDirCheck(PX_FAT_DIR_INFO pDir2, F_LONG_NAME *pLdir1)
{
    uint8_t      i, j, k, sum, nodir, nodir1;
    F_LONG_NAME *pLdir2;
    uint16_t    *pLName;
    for(i = 0; i != 16; i++)
    {
        if(pDir2->DIR_Name[0] == 0xe5)
        {
            pDir2 += 1;
            continue;
        } /* If this item is deleted, continue to analyze the next directory */ /* If the deleted file name is skipped */
        if(pDir2->DIR_Name[0] == 0x00)
        {
            return i | 0x80;
        } /* Analysis: No file name exists in the following space. */
        if((pDir2->DIR_Attr == 0x0f) | (pDir2->DIR_Attr == 0x8))
        {
            pDir2 += 1;
            continue;
        } /* If the volume label or long file name is found, continue */
        /* Find a short file name */
        k = i - 1; /* The long file name should be on the short file name */
        if(i == 0)
        {                    /* If this short file name is in the first item in this sector */
            pLdir2 = pLdir1; /* The long file name should be in the last item in the previous sector */
            k = 15;          /* Record the location of the long file name */
            pLdir2 += 15;    /* Offset to the end */
        }
        else
            pLdir2 = (F_LONG_NAME *)(pDir2 - 1); /* Take the long file name directory item */
        sum = ChkSum(pDir2);                     /* Calculate the accumulated sum */
        pLName = LongFileName;                   /* A long file name specified by the item */
        nodir = 0;                               /* Initialize flags */
        nodir1 = 1;
        while(1)
        {
            if((pLdir2->LDIR_Ord != 0xe5) & (pLdir2->LDIR_Attr == ATTR_LONG_NAME) & (pLdir2->LDIR_Chksum == sum))
            { /* Find a long file name */
                for(j = 0; j != 5; j++)
                {
                    if((pLdir2->LDIR_Name1[j] == 0x00) | (*pLName == 0))
                        continue; /* Analyze to the end of long filename */
                    if((pLdir2->LDIR_Name1[j] == 0xff) | (*pLName == 0))
                        continue; /* Analyze to the end of long filename */
                    if(pLdir2->LDIR_Name1[j] != *pLName)
                    { /* Set flags if not */
                        nodir = 1;
                        break;
                    }
                    pLName++;
                }
                if(nodir == 1)
                    break; /* Exit with different file names */
                for(j = 0; j != 6; j++)
                {
                    if((pLdir2->LDIR_Name2[j] == 0x00) | (*pLName == 0))
                        continue;
                    if((pLdir2->LDIR_Name2[j] == 0xff) | (*pLName == 0))
                        continue;
                    if(*pLName != pLdir2->LDIR_Name2[j])
                    {
                        nodir = 1;
                        break;
                    }
                    pLName++;
                }
                if(nodir == 1)
                    break; /* Exit with different file names */
                for(j = 0; j != 2; j++)
                {
                    if((pLdir2->LDIR_Name3[j] == 0x00) | (*pLName == 0))
                        continue;
                    if((pLdir2->LDIR_Name3[j] == 0xff) | (*pLName == 0))
                        continue;
                    if(*pLName != pLdir2->LDIR_Name3[j])
                    {
                        nodir = 1;
                        break;
                    }
                    pLName++;
                }
                if(nodir == 1)
                    break; /* Exit with different file names */
                if((pLdir2->LDIR_Ord & 0x40) == 0x40)
                {
                    nodir1 = 0;
                    break;
                } /* Find the long file name and compare it to the end */
            }
            else
                break; /* Exit without a continuous long file name */
            if(k == 0)
            {
                pLdir2 = pLdir1;
                pLdir2 += 15;
                k = 15;
            }
            else
            {
                k = k - 1;
                pLdir2 -= 1;
            }
        }
        if(nodir1 == 0)
            return i; /* Indicates that the long file name is found, and returns the directory entry in which the short file name is located. */
        pDir2 += 1;
    }
    return 0xff; /* Refers to the long file name that did not find the response after searching for this sector. */
}

/* ***************************************************************************
 * @fn mChkName
 *
 * @brief Check the previous subdirectory and open it
 *
 * @param pJ - Return a set of data
 *
 * @return status */
uint8_t mChkName(unsigned char *pJ)
{
    uint8_t i, j;
    j = 0xFF;
    for(i = 0; i != sizeof(mCmdParam.Create.mPathName); i++)
    { /* Check directory path */
        if(mCmdParam.Create.mPathName[i] == 0)
            break;
        if(mCmdParam.Create.mPathName[i] == PATH_SEPAR_CHAR1 || mCmdParam.Create.mPathName[i] == PATH_SEPAR_CHAR2)
            j = i; /* Record the previous directory */
    }
    i = ERR_SUCCESS;
    if((j == 0) || ((j == 2) && (mCmdParam.Create.mPathName[1] == ':')))
    { /* Create in the root directory */
        mCmdParam.Open.mPathName[0] = '/';
        mCmdParam.Open.mPathName[1] = 0;
        i = CHRV3FileOpen(); /* Open the root directory */
        if(i == ERR_OPEN_DIR)
            i = ERR_SUCCESS; /* Successfully opened the previous directory */
    }
    else
    {
        if(j != 0xFF)
        { /* For the absolute path, the starting cluster number of the upper directory should be obtained. */
            mCmdParam.Create.mPathName[j] = 0;
            i = CHRV3FileOpen(); /* Open the previous directory */
            if(i == ERR_SUCCESS)
                i = ERR_MISS_DIR; /* It's a file, not a directory */
            else if(i == ERR_OPEN_DIR)
                i = ERR_SUCCESS;                              /* Successfully opened the previous directory */
            mCmdParam.Create.mPathName[j] = PATH_SEPAR_CHAR1; /* Restore directory separator */
        }
    }
    *pJ = j; /* Return a set of data in the pointer */
    return i;
}

/* ***************************************************************************
 * @fn CreatLongName
 *
 * @brief Create and open the long file name of the file, enter the path in the short file name space and reference the short file name, and enter the UNICODE code of the long file name of the file in the long file name space
 *
 * @return Return 00 means success, and returns the real short file name in the short file namespace, others are unsuccessful */
uint8_t CreatLongName()
{
    uint8_t         ParData[MAX_PATH_LEN]; /**/
    uint16_t        tempSec;               /* Sector Offset */
    uint8_t         i, j, k, x, sum, y, z;
    P_LONG_NAME     pLDirName;
    PX_FAT_DIR_INFO pDirName, pDirName1;
    BOOL            FBuf;
    uint8_t        *pBuf1;
    uint16_t       *pBuf;
    CHRV3DirtyBuffer();
    for(k = 0; k != MAX_PATH_LEN; k++)
        ParData[k] = mCmdParam.Other.mBuffer[k];
    i = mChkName(&j);
    if(i == ERR_SUCCESS)
    {             /* Successfully obtain the starting cluster number of the previous directory */
        FBuf = 0; /* initialization */
        tempSec = 0;
        DATA_BASE_BUF1[0] = 0xe5; /* Invalid last buffer */
        k = 0xff;
        while(1)
        {                                                                                        /* The following is to read and analyze the directory items */
            pDirName = FBuf ? (PX_FAT_DIR_INFO)DATA_BASE_BUF1 : (PX_FAT_DIR_INFO)DATA_BASE_BUF0; /* Short file name pointer to buffer */
            pLDirName = FBuf ? (P_LONG_NAME)DATA_BASE_BUF0 : (P_LONG_NAME)DATA_BASE_BUF1;
            mCmdParam.Read.mSectorCount = 1;                                     /* Read a sector data */
            mCmdParam.Read.mDataBuffer = FBuf ? DATA_BASE_BUF1 : DATA_BASE_BUF0; /* The file buffer currently processed, a two-way buffer is used here to process the file name. */
            FBuf = !FBuf;                                                        /* Buffer flag flip */
            i = CHRV3FileRead();
            if(mCmdParam.Read.mSectorCount == 0)
            {
                k = 0xff;
                break;
            }
            tempSec += 1;
            k = mLDirCheck(pDirName, pLDirName);
            z = k;
            z &= 0x0f;
            if(k != 0x0ff)
            {
                break;
            } /* Find the file or find the end of the file to exit */
        }
        if(k < 16)
        {
            pDirName += k; /* The file you are looking for is the short file name in this directory */
            if(j != 0xff)
            {
                for(k = 0; k != j + 1; k++)
                    mCmdParam.Other.mBuffer[k] = ParData[k];
            }
            pBuf1 = &mCmdParam.Other.mBuffer[j + 1]; /* Get the address of the file name */
            //else pBuf1=&mCmdParam.Other.mBuffer;
            for(i = 0; i != 8; i++)
            {
                if(pDirName->DIR_Name[i] == 0x20)
                    continue;
                else
                {
                    *pBuf1 = pDirName->DIR_Name[i];
                    pBuf1++;
                }
            }
            if(pDirName->DIR_Name[i] != 0x20)
            {
                *pBuf1 = '.';
                pBuf1++;
            }
            for(; i != 11; i++)
            {
                if(pDirName->DIR_Name[i] == 0x20)
                    continue;
                else
                {
                    *pBuf1 = pDirName->DIR_Name[i];
                    pBuf1++;
                }

            } /* Copy the short file name */
            i = CHRV3FileClose();
            i = CHRV3FileCreate(); /* I wonder if I should restore the cluster number when I first entered this function */
            PRINT("k<16\r\n");
            return i; /* Create a file and return to status */
        }
        else
        { /* Indicates that the directory entry enumeration is at the end and the file is to be created. */
            if(k == 0xff)
            {
                z = 00;
                tempSec += 1;
            }
            i = CHRV3FileClose();
            for(k = 0; k != MAX_PATH_LEN; k++)
                mCmdParam.Other.mBuffer[k] = ParData[k]; /* Try to create a short file name */
            for(x = 0x31; x != 0x3a; x++)
            { /* Generate short filenames */
                for(y = 0x31; y != 0x3a; y++)
                {
                    for(i = 0x31; i != 0x3a; i++)
                    {
                        mCmdParam.Other.mBuffer[j + 7] = i;
                        mCmdParam.Other.mBuffer[j + 6] = '~';
                        mCmdParam.Other.mBuffer[j + 5] = y;
                        mCmdParam.Other.mBuffer[j + 4] = x;
                        if(CHRV3FileOpen() != ERR_SUCCESS)
                            goto XAA1;
                        /**/
                    }
                }
            }
            i = 0xff;
            goto XBB;
        /* The naming cannot be performed correctly */
        XAA1:

            i = CHRV3FileCreate();
            if(i != ERR_SUCCESS)
                return i; // {goto XCC;} /*If an error occurs, you cannot continue*/
            for(k = 0; k != MAX_PATH_LEN; k++)
                ParData[k] = mCmdParam.Other.mBuffer[k]; /* Try to create a short file name */
            i = mChkName(&j);
            mCmdParam.Locate.mSectorOffset = tempSec - 1;
            i = CHRV3FileLocate();
            if(i != ERR_SUCCESS)
                return i; // {goto XCC;} /*If an error occurs, you cannot continue*/
            mCmdParam.Read.mSectorCount = 1;
            mCmdParam.Read.mDataBuffer = DATA_BASE_BUF0;
            pDirName = (PX_FAT_DIR_INFO)DATA_BASE_BUF0;
            pDirName += z;       /* Offset to create filename */
            i = CHRV3FileRead(); /* Read the data of a sector and take the first directory entry as the short file name you just created */
            if(i != ERR_SUCCESS)
                return i; // {goto XCC;} /*Error handling is required here*/

            for(i = 0; i != FILE_LONG_NAME; i++)
            {
                if(LongFileName[i] == 00)
                    break; /* Calculate the length of a long file name */
            }
            for(k = i + 1; k != FILE_LONG_NAME; k++)
            { /* Fill in invalid long directory */
                LongFileName[k] = 0xffff;
            }
            k = i / 13; /* Take the number of long file names and groups */
            i = i % 13;
            if(i != 0)
                k = k + 1; /* If there is a remainder, it will be calculated as a group */
            i = k;
            k = i + z; /* z is short file offset, z-1 is long file offset */
            if(k < 16)
            {
                pDirName1 = (PX_FAT_DIR_INFO)DATA_BASE_BUF0;
                pDirName1 += k;
                sum = ChkSum(pDirName1); /* Calculate the accumulated sum */
                pLDirName = (P_LONG_NAME)DATA_BASE_BUF0;
                pLDirName += (k - 1);
            }
            else if(k == 16)
            {
                pDirName1 = (PX_FAT_DIR_INFO)DATA_BASE_BUF1;
                pDirName1 += (k - 16);
                pLDirName = (F_LONG_NAME *)DATA_BASE_BUF0;
                pLDirName += (k - 1);
            }
            else if(k > 16)
            {
                pDirName1 = (PX_FAT_DIR_INFO)DATA_BASE_BUF1;
                pDirName1 += (k - 16);
                pLDirName = (F_LONG_NAME *)DATA_BASE_BUF1;
                pLDirName += (k - 1 - 16);
            }
            /* Copy the short file name and copy the short file name to the specified area */
            pDirName1->DIR_NTRes = pDirName->DIR_NTRes;
            pDirName1->DIR_CrtTimeTenth = pDirName->DIR_CrtTimeTenth;
            pDirName1->DIR_CrtTime = pDirName->DIR_CrtTime;
            pDirName1->DIR_CrtDate = pDirName->DIR_CrtDate;
            pDirName1->DIR_LstAccDate = pDirName->DIR_LstAccDate;
            pDirName1->DIR_FstClusHI = pDirName->DIR_FstClusHI;
            pDirName1->DIR_WrtTime = pDirName->DIR_WrtTime;
            pDirName1->DIR_WrtDate = pDirName->DIR_WrtDate;
            pDirName1->DIR_FstClusLO = pDirName->DIR_FstClusLO;
            pDirName1->DIR_FileSize = pDirName->DIR_FileSize;
            pDirName1->DIR_Attr = pDirName->DIR_Attr;

            pDirName1->DIR_Name[0] = pDirName->DIR_Name[0];
            pDirName1->DIR_Name[1] = pDirName->DIR_Name[1];
            pDirName1->DIR_Name[2] = pDirName->DIR_Name[2];
            pDirName1->DIR_Name[3] = pDirName->DIR_Name[3];
            pDirName1->DIR_Name[4] = pDirName->DIR_Name[4];
            pDirName1->DIR_Name[5] = pDirName->DIR_Name[5];
            pDirName1->DIR_Name[6] = pDirName->DIR_Name[6];
            pDirName1->DIR_Name[7] = pDirName->DIR_Name[7];
            pDirName1->DIR_Name[8] = pDirName->DIR_Name[8];
            pDirName1->DIR_Name[9] = pDirName->DIR_Name[9];
            pDirName1->DIR_Name[10] = pDirName->DIR_Name[10];
            sum = ChkSum(pDirName1); /* Calculate the accumulated sum */
            pBuf = LongFileName;     /* Point to long file namespace */
            y = 1;
            if(k > 16)
            {
                for(i = 1; i != k - 16 + 1; i++)
                {
                    pLDirName->LDIR_Ord = y;
                    pLDirName->LDIR_Name1[0] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name1[1] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name1[2] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name1[3] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name1[4] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Attr = 0x0f;
                    pLDirName->LDIR_Type = 0;
                    pLDirName->LDIR_Chksum = sum;
                    pLDirName->LDIR_Name2[0] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name2[1] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name2[2] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name2[3] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name2[4] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name2[5] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_FstClusLO[0] = 0;
                    pLDirName->LDIR_FstClusLO[1] = 0;
                    pLDirName->LDIR_Name3[0] = *pBuf;
                    pBuf++;
                    pLDirName->LDIR_Name3[1] = *pBuf;
                    pBuf++;
                    pLDirName--;
                    y += 1;
                }
                k = 16;
                i = 0;
                pLDirName = (F_LONG_NAME *)DATA_BASE_BUF0;
                pLDirName += (k - 1);
            }
            if(k > 16)
                k = 16;
            for(i = 1; i != k - z; i++)
            {
                pLDirName->LDIR_Ord = y;
                pLDirName->LDIR_Name1[0] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name1[1] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name1[2] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name1[3] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name1[4] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Attr = 0x0f;
                pLDirName->LDIR_Type = 0;
                pLDirName->LDIR_Chksum = sum;
                pLDirName->LDIR_Name2[0] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name2[1] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name2[2] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name2[3] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name2[4] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name2[5] = *pBuf;
                pBuf++;
                pLDirName->LDIR_FstClusLO[0] = 0;
                pLDirName->LDIR_FstClusLO[1] = 0;
                pLDirName->LDIR_Name3[0] = *pBuf;
                pBuf++;
                pLDirName->LDIR_Name3[1] = *pBuf;
                pBuf++;
                pLDirName--;
                y += 1;
            }
            pLDirName->LDIR_Ord = y | 0x40;
            pLDirName->LDIR_Name1[0] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name1[1] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name1[2] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name1[3] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name1[4] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Attr = 0x0f;
            pLDirName->LDIR_Type = 0;
            pLDirName->LDIR_Chksum = sum;
            pLDirName->LDIR_Name2[0] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name2[1] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name2[2] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name2[3] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name2[4] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name2[5] = *pBuf;
            pBuf++;
            pLDirName->LDIR_FstClusLO[0] = 0;
            pLDirName->LDIR_FstClusLO[1] = 0;
            pLDirName->LDIR_Name3[0] = *pBuf;
            pBuf++;
            pLDirName->LDIR_Name3[1] = *pBuf;
            pBuf++;
            pBuf = (uint16_t *)pDirName1;
            pBuf += 16;

            if(pBuf < (uint16_t *)(DATA_BASE_BUF0 + 0x200))
            {
                i = 2;
                while(1)
                {
                    *pBuf = 0;
                    pBuf++;
                    if(pBuf == (uint16_t *)(DATA_BASE_BUF0 + 0x200))
                        break;
                }
                i++;
            }
            else if(pBuf < (uint16_t *)(DATA_BASE_BUF1 + 0x200))
            {
                i = 1;
                while(1)
                {
                    *pBuf = 0;
                    pBuf++;
                    if(pBuf == (uint16_t *)(DATA_BASE_BUF1 + 0x200))
                        break;
                }
                i++;
            }
            mCmdParam.Locate.mSectorOffset = tempSec - 1;
            CHRV3DirtyBuffer();
            i = CHRV3FileLocate();
            if(i != ERR_SUCCESS)
                return i;                    /* If an error occurs, it cannot continue */
            mCmdParam.Read.mSectorCount = 1; /* Next */
            mCmdParam.Read.mDataBuffer = DATA_BASE_BUF0;
            CHRV3DirtyBuffer();
            i = CHRV3FileWrite(); /* Read the data of the next sector and take the first directory entry as the short file name you just created */
            CHRV3DirtyBuffer();
            if(i != ERR_SUCCESS)
                return i;                      /* Error handling is required here */
            pBuf = (uint16_t *)DATA_BASE_BUF1; /**/
            if(*pBuf != 0)
            {
                mCmdParam.Read.mSectorCount = 1;
                mCmdParam.Read.mDataBuffer = DATA_BASE_BUF1;
                i = CHRV3FileWrite();
                CHRV3DirtyBuffer();
            }
            /* If you are operating in the root directory, you should close the root directory */
            /* The file needs to be opened below */
            i = CHRV3FileClose();
            for(k = 0; k != MAX_PATH_LEN; k++)
                mCmdParam.Other.mBuffer[k] = ParData[k]; /* Try to create a short file name */
            i = CHRV3FileOpen();                         /* Open the created file */
            return i;
        }
    }
XBB:
{
    return i = 0xfe;
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
    uint8_t s, i;

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
    while(1)
    {
        s = ERR_SUCCESS;
        if(R8_USB_INT_FG & RB_UIF_DETECT) // If there is a USB host detection interrupt, it will be handled
        {
            R8_USB_INT_FG = RB_UIF_DETECT; // Clear connection interrupt flag
            s = AnalyzeRootHub();          // Analyze ROOT-HUB status
            if(s == ERR_USB_CONNECT)
                FoundNewDev = 1;
        }

        if(FoundNewDev || s == ERR_USB_CONNECT) // There is a new USB device plugged in
        {
            FoundNewDev = 0;
            mDelaymS(200);        // Since the USB device has just been plugged in, it is not stable yet, so wait for the USB device to be hundreds of milliseconds to eliminate plug-in and unplug jitter
            s = InitRootDevice(); // Initialize USB device
            if(s == ERR_SUCCESS)
            {
                // USB drive operation process: USB bus reset, USB drive connection, obtain device descriptors and set USB address, optional obtain configuration descriptors, and then arrive here, and the CHRV3 subroutine library continues to complete the subsequent work.
                CHRV3DiskStatus = DISK_USB_ADDR;
                for(i = 0; i != 10; i++)
                {
                    PRINT("Wait DiskReady\n");
                    s = CHRV3DiskReady(); // Wait for the USB drive to be ready
                    if(s == ERR_SUCCESS)
                    {
                        break;
                    }
                    else
                    {
                        PRINT("%02x\n", (uint16_t)s);
                    }
                    mDelaymS(50);
                }

                if(CHRV3DiskStatus >= DISK_MOUNTED)
                {
                    // Create a long file name file demonstration
                    PRINT("Create Long Name\n");
                    strcpy((uint8_t *)mCmdParam.Create.mPathName, "/TCBD~1.CSV"); /* New file name, in the root directory, Chinese file name */

                    LongFileName[0] = 0X0054; /* Give UNICODE long file name */
                    LongFileName[1] = 0X0043; //TCBD_data_day.csv
                    LongFileName[2] = 0X0042;
                    LongFileName[3] = 0X0044;
                    LongFileName[4] = 0X005F;
                    LongFileName[5] = 0X0064;
                    LongFileName[6] = 0X0061;
                    LongFileName[7] = 0X0074;
                    LongFileName[8] = 0X0061;
                    LongFileName[9] = 0X005F;
                    LongFileName[10] = 0X0064;
                    LongFileName[11] = 0X0061;
                    LongFileName[12] = 0X0079;
                    LongFileName[13] = 0X002e;
                    LongFileName[14] = 0X0063;
                    LongFileName[15] = 0X0073;
                    LongFileName[16] = 0X0076;
                    LongFileName[17] = 0X0000;

                    s = CreatLongName(); /* Create a long file name */
                    if(s != ERR_SUCCESS)
                        PRINT("Error: %02x\n", s);
                    else
                        PRINT("Creat end\n");
                }
            }
        }
        mDelaymS(100);  // Simulate a microcontroller to do other things
        SetUsbSpeed(1); // Default is full speed
    }
}
