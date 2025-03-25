/* 2014.09.09
*****************************************
**   Copyright  (C)  W.ch  1999-2019   **
**   Web:      http://wch.cn           **
*****************************************
**  USB-flash File Interface for CHRV3 **
**  KEIL423, gcc 8.20          **
*****************************************
*/
/* CHRV3 USB host file system interface, support: FAT12/FAT16/FAT32 */

// #define DISK_BASE_BUF_LEN 512 /* The default disk data buffer size is 512 bytes (can be selected as 2048 or even 4096 to support USB disks with large sectors). If 0 is prohibited from defining buffers in this file and specified by the application in pDISK_BASE_BUF */
/* If you need to multiplex the disk data buffer to save RAM, then DISK_BASE_BUF_LEN can be defined as 0 to prohibit the definition of buffers in this file. The application will place the buffer start address used with other programs into the pDISK_BASE_BUF variable before calling CHRV3LibInit */

// #define NO_DEFAULT_ACCESS_SECTOR 1 /* The default disk sector reading and writing subroutine is prohibited, and the following is a self-written program instead of it */
// #define NO_DEFAULT_DISK_CONNECT 1 /* The default check of disk connection subroutine is prohibited, and the following is a self-written program instead of it */
// #define NO_DEFAULT_FILE_ENUMER 1 /* The default file name enumeration callback program is prohibited, and the following is a self-written program instead of it */
//#include "CHRV3SFR.H"

#include "CH58x_common.h"
#include "CHRV3UFI.h"
#include "usb_host_config.h"

uint8_t USBHostTransact( uint8_t endp_pid, uint8_t tog, uint32_t timeout )
{
#if DEF_USB_PORT_FS_EN
    uint8_t  r, trans_rerty;
    uint16_t i;
    USBFSD->HOST_TX_CTRL = USBFSD->HOST_RX_CTRL = 0;
    if( tog & 0x80 )
    {
        USBFSD->HOST_RX_CTRL = 1<<2;
    }
    if( tog & 0x40 )
    {
        USBFSD->HOST_TX_CTRL = 1<<2;
    }
    trans_rerty = 0;
    do
    {
        USBFSD->HOST_EP_PID = endp_pid;       // Specify token PID and endpoint number
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;  // Allow transmission
        for( i = DEF_WAIT_USB_TOUT_200US; ( i != 0 ) && ( ( USBFSD->INT_FG & USBFS_UIF_TRANSFER ) == 0 ); i-- )
        {
            DelayUs( 1 );
        }
        USBFSD->HOST_EP_PID = 0x00;  // Stop USB transfer
        if( ( USBFSD->INT_FG & USBFS_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }
        else
        {
            /* Complete transfer */
            if( USBFSD->INT_ST & USBFS_UIS_TOG_OK )
            {
                return ERR_SUCCESS;
            }
            r = USBFSD->INT_ST & USBFS_UIS_H_RES_MASK;  // USB device answer status
            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }
            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFF )
                {
                    timeout--;
                }
                --trans_rerty;
            }
            else switch ( endp_pid >> 4 )
            {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) && ( r == USB_PID_DATA1 ) )
                    {
                        ;
                    }
                    else if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        DelayUs( 15 );
        if( USBFSD->INT_FG & USBFS_UIF_DETECT )
        {
            DelayUs( 200 );
            if( USBFSH_CheckRootHubPortEnable( ) == 0 )
            {
                return ERR_USB_DISCON;  // USB device disconnect event
            }
        }
    }while( ++trans_rerty < 10 );

    return ERR_USB_TRANSFER; // Reply timeout
#elif DEF_USB_PORT_HS_EN

    return (USBHSH_Transact( endp_pid<<4 | endp_pid>>4 , tog, timeout ));


#endif
}

uint8_t HostCtrlTransfer( uint8_t *DataBuf, uint8_t *RetLen )
{
    uint8_t  ret;
    uint16_t retlen;
    retlen = (uint16_t)(*RetLen);
    ret = USBHSH_CtrlTransfer( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, DataBuf, &retlen );
    return ret;
}

uint8_t CtrlGetDeviceDescrTB( void )
{
    uint8_t ret;
    ret = USBHSH_GetDeviceDescr( &RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer );
    return ret;
}

uint8_t CtrlGetConfigDescrTB( void )
{
    uint16_t len;
    uint8_t  ret;
    ret = USBHSH_GetConfigDescr( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer, 256, &len );
    return ret;
}

uint8_t CtrlSetUsbConfig( uint8_t cfg )
{
    uint8_t ret;
    ret = USBHSH_SetUsbConfig( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, cfg );
    return ret;
}

uint8_t CtrlSetUsbAddress( uint8_t addr )
{
    uint8_t ret;
    ret = USBHSH_SetUsbAddress( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, addr );
    return ret;
}

uint8_t CtrlClearEndpStall( uint8_t endp )
{
    uint8_t ret;
    ret = USBHSH_ClearEndpStall( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, endp );
    return ret;
}

#ifndef FOR_ROOT_UDISK_ONLY
uint8_t CtrlGetHubDescr( void )
{

}

uint8_t HubGetPortStatus( uint8_t HubPortIndex )
{

}

uint8_t HubSetPortFeature( uint8_t HubPortIndex, uint8_t FeatureSelt )
{

}

uint8_t HubClearPortFeature( uint8_t HubPortIndex, uint8_t FeatureSelt )
{

}
#endif

CMD_PARAM_I	mCmdParam;						/* Command parameters */
#if		DISK_BASE_BUF_LEN > 0
// uint8_t DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((at(BA_RAM+SZ_RAM/2))); /* The disk data buffer of external RAM, the buffer length is the length of one sector */
uint8_t	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));	        /* The disk data buffer of external RAM, the buffer length is the length of a sector */
// UINT8 DISK_FAT_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4))); /* Disk FAT data buffer of external RAM, buffer length is the length of one sector */
#endif

/* The following programs can be modified as needed */

#ifndef	NO_DEFAULT_ACCESS_SECTOR		/* Defining NO_DEFAULT_ACCESS_SECTOR in the application can prohibit the default disk sector reading and writing subroutines and then replace it with a self-written program */
// if ( use_external_interface ) { // Replace the underlying read and write subroutine of the USB disk sector
// CHRV3vSectorSize=512; // Set the actual sector size, which must be a multiple of 512, and this value is the sector size of the disk
// CHRV3vSectorSizeB=9; // Set the displacement number of the actual sector size, 512 corresponds to 9, 1024 corresponds to 10, 2048 corresponds to 11
// CHRV3DiskStatus=DISK_MOUNTED; // Forced block device connection successfully (difference analysis file system only)
//}

uint8_t	CHRV3ReadSector( uint8_t SectCount, uint8_t *DataBuf )  /* Read data from multiple sectors into the buffer from disk */
{
    uint8_t	retry;
// if ( use_external_interface ) return( extReadSector( CHRV3vLbaCurrent, SectCount, DataBuf ) ); /* External interface */
	for( retry = 0; retry < 3; retry ++ ) {  /* Try again in error */
		pCBW -> mCBW_DataLen = (uint32_t)SectCount << CHRV3vSectorSizeB;  /* Data transmission length */
		pCBW -> mCBW_Flag = 0x80;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (uint8_t)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (uint8_t)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (uint8_t)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (uint8_t)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* Execute commands based on BulkOnly protocol */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* Disk operation error */
}

#ifdef	EN_DISK_WRITE
uint8_t	CHRV3WriteSector( uint8_t SectCount, uint8_t *DataBuf )  /* Write data blocks of multiple sectors in the buffer to disk */
{
    uint8_t	retry;
// if ( use_external_interface ) return( extWriteSector( CHRV3vLbaCurrent, SectCount, DataBuf ) ); /* External interface */
	for( retry = 0; retry < 3; retry ++ ) {  /* Try again in error */
		pCBW -> mCBW_DataLen = (uint32_t)SectCount << CHRV3vSectorSizeB;  /* Data transmission length */
		pCBW -> mCBW_Flag = 0x00;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_WRITE10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (uint8_t)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (uint8_t)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (uint8_t)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (uint8_t)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* Execute commands based on BulkOnly protocol */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			DelayUs( 200 );  /* Delay after write operation */
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* Disk operation error */
}
#endif
#endif  // NO_DEFAULT_ACCESS_SECTOR

#ifndef	NO_DEFAULT_DISK_CONNECT			/* Defining NO_DEFAULT_DISK_CONNECT in the application can prohibit the default check of disk connection subroutines and replace it with a self-written program */

/* Convention: USB device address allocation rules (refer to USB_DEVICE_ADDR)
Address Value Device Location
0x02 USB device or external HUB under built-in Root-HUB0
0x03 USB device or external HUB under built-in Root-HUB1
0x1x The USB device under port x of the external HUB under built-in Root-HUB0, x is 1~n
0x2x The USB device under port x of the external HUB under built-in Root-HUB1, x is 1~n */

#define		UHUB_DEV_ADDR	(R8_U2H_DEV_AD)
#define		UHPORT_STATUS	(R16_U2H_PORT_ST)
#define		bUMS_CONNECT	(1<<0)
#define		bUMS_SUSPEND	(1<<2)

/* Check if the disk is connected */
uint8_t	CHRV3DiskConnect( void )
{
    uint8_t	ums, devaddr;
	UHUB_DEV_ADDR = UHUB_DEV_ADDR & 0x7F;
	ums = UHPORT_STATUS;
	devaddr = UHUB_DEV_ADDR;
	if ( devaddr == USB_DEVICE_ADDR+1 )
	{
	    /* USB devices built-in Root-HUB */
		if ( ums & bUMS_CONNECT )
		{
		    /* The USB device under built-in Root-HUB exists */
			if ( ( ums & bUMS_SUSPEND ) == 0 )
			{
			    /* The USB device under the built-in Root-HUB exists and is not plugged in */
				return( ERR_SUCCESS );  /* The USB device is connected and not plugged in */
			}
			else
			{
			    /* The USB device under built-in Root-HUB exists */
				CHRV3DiskStatus = DISK_CONNECT;  /* Have been disconnected */
				return( ERR_SUCCESS );  /* The external HUB or USB device has been connected or disconnected and reconnected */
			}
		}
		else
		{
		    /* USB device disconnected */
mDiskDisconn:
			CHRV3DiskStatus = DISK_DISCONNECT;
			return( ERR_USB_DISCON );
		}
	}
	else
	{
		goto mDiskDisconn;
	}
}
#endif  // NO_DEFAULT_DISK_CONNECT

#ifndef	NO_DEFAULT_FILE_ENUMER			/* Defining NO_DEFAULT_FILE_ENUMER in the application can prohibit the default filename enumeration callback program and then replace it with a self-written program */
void xFileNameEnumer( void )			/* File name enumeration callback subroutine */
{
/* If you call FileOpen after specifying the enumeration number CHRV3vFileSize to 0xFFFFFFFF, then this callback will be called every time a file FileOpen is searched.
   After the callback xFileNameEnumer returns, FileOpen decrements CHRV3vFileSize and continues to enumerate until no file or directory is searched. The recommended approach is,
   Before calling FileOpen, define a global variable as 0. When FileOpen calls back to this program, this program obtains the structure FAT_DIR_INFO from CHRV3vFdtOffset.
   Analyze the DIR_Attr and DIR_Name in the structure to determine whether they are the required file name or directory name, record relevant information, and count the global variables into increments.
   When FileOpen returns, it is determined that if the return value is ERR_MISS_FILE or ERR_FOUND_NAME, it is considered that the operation is successful, and the global variable is the number of valid files found.
   If CHRV3vFileSize is set to 1 in this callback xFileNameEnumer, you can notify FileOpen to end the search in advance. The following is a callback example */
#if		0
    uint8_t			i;
    uint16_t	    FileCount;
	PX_FAT_DIR_INFO	pFileDir;
	uint8_t			*NameBuf;
	pFileDir = (PX_FAT_DIR_INFO)( pDISK_BASE_BUF + CHRV3vFdtOffset );  /* The starting address of the current FDT */
	FileCount = (UINT16)( 0xFFFFFFFF - CHRV3vFileSize );  /* The enumeration number of the current file name, the initial value of CHRV3vFileSize is 0xFFFFFFFF. After finding the file name, it is reduced. */
	if ( FileCount < sizeof( FILE_DATA_BUF ) / 12 ) {  /* Check whether the buffer is sufficient to store, assuming that each file name needs to occupy 12 bytes to store */
		NameBuf = & FILE_DATA_BUF[ FileCount * 12 ];  /* Calculate the buffer address that saves the current file name */
		for ( i = 0; i < 11; i ++ ) NameBuf[ i ] = pFileDir -> DIR_Name[ i ];  /* Copy file name, length is 11 characters, no spaces processed */
// if ( pFileDir -> DIR_Attr & ATTR_DIRECTORY ) NameBuf[ i ] = 1; /* It is judged as directory name */
		NameBuf[ i ] = 0;  /* File name ending character */
	}
#endif
}
#endif  // NO_DEFAULT_FILE_ENUMER

uint8_t	CHRV3LibInit( void )  /* Initialize the CHRV3 program library, the operation returns 0 successfully */
{
    uint8_t s;
    s = CHRV3GetVer( );
	if( s < CHRV3_LIB_VER )
	{
        return( 0xFF );  /* Get the version number of the current subroutine library. If the version is too low, the error will be returned. */
	}
	PRINT( "lib vision:%02x\r\n",s );
#if		DISK_BASE_BUF_LEN > 0
	pDISK_BASE_BUF = & DISK_BASE_BUF[0]; /* Disk data buffer pointing to external RAM */
	pDISK_FAT_BUF = & DISK_BASE_BUF[0];  /* The disk FAT data buffer pointing to external RAM can be used in conjunction with pDISK_BASE_BUF to save RAM */
// pDISK_FAT_BUF = & DISK_FAT_BUF[0]; /* The disk FAT data buffer pointing to the external RAM, independent of pDISK_BASE_BUF to increase speed */
/* If you want to improve file access speed, you can repoint pDISK_FAT_BUF to another independently allocated buffer of the same size as pDISK_BASE_BUF after calling CHRV3LibInit in the main program. */
#endif
	CHRV3DiskStatus = DISK_UNKNOWN;  /* Unknown status */
	CHRV3vSectorSizeB = 9;  /* The default physical disk sector is 512B */
	CHRV3vSectorSize = 512; /* The default sector of the physical disk is 512B, which is the sector size of the disk */
	CHRV3vStartLba = 0;     /* Default is to automatically analyze FDD and HDD */
	CHRV3vPacketSize = 512;  /* The maximum package length of USB storage device: 64@FS, 512@HS/SS, initialized by the application, enumerate the USB disk, if it is high-speed or overspeed, it will be updated to 512 in time. */

    pTX_DMA_A_REG = (uint32_t *)&(R32_U2H_TX_DMA);  /* Point to the send DMA address register, initialized by the application */
    pRX_DMA_A_REG = (uint32_t *)&(R32_U2H_RX_DMA);  /* Point to the receiving DMA address register, initialized by the application */
    pTX_LEN_REG = (uint16_t *)&(R32_U2H_TX_LEN);    /* Point to the send length register, initialized by the application */
    pRX_LEN_REG = (uint16_t *)&(R32_U2H_RX_LEN);         /* Point to the receive length register, initialized by the application */

	return( ERR_SUCCESS );
}

