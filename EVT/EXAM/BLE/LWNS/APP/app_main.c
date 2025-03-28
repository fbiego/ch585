/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/08/06
* Description        :
*******************************************************************************/



/******************************************************************************/
/* The header file contains */
#include "CONFIG.h"
#include "CH58x_common.h"
#include "HAL.h"
#include "stdlib.h"
#include "lwns_adapter_csma_mac.h"
#include "lwns_adapter_blemesh_mac.h"
#include "lwns_adapter_no_mac.h"
#include "lwns_broadcast_example.h"
#include "lwns_ruc_example.h"
#include "lwns_rucft_example.h"
#include "lwns_unicast_example.h"
#include "lwns_multicast_example.h"
#include "lwns_netflood_example.h"
#include "lwns_uninetflood_example.h"
#include "lwns_multinetflood_example.h"
#include "lwns_mesh_example.h"

// Each file has a separate debug print switch, setting 0 can prohibit internal printing of this file.
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
#define PRINTF(...) PRINT(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE/4];

#if (defined (BLE_MAC)) && (BLE_MAC == TRUE)
uint8_t const MacAddr[6] = {0x84,0xC2,0xE4,0x03,0x02,0x02};
#endif

/* *********************************************************************************************
* Function Name: Main_Circulation
* Description: Main loop
* Input: None
* Output: None
* Return : None
********************************************************************************************* */
__attribute__((section(".highcode")))
void Main_Circulation()
{
  while(1){
    TMOS_SystemProcess( );
  }
}

/* *********************************************************************************************
* Function Name : main
* Description: main function
* Input: None
* Output: None
* Return : None
********************************************************************************************* */
int main( void )
{
#if (defined (DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
  PWR_DCDCCfg( ENABLE );
#endif
  SetSysClock( CLK_SOURCE_HSE_PLL_62_4MHz );
#if (defined (HAL_SLEEP)) && (HAL_SLEEP == TRUE)
  GPIOA_ModeCfg( GPIO_Pin_All, GPIO_ModeIN_PU );
  GPIOB_ModeCfg( GPIO_Pin_All, GPIO_ModeIN_PU );
#endif
#ifdef DEBUG
  GPIOA_SetBits(GPIO_Pin_14);
  GPIOPinRemap(ENABLE, RB_PIN_UART0);
  GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
  GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
  UART0_DefInit();
#endif  
  PRINTF("start.\n");
  {
    PRINTF("%s\n",VER_LIB);
  }
  CH58x_BLEInit( );
  HAL_Init(  );
  RF_RoleInit( );
  RF_Init( );
  lwns_init();// Initial lwns protocol stack
  // lwns_broadcast_process_init();//Broadcast
  // lwns_multicast_process_init();//Multicast
  // lwns_unicast_process_init();//Unicast
  // lwns_ruc_process_init();//Reliable unicast
  // lwns_rucft_process_init();//Reliable unicast file transfer
  lwns_netflood_process_init();// Network flooding
  // lwns_uninetflood_process_init();//Unicast network flooding
  // lwns_multinetflood_process_init();//Multicast network flooding
  // lwns_mesh_process_init();//mesh networking
  Main_Circulation();
}
/******************************** endfile @ main ******************************/
