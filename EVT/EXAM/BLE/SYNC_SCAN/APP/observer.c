/* ********************************* (C) COPYRIGHT ***************************
 * File Name: observer.c
 * Author: WCH
 * Version: V1.0
 * Date: 2018/12/10
 * Description: Observe the application, initialize the scan parameters, and then scan regularly. If the scan result is not empty, print the scanned broadcast address.
 ************************************************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "observer.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES             58

// Scan duration in (625us)
#define DEFAULT_SCAN_DURATION            4800

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE           DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN    FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST     FALSE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8_t gStatus;

// Task ID for internal task/event processing
static uint8_t ObserverTaskId;

// Number of scan results and scan result index
static uint8_t ObserverScanRes;

// Current sync status
static uint8_t syncStatus=0;

// Scan result list
static gapDevRec_t ObserverDevList[DEFAULT_MAX_SCAN_RES];

// Peer device address
static uint8_t PeerAddrDef[B_ADDR_LEN] = {0x02, 0x02, 0x03, 0xE4, 0xC2, 0x84};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ObserverEventCB(gapRoleEvent_t *pEvent);
static void Observer_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void ObserverAddDeviceInfo(uint8_t *pAddr, uint8_t addrType);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapRoleObserverCB_t ObserverRoleCB = {
    ObserverEventCB // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Observer_Init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Observer_Init()
{
    ObserverTaskId = TMOS_ProcessEventRegister(Observer_ProcessEvent);

    // Setup Observer Profile

    // Setup GAP
    GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_DISC_SCAN_PHY, GAP_PHY_BIT_LE_1M);

    // Setup a delayed profile startup
    tmos_set_event(ObserverTaskId, START_DEVICE_EVT);
}

/*********************************************************************
 * @fn      Observer_ProcessEvent
 *
 * @brief   Simple BLE Observer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Observer_ProcessEvent(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(ObserverTaskId)) != NULL)
        {
            Observer_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_ObserverStartDevice((gapRoleObserverCB_t *)&ObserverRoleCB);

        return (events ^ START_DEVICE_EVT);
    }

    if(events & START_SYNC_TIMEOUT_EVT)
    {
        syncStatus = 0;
        GAPRole_CancelSync();
        PRINT("Creat SYNC timeout, restart discovery..\n");
        GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST);
        return (events ^ START_SYNC_TIMEOUT_EVT);
    }
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Observer_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Observer_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GATT_MSG_EVENT:
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      ObserverEventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void ObserverEventCB(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST);
            PRINT("Discovering...\n");
        }
        break;

        case GAP_DEVICE_INFO_EVENT:
        {
//            ObserverAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType);
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
        {
            PRINT("Discovery over...\n");
            ObserverScanRes = 0;
            if( syncStatus < 2 )
            {
                GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                               DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                               DEFAULT_DISCOVERY_WHITE_LIST);
            }
        }
        break;

        case GAP_EXT_ADV_DEVICE_INFO_EVENT:
        {
            if(tmos_memcmp(PeerAddrDef, pEvent->deviceExtAdvInfo.addr, B_ADDR_LEN) &&
               (pEvent->deviceExtAdvInfo.periodicAdvInterval != 0) && (!syncStatus))
            {
                gapCreateSync_t sync = {0};
                uint8_t           state;

                tmos_memcpy(sync.addr, pEvent->deviceExtAdvInfo.addr, B_ADDR_LEN);
                sync.addrType = pEvent->deviceExtAdvInfo.addrType;
                sync.advertising_SID = pEvent->deviceExtAdvInfo.advertisingSID;
                sync.options = DUPLICATE_FILTERING_INITIALLY_ENABLED;
                sync.syncTimeout = (pEvent->deviceExtAdvInfo.periodicAdvInterval * 6 + 7) / 8; //6 times the periodicAdvInterval
                
                /* Only one sync can be established at the same time for now */
                state = GAPRole_CreateSync(&sync);

                PRINT("GAPRole_CreateSync %d return %d...\n ", pEvent->deviceExtAdvInfo.periodicAdvInterval, state);
                if (state == SUCCESS) {
                    syncStatus = 1;
                    tmos_start_task(ObserverTaskId, START_SYNC_TIMEOUT_EVT, sync.syncTimeout*16);
                }
            }
            PRINT("GAP_EXT_ADV_DEVICE_INFO_EVENT...\n");
        }
        break;

        case GAP_SYNC_ESTABLISHED_EVENT:
        {
            if(pEvent->syncEstEvt.status == SUCCESS)
            {
                GAPRole_ObserverCancelDiscovery();
                syncStatus = 2;
                tmos_stop_task(ObserverTaskId, START_SYNC_TIMEOUT_EVT);
                PRINT("GAP_SYNC_ESTABLISHED...\n");
                PRINT("sync handle: %#x\n", pEvent->syncEstEvt.syncHandle);
                PRINT("sync interval: %d\n", pEvent->syncEstEvt.periodicInterval);
            }
        }
        break;

        case GAP_PERIODIC_ADV_DEVICE_INFO_EVENT:
        {
            PRINT("periodic adv - len %d (", pEvent->devicePeriodicInfo.dataLength);
            for (int i = 0 ; i < pEvent->devicePeriodicInfo.dataLength; i++) {
                PRINT(" %#x", pEvent->devicePeriodicInfo.pEvtData[i]);
            }
            PRINT(" )\n");
        }
        break;

        case GAP_SYNC_LOST_EVENT:
        {
            syncStatus = 0;
            GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST);
            PRINT("GAP_SYNC_LOST ...\n");
        }
        break;

        default:
            PRINT("opcode %x ...\n", pEvent->gap.opcode);
            break;
    }
}

/*********************************************************************
 * @fn      ObserverAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void ObserverAddDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    // If result count not at max
    if(ObserverScanRes < DEFAULT_MAX_SCAN_RES)
    {
        // Check if device is already in scan results
        for(i = 0; i < ObserverScanRes; i++)
        {
            if(tmos_memcmp(pAddr, ObserverDevList[i].addr, B_ADDR_LEN))
            {
                return;
            }
        }

        // Add addr to scan result list
        tmos_memcpy(ObserverDevList[ObserverScanRes].addr, pAddr, B_ADDR_LEN);
        ObserverDevList[ObserverScanRes].addrType = addrType;

        // Increment scan result count
        ObserverScanRes++;
        PRINT("Device %d - Addr %x %x %x %x %x %x \n", ObserverScanRes,
              ObserverDevList[ObserverScanRes - 1].addr[0],
              ObserverDevList[ObserverScanRes - 1].addr[1],
              ObserverDevList[ObserverScanRes - 1].addr[2],
              ObserverDevList[ObserverScanRes - 1].addr[3],
              ObserverDevList[ObserverScanRes - 1].addr[4],
              ObserverDevList[ObserverScanRes - 1].addr[5]);
    }
}

/*********************************************************************
*********************************************************************/
