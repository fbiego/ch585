/* ********************************* (C) COPYRIGHT ***************************
 * File Name : wch_nfca_pcd_bsp.h
 * Author: WCH
 * Version: V1.1
 * Date: 2024/11/14
 * Description: NFC-A PCD BSP underlying interface
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#ifndef _WCH_NFCA_PCD_BSP_H_
#define _WCH_NFCA_PCD_BSP_H_

#include "wch_nfca_pcd_config.h"
#include "wch_nfca_pcd.h"

#define PU8_BUF(BUF)                    ((uint8_t *)BUF)
#define PU16_BUF(BUF)                   ((uint16_t *)BUF)
#define PU32_BUF(BUF)                   ((uint32_t *)BUF)

#define NFCA_PCD_MAX_SEND_NUM           (NFCA_PCD_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM           (NFCA_PCD_DATA_BUF_SIZE * 16 / 9)
#define NFCA_PCD_MAX_PARITY_NUM         (NFCA_PCD_MAX_RECV_NUM)

extern uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
extern uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
extern uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];

extern uint16_t g_nfca_pcd_recv_buf_len;
extern uint32_t g_nfca_pcd_recv_bits;

/* *********************************************************************************************
 * @fn nfca_pcd_init
 *
 * @brief nfc-a pcd initialization
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_init(void);

/* *********************************************************************************************
 * @fn nfca_pcd_start
 *
 * @brief nfc-a pcd starts running and starts sending continuous waves
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_start(void);

/* *********************************************************************************************
 * @fn nfca_pcd_stop
 *
 * @brief nfc-a pcd stops running, stops sending continuous waves
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_stop(void);

/* *********************************************************************************************
 * @fn nfca_pcd_lpcd_calibration
 *
 * @brief nfc-a pcd lpcd ADC card detection method threshold calibration
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_lpcd_calibration(void);

/* *********************************************************************************************
 * @fn nfca_pcd_lpcd_check
 *
 * @brief nfc-a pcd lpcd ADC check card
 *
 * @param None.
 *
 * @return 1 has a card, 0 has no card. */
extern uint8_t nfca_pcd_lpcd_check(void);

/* *********************************************************************************************
 * @fn nfca_adc_get_ant_signal
 *
 * @brief nfc-a pcd lpcd ADC check card
 *
 * @param None.
 *
 * @return uint16_t, returns the adc detection value of the signal on the antenna. */
extern uint16_t nfca_adc_get_ant_signal(void);

/* *********************************************************************************************
 * @fn nfca_pcd_wait_communicate_end
 *
 * @brief nfc-a pcd Wait for the end of communication
 *
 * @param None.
 *
 * @return nfca_pcd_controller_state_t, return to the communication end state. */
extern nfca_pcd_controller_state_t nfca_pcd_wait_communicate_end(void);

/* *********************************************************************************************
 * @fn nfca_pcd_rand
 *
 * @brief nfc-a pcd Get random number interface
 *
 * @param None.
 *
 * @return uint32_t, return a random number */
extern uint32_t nfca_pcd_rand(void);

/* *********************************************************************************************
 * @fn nfca_pcd_ctr_init
 *
 * @brief nfc-a pcd antenna signal control initialization
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_ctr_init(void);

/* *********************************************************************************************
 * @fn nfca_pcd_ctr_handle
 *
 * @brief nfc-a pcd antenna signal control processing
 *
 * @param None.
 *
 * @return None. */
extern void nfca_pcd_ctr_handle(void);

#endif /* _WCH_NFCA_PCD_BSP_H_ */
