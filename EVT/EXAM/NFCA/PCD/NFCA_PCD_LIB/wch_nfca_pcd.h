/* ********************************* (C) COPYRIGHT ***************************
 * File Name : wch_nfca_pcd.h
 * Author: WCH
 * Version: V1.0
 * Date: 2024/08/22
 * Description: NFC-A PCD library header file
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#ifndef _WCH_NFCA_PCD_H_
#define _WCH_NFCA_PCD_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    NFCA_PCD_CONTROLLER_STATE_FREE = 0,
    NFCA_PCD_CONTROLLER_STATE_SENDING,
    NFCA_PCD_CONTROLLER_STATE_RECEIVING,
    NFCA_PCD_CONTROLLER_STATE_COLLISION,
    NFCA_PCD_CONTROLLER_STATE_OVERTIME,
    NFCA_PCD_CONTROLLER_STATE_DONE,
    NFCA_PCD_CONTROLLER_STATE_ERR,
} nfca_pcd_controller_state_t;

typedef enum
{
    NFCA_PCD_DRV_CTRL_LEVEL0        = (0x00 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL1        = (0x01 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL2        = (0x02 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL3        = (0x03 << 13),
} NFCA_PCD_DRV_CTRL_Def;

typedef enum
{
    NFCA_PCD_LP_CTRL_0_5_VDD        = (0x00 << 11),
    NFCA_PCD_LP_CTRL_0_6_VDD        = (0x01 << 11),
    NFCA_PCD_LP_CTRL_0_7_VDD        = (0x02 << 11),
    NFCA_PCD_LP_CTRL_0_8_VDD        = (0x03 << 11),
} NFCA_PCD_LP_CTRL_Def;

typedef enum
{
    NFCA_PCD_REC_GAIN_12DB          = (0x00 << 4),
    NFCA_PCD_REC_GAIN_18DB          = (0x01 << 4),
    NFCA_PCD_REC_GAIN_24DB          = (0x02 << 4),
    NFCA_PCD_REC_GAIN_30DB          = (0x03 << 4),
} NFCA_PCD_REC_GAIN_Def;

typedef enum
{
    NFCA_PCD_REC_THRESHOLD_100MV    = (0x00),
    NFCA_PCD_REC_THRESHOLD_150MV    = (0x01),
    NFCA_PCD_REC_THRESHOLD_200MV    = (0x02),
    NFCA_PCD_REC_THRESHOLD_250MV    = (0x03),
} NFCA_PCD_REC_THRESHOLD_Def;

typedef enum
{
    NFCA_PCD_REC_MODE_NONE          = 0,
    NFCA_PCD_REC_MODE_NORMAL        = 1,            /* No conflict detection is performed during reception, and decode as much as possible */
    NFCA_PCD_REC_MODE_COLI          = 0x10,         /* Conflict detection is performed during reception */
} NFCA_PCD_REC_MODE_Def;

typedef struct _nfca_pcd_config_struct
{
    uint16_t *data_buf;
    uint8_t *send_buf;
    uint8_t *recv_buf;
    uint8_t *parity_buf;

    uint16_t data_buf_size;
    uint16_t send_buf_size;
    uint16_t recv_buf_size;
    uint16_t parity_buf_size;
} nfca_pcd_config_t;

/* *********************************************************************************************
 * @fn nfca_pcd_lib_init
 *
 * @brief nfc-a initialization
 *
 * @param cfg - Configuration parameter pointer
 *
 * @return None. */
extern void nfca_pcd_lib_init(nfca_pcd_config_t *cfg);

/* *********************************************************************************************
 * @fn nfca_pcd_lib_start
 *
 * @brief nfc-a library starts running and starts sending continuous waves on the antenna
 *
 * @param None
 *
 * @return None. */
extern void nfca_pcd_lib_start(void);

/* *********************************************************************************************
 * @fn nfca_pcd_lib_stop
 *
 * @brief nfc-a library stops running and stops sending continuous waves on the antenna
 *
 * @param None
 *
 * @return None. */
extern void nfca_pcd_lib_stop(void);

/* *********************************************************************************************
 * @fn nfca_pcd_antenna_on
 *
 * @brief Start sending continuous waves on the antenna
 *
 * @param None
 *
 * @return None. */
extern void nfca_pcd_antenna_on(void);

/* *********************************************************************************************
 * @fn nfca_pcd_antenna_off
 *
 * @brief stop sending continuous waves on the antenna
 *
 * @param None
 *
 * @return None. */
extern void nfca_pcd_antenna_off(void);

/* *********************************************************************************************
 * @fn nfca_pcd_communicate
 *
 * @brief nfc-a starts communication and transmits data
 *
 * @param data_bits_num - uint16_t, the number of bits that need to be sent
 * @param mode - NFCA_PCD_REC_MODE_Def, reception mode after sending
 * @param offset - uint8_t(0 - 7), the number of offsets to be sent in the first byte
 *
 * @return 0 if success, others failed. */
extern uint8_t nfca_pcd_communicate(uint16_t data_bits_num, NFCA_PCD_REC_MODE_Def mode, uint8_t offset);

/* *********************************************************************************************
 * @fn nfca_pcd_get_communicate_status
 *
 * @brief nfc-a starts communication and transmits data
 *
 * @param None.
 *
 * @return nfca_pcd_controller_state_t, get the current communication status. . */
extern nfca_pcd_controller_state_t nfca_pcd_get_communicate_status(void);


/* *********************************************************************************************
 * @fn nfca_pcd_get_recv_data_len
 *
 * @brief Get the data length received and decoded this time
 *
 * @param None
 *
 * @return uint16_t - Data length. */
extern uint16_t nfca_pcd_get_recv_data_len(void);

/* *********************************************************************************************
 * @fn nfca_pcd_get_recv_bits
 *
 * @brief Get the number of bits received this time
 *
 * @param None
 *
 * @return uint16_t - The number of bits received. */
extern uint16_t nfca_pcd_get_recv_bits(void);

/* *********************************************************************************************
 * @fn nfca_pcd_set_out_drv
 *
 * @brief nfc-a sets the antenna output impedance, default Level1
 *
 * @param drv - NFCA_PCD_DRV_CTRL_Def, antenna transmit pin output level
 *
 * @return None. */
extern void nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_Def drv);

/* *********************************************************************************************
 * @fn nfca_pcd_set_recv_gain
 *
 * @brief nfc-a sets the receive gain, default 18DB
 *
 * @param gain - NFCA_PCD_REC_GAIN_Def, receive gain
 *
 * @return None. */
extern void nfca_pcd_set_recv_gain(NFCA_PCD_REC_GAIN_Def gain);

/* *********************************************************************************************
 * @fn nfca_pcd_set_lp_ctrl
 *
 * @brief nfc-a sets the antenna signal detection gear, default is 0.8VDD
 *
 * @param lp - NFCA_PCD_LP_CTRL_Def, antenna signal detection gear
 *
 * @return None. */
extern void nfca_pcd_set_lp_ctrl(NFCA_PCD_LP_CTRL_Def lp);

/* *********************************************************************************************
 * @fn nfca_pcd_set_rec_threshold
 *
 * @brief nfc-a sets comparison threshold, default is 150mv
 *
 * @param th - NFCA_PCD_REC_THRESHOLD_Def, comparison threshold for decoding analog signal
 *
 * @return None. */
extern void nfca_pcd_set_rec_threshold(NFCA_PCD_REC_THRESHOLD_Def th);

/* *********************************************************************************************
 * @fn nfca_pcd_set_wait_ms
 *
 * @brief NFC sets the reception timeout time
 *
 * @param us - uint16_t, timeout time, unit ms, maximum 38ms.
 *
 * @return None. */
extern void nfca_pcd_set_wait_ms(uint8_t ms);

/* *********************************************************************************************
 * @fn nfca_pcd_set_wait_us
 *
 * @brief NFC sets the reception timeout time
 *
 * @param us - uint16_t, timeout time, unit us, maximum 38000us.
 *
 * @return None. */
extern void nfca_pcd_set_wait_us(uint16_t us);

/* *********************************************************************************************
 * @fn nfca_pcd_get_lp_status
 *
 * @brief Is the NFC reading antenna signal too low
 *
 * @param None.
 *
 * @return 1 is below the set threshold, 0 is not below the set threshold. */
extern uint8_t nfca_pcd_get_lp_status(void);

/* *********************************************************************************************
 * @fn NFC_IRQLibHandler
 *
 * @brief NFC interrupt handling function
 *
 * @param None
 *
 * @return None. */
extern void NFC_IRQLibHandler(void);

#ifdef __cplusplus
}
#endif

#endif  /* _WCH_NFCA_PCD_H_ */
