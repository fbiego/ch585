/* ********************************* (C) COPYRIGHT ***************************
 * File Name : wch_nfca_picc_config.h
 * Author: WCH
 * Version: V1.0
 * Date: 2024/11/13
 * Description: nfc picc bsp underlying configuration
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#ifndef _WCH_NFCA_PICC_CONFIG_H_
#define _WCH_NFCA_PICC_CONFIG_H_

#define PICC_DATA_BUF_LEN           20          /* The NFC PICC signal data area and check bit area cache area size configuration can be configured according to the simulated card protocol. */
#define PICC_SIGNAL_BUF_LEN         512         /* The NFC PICC signal capture data buffer size configuration needs to meet the following conditions to judge. It will be more stable than the conditions, and the delay to interrupt can be extended. */

#if PICC_SIGNAL_BUF_LEN < (PICC_DATA_BUF_LEN * 18 + 12)
#error "PICC_SIGNAL_BUF_LEN must bigger than (PICC_DATA_BUF_LEN * 18 + 12)."
#endif

#define NFCA_PICC_RSP_POLAR         0           /* NFC PICC signal reply modulation polarity, generally no need to change */

#endif  /* _WCH_NFCA_PICC_CONFIG_H_ */
