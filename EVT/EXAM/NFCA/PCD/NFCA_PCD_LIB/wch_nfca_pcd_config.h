/* ********************************* (C) COPYRIGHT ***************************
 * File Name : wch_nfca_pcd_config.h
 * Author: WCH
 * Version: V1.1
 * Date: 2024/11/14
 * Description: NFC-A PCD underlying configuration
 ************************************************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ********************************************************************************************* */

#ifndef _WCH_NFCA_PCD_CONFIG_H_
#define _WCH_NFCA_PCD_CONFIG_H_

#include "CH58x_common.h"

#ifndef NFCA_PCD_USE_NFC_CTR_PIN
#define NFCA_PCD_USE_NFC_CTR_PIN                                1       /* Whether to use the NFC PCD antenna control pin NFC_CTR, this pin can be used to control the antenna signal voltage division to obtain a wider operating range */
#endif

#ifndef NFCA_PCD_DATA_BUF_SIZE
#define NFCA_PCD_DATA_BUF_SIZE                                  32      /* NFC PCD signal data buffer size configuration */
#endif

#if NFCA_PCD_DATA_BUF_SIZE < 20
#error "NFCA_PCD_DATA_BUF_SIZE must bigger or equal than 20."
#endif

#ifndef NFCA_PCD_WAIT_MAX_MS
#define NFCA_PCD_WAIT_MAX_MS                                    1000    /* The maximum waiting time for each communication is exceeded, if the exception ends, and it needs to be modified according to the situation of different cards. */
#endif

#ifndef NFCA_PCD_LPCD_THRESHOLD_PERMIL
#define NFCA_PCD_LPCD_THRESHOLD_PERMIL                          5       /* ADC difference ratio threshold for low power detection cards, thousand percent. It cannot be less than 3, too small may lead to false awakening */
#endif

#if NFCA_PCD_LPCD_THRESHOLD_PERMIL < 3
#error "NFCA_PCD_LPCD_THRESHOLD_PERMIL must bigger or equal than 3."
#endif

#ifndef NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL
#define NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL                20      /* The threshold value is updated when the detection value of the low-power detection card becomes larger, and the limit is performed. */
#endif

#ifndef NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL
#define NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL                1       /* The detection value of the low-power detection card changes to the smallest threshold update limit, a thousandth percentile. */
#endif

#endif  /* _WCH_NFCA_PCD_CONFIG_H_ */
