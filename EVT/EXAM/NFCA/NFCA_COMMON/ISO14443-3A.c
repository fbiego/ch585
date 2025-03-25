/* ********************************* (C) COPYRIGHT ***************************
 * File Name: ISO14443-3A.c
 * Author: WCH
 * Version: V1.1
 * Date: 2024/11/12
 * Description: ISO14443-3A related functions
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************************* */
#include "ISO14443-3A.h"

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

__attribute__((always_inline)) RV_STATIC_INLINE uint16_t _crc_ccitt_update(uint16_t crc, uint8_t data)
{
    data ^= LO_UINT16(crc);
    data ^= data << 4;

    return ((((uint16_t)data << 8) | HI_UINT16(crc)) ^ (uint8_t)(data >> 4)
            ^ ((uint16_t)data << 3));
}

/* ***************************************************************************
 * @fn ISO14443_CRCA
 *
 * @brief Check if CRC is correct
 *
 * @param buf - Data pointer
 * @param len - Data length
 *
 * @return 0 if everything is ok, others is error. */
__attribute__((section(".highcode")))
uint16_t ISO14443_CRCA(uint8_t *buf, uint8_t len)
{
    uint8_t *data = buf;
    uint16_t crc = CRC_INIT;
    uint8_t ch;
    while (len--)
    {
        ch = *data++ ^ crc;
        ch = ch ^ (ch << 4);
        crc = (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4);
    }
    return crc;
}

/* ***************************************************************************
 * @fn ISO14443AAppendCRCA
 *
 * @brief generates CRC at the end of the data
 *
 * @param buf - Data pointer
 * @param len - Data length
 *
 * @return CRC. */
__attribute__((section(".highcode")))
uint16_t ISO14443AAppendCRCA(void *buf, uint16_t len)
{
    uint16_t crc = CRC_INIT;
    uint8_t *data = (uint8_t *) buf;

    while (len--)
    {
        uint8_t byte8 = *data++;
        crc = _crc_ccitt_update(crc, byte8);
    }

    data[0] = (crc >> 0) & 0x00FF;
    data[1] = (crc >> 8) & 0x00FF;

    return crc;
}

/* ***************************************************************************
 * @fn ISO14443ACheckOddParityBit
 *
 * @brief Check odd check bit
 *
 * @param data - Data pointer
 * @param parity - check digit pointer
 * @param len - Data length
 *
 * @return 1 if ok, 0 if error. */
__attribute__((section(".highcode")))
uint8_t ISO14443ACheckOddParityBit(uint8_t *data, uint8_t *parity, uint16_t len)
{
    uint16_t i;
    uint8_t odd;

    for (i = 0; i < len; i++)
    {
        if ((byteParityBitsTable[data[i]]) != parity[i])
        {
            return 0;
        }
    }
    return 1;
}

/* ***************************************************************************
 * @fn ISO14443ACalOddParityBit
 *
 * @brief calculate odd check bits
 *
 * @param data - Data pointer
 * @param out_parity - check digit pointer
 * @param len - Data length
 *
 * @return None. */
__attribute__((section(".highcode")))
void ISO14443ACalOddParityBit(uint8_t *data, uint8_t *out_parity, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        out_parity[i] = byteParityBitsTable[data[i]];
    }
}

__attribute__((always_inline)) RV_STATIC_INLINE  uint8_t _ISO14443ASelectB0(uint8_t *data, uint8_t *uid, uint8_t collision_byte_cnt, uint8_t collision_bit_cnt)
{
    if((collision_byte_cnt == 4) && (collision_bit_cnt == 0))
    {
        if((uid[0] == data[2]) && (uid[1] == data[3]) && (uid[2] == data[4]) && (uid[3] == data[6]))
        {
            return 1;
        }
    }
    return 0;
}

__attribute__((always_inline)) RV_STATIC_INLINE  uint8_t _ISO14443ASelectB1(uint8_t *data, uint8_t mask, uint8_t *uid, uint8_t collision_byte_cnt, uint8_t collision_bit_cnt)
{
    uint8_t i;
    if(collision_byte_cnt < 4)
    {
        if((uid[collision_byte_cnt] & mask) == (data[collision_byte_cnt + 2] & mask))
        {
            for(i = 0; i < collision_byte_cnt; i++)
            {
                if(uid[i] != data[2 + i])
                {
                    break;
                }
            }
            if(i == collision_byte_cnt)
            {
                return 1;
            }
        }
    }
    return 0;
}

__attribute__((always_inline)) RV_STATIC_INLINE  uint8_t _ISO14443ASelectB2(uint8_t *data, uint8_t mask, uint8_t *uid, uint8_t collision_byte_cnt, uint8_t collision_bit_cnt)
{
    if((collision_byte_cnt == 5) || ((collision_byte_cnt == 4) && (collision_bit_cnt > 0)))
    {
        if((uid[0] == data[2]) && (uid[1] == data[3]) && (uid[2] == data[4]) && (uid[3] == data[6]))
        {
            if((ISO14443A_CALC_BCC(uid) & mask) == (data[6] & mask))
            {
                return 1;
            }
        }
    }
    return 0;
}

/* ***************************************************************************
 * @fn ISO14443ASelect
 *
 * @brief card selection process
 *
 * @param data - Data pointer
 * @param bit_count - Data bit count pointer
 * @param uid - its own card uid data pointer
 * @param sak - its own card sak value
 *
 * @return 1 - Still in the card selection process, 0 - The card selection process ends. */
__attribute__((section(".highcode")))
uint8_t ISO14443ASelect(uint8_t *data, uint16_t *bit_count, uint8_t *uid, uint8_t sak)
{
    uint8_t cmd = data[1];
    uint8_t collision_byte_cnt, collision_bit_cnt, mask;

    switch (cmd)
    {
    case ISO14443A_NVB_AC_START:
#if IS014443A_FAST_CL == 0
        data[0] = uid[0];
        data[1] = uid[1];
        data[2] = uid[2];
        data[3] = uid[3];
        data[4] = ISO14443A_CALC_BCC(data);
#endif
        *bit_count = ISO14443A_CL_FRAME_SIZE;
        return 1;

    case ISO14443A_NVB_AC_END:
        if ((data[2] == uid[0]) && (data[3] == uid[1]) && (data[4] == uid[2]) && (data[5] == uid[3]))
        {
#if IS014443A_FAST_CL == 0
            data[0] = sak;
            ISO14443AAppendCRCA((void *)data, 1);
#endif
            *bit_count = ISO14443A_SAK_FRAME_SIZE;
            return 0;
        }
        break;
    default:
        collision_byte_cnt = ((cmd >> 4) & 0x0f) - 2;
        collision_bit_cnt  = (cmd >> 0) & 0x0f;
        mask = 0xFF >> (8 - collision_bit_cnt);

        if (_ISO14443ASelectB0(data, uid, collision_byte_cnt, collision_bit_cnt)        ||
            _ISO14443ASelectB1(data, mask, uid, collision_byte_cnt, collision_bit_cnt)  ||
            _ISO14443ASelectB2(data, mask, uid, collision_byte_cnt, collision_bit_cnt))
        {
#if IS014443A_FAST_CL == 0
            data[0] = uid[0];
            data[1] = uid[1];
            data[2] = uid[2];
            data[3] = uid[3];
            data[4] = ISO14443A_CALC_BCC(data);
#endif
            *bit_count = ISO14443A_CL_FRAME_SIZE;
            return 1;
        }
        break;
    }

    *bit_count = 0;
    return 1;
}
