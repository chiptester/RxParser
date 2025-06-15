/*
 * crsf_mini.h
 *
 *  Created on: Jun 14, 2025
 *      Author: chiptester
 */

#ifndef INC_CRSF_MINI_H_
#define INC_CRSF_MINI_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define CRSF_SYNC_BYTE         0xC8
#define CRSF_FRAME_SIZE_MAX    64


typedef struct crsfFrameDef_s {
    uint8_t syncbyte;
    uint8_t length;
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    uint8_t type;
    uint8_t crc8;
} crsfFrameDef_t;


struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

typedef struct crsfPayloadLinkstatistics_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

#endif /* INC_CRSF_MINI_H_ */
