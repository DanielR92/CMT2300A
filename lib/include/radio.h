/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    radio.h
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */
 
#ifndef __RADIO_H
#define __RADIO_H

#include "typedefs.h"
#include "cmt2300a.h"

/* RF state machine */
typedef enum {
    RF_STATE_IDLE = 0,
    RF_STATE_RX_START,
    RF_STATE_RX_WAIT,
    RF_STATE_RX_DONE,
    RF_STATE_RX_TIMEOUT,
    RF_STATE_TX_START,
    RF_STATE_TX_WAIT,
    RF_STATE_TX_DONE,
    RF_STATE_TX_TIMEOUT,
    RF_STATE_ERROR,
} EnumRFStatus;

/* RF process function results */
typedef enum {
    RF_IDLE = 0,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_ERROR,
} EnumRFResult;

//#define ENABLE_ANTENNA_SWITCH

#define U32_B3(val) ((uint8_t)((val >> 24) & 0xff))
#define U32_B2(val) ((uint8_t)((val >> 16) & 0xff))
#define U32_B1(val) ((uint8_t)((val >>  8) & 0xff))
#define U32_B0(val) ((uint8_t)((val      ) & 0xff))

#define HOY_BOOT_FREQ 868000000 // Hoymiles boot/init frequency after power up inverter

bool RF_Init(void);
bool RF_Config(void);

void RF_SetStatus(EnumRFStatus nStatus);
EnumRFStatus RF_GetStatus(void);
u8 RF_GetInterruptFlags(void);

void RF_StartRx(u8 buf[], u16 len, u32 timeout);
void RF_StartTx(u8 buf[], u16 len, u32 timeout);

EnumRFResult RF_Process(void);

void cmtTxData(uint8_t buf[], uint8_t len, const bool calcCrc16 = true, const bool calcCrc8 = true);
bool cmtSwitchDtuFreq(const uint32_t to_freq_kHz);
bool cmtSwitchInvAndDtuFreq(const uint32_t from_freq_kHz, const uint32_t to_freq_kHz, const uint64_t inv_serial, const uint32_t dtu_serial);

bool cmtFirstDataReceived(void);

#endif
