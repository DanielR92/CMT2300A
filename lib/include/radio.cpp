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
 * @file    radio.c
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#include <Arduino.h>
#include "radio.h"
#include "cmt2300a_params.h"
#include "crc.h"
#include "debug_helper.h"
#include "config.h"

#include <string.h>

static EnumRFStatus g_nNextRFState = RF_STATE_IDLE;
static u8* g_pRxBuffer = NULL;
static u8* g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;

static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static u32 g_nRxTimeCount = 0;
static u32 g_nTxTimeCount = 0;

static u8 g_nInterrutFlags = 0;

bool cmtGotPktOKIntr;               // GPIO3 PKT OK interrupt

#define CMT2300A_ONE_STEP_SIZE 2500 // frequency channel step size for fast frequency hopping operation: One step size is 2.5 kHz.
#define FH_OFFSET 100               // value * CMT2300A_ONE_STEP_SIZE = channel frequency offset
#define HOY_BASE_FREQ 860000000     // Hoymiles base frequency for CMD56 channels is 860.00 MHz

uint8_t cmtBaseChOff860;            // offset from initalized CMT base frequency to Hoy base frequency in channels
uint8_t cmtActualCh;                // actual used channel, should be stored per inverter und set before next Tx, if hopping is used

bool cmtFirstRx;                    // first data received

uint8_t cmtRec[10][32];             // 10 frames with max. 32 byte
uint8_t cmtRecId;                   // actual frame count (++ for each received, irrespective of frame number)
int8_t cmtRssi[10];                 // RSSI per frame
uint32_t cmtRxTime[10];             // offset to start time per frame
bool cmtRxFrag[10];                 // frame received per frame
uint8_t cmtMaxFragId;               // set max frameID with 0x8_ number
uint8_t cmtLastRecId;               // count of last received frame (irrespective of frame number)

IRAM_ATTR void cmtHandlePktOkIntr(void) {
    cmtGotPktOKIntr = true;
}

bool RF_Init(void)
{
    cmtFirstRx = false;

    u8 tmp;

    CMT2300A_InitSpi();
	if(!CMT2300A_Init())
        return false;
    
    /* Config registers */
    CMT2300A_ConfigRegBank(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank          , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank       , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank_EU , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank     , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank_EU  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank           , CMT2300A_TX_BANK_SIZE        );

    cmtBaseChOff860 = (863000000 - HOY_BASE_FREQ) / CMT2300A_ONE_STEP_SIZE / FH_OFFSET; // 3 MHz(diff) / 2.5 kHz / 100 = channel 12

    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10, tmp|0x02);
    
	if(!RF_Config())
        return false;
    
    // GPIO3 PKT OK interrupt
    cmtGotPktOKIntr = false;
    pinMode(CMT_PIN_GPIO3, INPUT);
    attachInterrupt(digitalPinToInterrupt(CMT_PIN_GPIO3), cmtHandlePktOkIntr, RISING);

    return true;
}

bool RF_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch(0);
    
#else
    /* Config GPIOs */
    CMT2300A_ConfigGpio(
        CMT2300A_GPIO3_SEL_INT2
        );
    
    /* Config interrupt */
    CMT2300A_ConfigInterrupt(
        CMT2300A_INT_SEL_TX_DONE, /* Config INT1 */
        CMT2300A_INT_SEL_PKT_OK   /* Config INT2 */
        );
#endif

    /* Enable interrupt */
    CMT2300A_EnableInterrupt(
        CMT2300A_MASK_TX_DONE_EN  |
		CMT2300A_MASK_PREAM_OK_EN |
		CMT2300A_MASK_SYNC_OK_EN  |
		CMT2300A_MASK_CRC_OK_EN   |
		CMT2300A_MASK_PKT_DONE_EN
        );
    
    /* Disable low frequency OSC calibration */
    //CMT2300A_EnableLfosc(FALSE);

    CMT2300A_SetFrequencyStep(100); // set FH_OFFSET to 100 (frequency = base freq + 2.5kHz*FH_OFFSET*FH_CHANNEL)

    /* Use a single 64-byte FIFO for either Tx or Rx */
    CMT2300A_EnableFifoMerge(TRUE);
    
    //CMT2300A_SetFifoThreshold(16);
    
    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);
    
    /* Go to sleep for configuration to take effect */
    if(!CMT2300A_GoSleep())
        return false; // CMT2300A not switched to sleep mode!

    return true;
}

void RF_SetStatus(EnumRFStatus nStatus)
{
    g_nNextRFState = nStatus;
}

EnumRFStatus RF_GetStatus(void)
{
    return g_nNextRFState;
}

u8 RF_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF_StartRx(u8 buf[], u16 len, u32 timeout) // actual unused
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;
    
    memset(g_pRxBuffer, 0, g_nRxLength);
    
    g_nNextRFState = RF_STATE_RX_START;
}

void RF_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;
    
    g_nNextRFState = RF_STATE_TX_START;
}

String cmtGetActFreq(void) {
    return String((HOY_BASE_FREQ + (cmtBaseChOff860 + cmtActualCh) * FH_OFFSET * CMT2300A_ONE_STEP_SIZE) / 1000000.0, 2) + String(" MHz");
}

void cmtBuild(void) {
    uint8_t payload[200];
    memset(payload, 0xcc, 200);

    uint8_t pos = 0;
    uint8_t len = 0;

    for(uint8_t i = 0; i < cmtMaxFragId; i++) {
        len = cmtRec[i][0]-1-10;
        memcpy(&payload[pos], &cmtRec[i][11], len);
        pos += len;
    }

    uint16_t crc = crc16(payload, pos-2, 0xffff);
    uint16_t recCrc = (payload[pos-2] << 8) | payload[pos-1];

    if(crc == recCrc) {
        //Serial.println("[cmtBuild] PAYLOAD OK");
        printData(payload, pos-2);
    }
    else
        Serial.println("[cmtBuild] PAYLOAD CRC ERROR: " + String(crc, HEX) + " != " + String(recCrc, HEX) + " cmtMaxFragId: " + String(cmtMaxFragId));
}

EnumRFResult RF_Process(void)
{
    EnumRFResult nRes = RF_BUSY;
    
    switch(g_nNextRFState) 
    {
    case RF_STATE_IDLE:
    {
        nRes = RF_IDLE;
        break;
    }
    
    case RF_STATE_RX_START:
    {
        CMT2300A_GoStby();
        CMT2300A_ClearInterruptFlags();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableReadFifo();
        CMT2300A_ClearRxFifo();
        
        if(FALSE==CMT2300A_GoRx())
            g_nNextRFState = RF_STATE_ERROR;
        else
            g_nNextRFState = RF_STATE_RX_WAIT;
        
        g_nRxTimeCount = CMT2300A_GetTickCount();
        
        break;
    }
    
    case RF_STATE_RX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH
        if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG))  /* Read PKT_OK flag */
#else
        if(cmtGotPktOKIntr)  /* Read INT2, PKT_OK */
#endif
        {
            cmtGotPktOKIntr = false; // reset interrupt
            g_nNextRFState = RF_STATE_RX_DONE;
        }
        
        if( (INFINITE != g_nRxTimeout) && ((CMT2300A_GetTickCount()-g_nRxTimeCount) > g_nRxTimeout) )
            g_nNextRFState = RF_STATE_RX_TIMEOUT;
        
        break;
    }
    
    case RF_STATE_RX_DONE:
    {
        CMT2300A_GoStby();

        uint8_t state = CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG);
        if((state & 0x1b) == 0x1b) {
            cmtFirstRx = true;

            // frame handling
            if(cmtRecId < 10) {
                cmtRxTime[cmtRecId] = millis() - cmtRxTime[cmtRecId];
                CMT2300A_ReadFifo(cmtRec[cmtRecId], 28); /* The length need be smaller than 32 */
                cmtRssi[cmtRecId] = CMT2300A_ReadReg(CMT2300A_CUS_RSSI_DBM) - 128;
                cmtRxFrag[(cmtRec[cmtRecId][10] & 0x7f) - 1] = true;
                if(cmtRec[cmtRecId][10] & 0x80) // last frame
                    cmtMaxFragId = cmtRec[cmtRecId][10] & 0x7f;
                cmtRecId++;
            }
        }
        else if ((state & 0x19) == 0x19)
            Serial.println("[cmtGetRx] state: " + String(state, HEX) + " (CRC ERROR)");
        else
            Serial.println("[cmtGetRx] wrong state: " + String(state, HEX));

        g_nInterrutFlags = CMT2300A_ClearInterruptFlags();
            
        CMT2300A_GoSleep();

        if((cmtRecId > 0) && (cmtLastRecId != cmtRecId)) {
            cmtLastRecId = cmtRecId;
        }

        if(cmtMaxFragId) { // last frame received
            // check if frames are complete
            bool complete = true;
            for(uint8_t i = 0; i < cmtMaxFragId; i++) {
                if(cmtRxFrag[i] == false) {
                    complete = false;
                    break;
                }
            }

            if(complete) {
                Serial.println("-----------------------------------");
                for(uint8_t i = 0; i < cmtRecId; i++) {
                    dumpBuf("RX", cmtRec[i], 28, false);
                    Serial.print(" | " + String(cmtGetActFreq()));
                    Serial.print(" | " + String(cmtRxTime[i]));
                    Serial.println(" ms | " + String(cmtRssi[i]) + " dBm");
                }
                //Serial.println("[cmtGetRx] complete!");
                cmtBuild();
            }
            else {
                Serial.println("[cmtGetRx] failed!, frame(s) missing");
            }
        }
        
        //g_nNextRFState = RF_STATE_IDLE;
        g_nNextRFState = RF_STATE_RX_START; // next answer
        nRes = RF_RX_DONE;
        break;
    }
    
    case RF_STATE_RX_TIMEOUT:
    {
        CMT2300A_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_RX_TIMEOUT;
        break;
    }
    
    case RF_STATE_TX_START:
    {
        CMT2300A_GoStby();
        CMT2300A_ClearInterruptFlags();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableWriteFifo();
        CMT2300A_ClearTxFifo();
        
        CMT2300A_WriteReg(0x46, (u8)g_nTxLength&0xFF); // from orig DTU
        /* The length need be smaller than 32 */
        CMT2300A_WriteFifo(g_pTxBuffer, g_nTxLength);
        
        if( 0==(CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG)) )
            g_nNextRFState = RF_STATE_ERROR;

        if(FALSE==CMT2300A_GoTx())
            g_nNextRFState = RF_STATE_ERROR;
        else
            g_nNextRFState = RF_STATE_TX_WAIT;
        
        g_nTxTimeCount = CMT2300A_GetTickCount();
        
        break;
    }
        
    case RF_STATE_TX_WAIT:
    {
//#ifdef ENABLE_ANTENNA_SWITCH
        if(CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1))  /* Read TX_DONE flag */
//#else
//        if(CMT2300A_ReadGpio1())  /* Read INT1, TX_DONE */
//#endif
        {
            g_nNextRFState = RF_STATE_TX_DONE;
        }
        
        if( (INFINITE != g_nTxTimeout) && ((CMT2300A_GetTickCount()-g_nTxTimeCount) > g_nTxTimeout) )
            g_nNextRFState = RF_STATE_TX_TIMEOUT;
            
        break;
    }
            
    case RF_STATE_TX_DONE:
    {
        CMT2300A_ClearInterruptFlags();
        CMT2300A_GoSleep();

        //g_nNextRFState = RF_STATE_IDLE;
        g_nNextRFState = RF_STATE_RX_START; // receive answer
        g_nTxTimeout = 500;
        nRes = RF_TX_DONE;
        break;
    }
    
    case RF_STATE_TX_TIMEOUT:
    {
        CMT2300A_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_TX_TIMEOUT;
        break;
    }
    
    case RF_STATE_ERROR:
    {
        CMT2300A_SoftReset();
        CMT2300A_DelayMs(20);
        
        CMT2300A_GoStby();
        RF_Config();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_ERROR;
        break;
    }
    
    default:
        break;
    }
    
    return nRes;
}

void cmtTxData(uint8_t buf[], uint8_t len, const bool calcCrc16, const bool calcCrc8) {
    // clear receive data
    memset(cmtRec, 0xcc, 10 * 32);
    for(uint8_t i = 0; i < 10; i++) {
        cmtRxTime[i] = millis(); // set start time for all posible frames
        cmtRxFrag[i] = false; // reset frame received for all posible frames
    }
    cmtRecId = 0;
    cmtLastRecId = 0;
    cmtMaxFragId = 0;


    // prepare buf
    if(calcCrc16) {
        buf[len-3] = 0x00;
        buf[len-2] = 0x00;
        buf[len-1] = 0x00;
        uint16_t crc2 = crc16(&buf[10], len-13, 0xffff);
        buf[len-3] = (crc2 >> 8 & 0xff);
        buf[len-2] = (crc2      & 0xff);
    }
    if(calcCrc8)
        buf[len-1] = crc8(buf, len-1);

    dumpBuf("TX", buf, len, false);
    Serial.println(" | " + String(cmtGetActFreq()));

    RF_StartTx(buf, len, 500);
}

void cmtSwitchChannel(const uint8_t channel) {
    yield();
    CMT2300A_SetFrequencyChannel(channel);
    yield();
    cmtActualCh = channel;
    Serial.println("[cmtSwitchChannel] switched channel to " + cmtGetActFreq());
}

uint8_t cmtFreqToChan(const String func_name, const String var_name, uint32_t freq_kHz) {
    if((freq_kHz % 250) != 0) {
        Serial.println(func_name + " " + var_name + " " + String(freq_kHz/1000.0, 3) + " MHz is not divisible by 250 kHz!");
        return 0xFF; // ERROR
    }
    const uint32_t min_Freq_kHz = (HOY_BASE_FREQ + cmtBaseChOff860 * CMT2300A_ONE_STEP_SIZE * FH_OFFSET) / 1000; // frequency can not be lower than actual initailized base freq
    const uint32_t max_Freq_kHz = (HOY_BASE_FREQ + 0xFE * CMT2300A_ONE_STEP_SIZE * FH_OFFSET) / 1000; // =923500, 0xFF does not work
    if(freq_kHz < min_Freq_kHz || freq_kHz > max_Freq_kHz) {
        Serial.println(func_name + " " + var_name + " " + String(freq_kHz/1000.0, 2) + " MHz is out of range! (" + String(min_Freq_kHz/1000.0, 2) + " MHz - " + String(max_Freq_kHz/1000.0, 2) + " MHz)");
        return 0xFF; // ERROR
    }
    return (freq_kHz * 1000 - HOY_BASE_FREQ) / CMT2300A_ONE_STEP_SIZE / FH_OFFSET - cmtBaseChOff860; // frequency to channel
}

bool cmtSwitchDtuFreq(const uint32_t to_freq_kHz) {
    const uint8_t toChannel = cmtFreqToChan("[cmtSwitchDtuFreq]", "to_freq_kHz", to_freq_kHz);
    if(toChannel == 0xFF)
        return false;

    cmtSwitchChannel(toChannel);

    return true;
}

bool cmtSwitchInvAndDtuFreq(const uint32_t from_freq_kHz, const uint32_t to_freq_kHz, const uint64_t inv_serial, const uint32_t dtu_serial) {
    const uint8_t fromChannel = cmtFreqToChan("[cmtSwitchInvAndDtuFreq]", "from_freq_kHz", from_freq_kHz);
    const uint8_t toChannel = cmtFreqToChan("[cmtSwitchInvAndDtuFreq]", "to_freq_kHz", to_freq_kHz);
    if(fromChannel == 0xFF || toChannel == 0xFF)
        return false;

    cmtSwitchChannel(fromChannel);
    uint8_t cfg1[] = { // CMD56 for inverter frequency/channel switch
        0x56, U32_B3(inv_serial), U32_B2(inv_serial), U32_B1(inv_serial), U32_B0(inv_serial), U32_B3(dtu_serial), U32_B2(dtu_serial), U32_B1(dtu_serial), U32_B0(dtu_serial), 
        0x02, 0x15, 0x21, (uint8_t)(cmtBaseChOff860 + toChannel), 0x14, 0x00
    };
    cmtTxData(cfg1, sizeof(cfg1), false, true);
    cmtSwitchChannel(toChannel);

    return true;
}

bool cmtFirstDataReceived(void) {
    return cmtFirstRx;
}
