//-----------------------------------------------------------------------------
// 2022 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include "Arduino.h"
#include "lib/EasyCMT2300.h"

extern bool isCMT2300ARecived;
extern uint8_t TPMSpkBuf[128];
extern uint8_t TPMSpklength; 

#define CRC8_INIT                       0x00
#define CRC8_POLY                       0x01

#define DEF_SCK_PIN			32
#define DEF_SDIO_PIN		12
#define DEF_CS_PIN			27
#define DEF_FCS_PIN			25
#define DEF_GPIO1_PIN		34  //interupt pin

// config
uint32_t ts  = 0x63BFDBE1; // timestamp
uint32_t dtu = 0x83266790;
uint32_t wr  = 0x80724087;  // me - 116480724087
            // 0x80423810   // lumapu
            
// end config

#define UINT32_BYTE_3(val) ((uint8_t)((val >> 24) & 0xff))
#define UINT32_BYTE_2(val) ((uint8_t)((val >> 16) & 0xff))
#define UINT32_BYTE_1(val) ((uint8_t)((val >>  8) & 0xff))
#define UINT32_BYTE_0(val) ((uint8_t)((val      ) & 0xff))

void dumpBuf(const char* des, uint8_t buf[], uint8_t len) {
    Serial.println("------------------");
    Serial.print(String(des));
    for(uint8_t i = 0; i < len; i++) {
        if((0 != i) && (i % 8 == 0))
            Serial.print("");
        Serial.print(String(buf[i], HEX) + " ");
    }
    Serial.println("");
}
uint8_t crc8(uint8_t buf[], uint8_t len) {
    uint8_t crc = CRC8_INIT;
    for(uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for(uint8_t b = 0; b < 8; b ++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRC8_POLY : 0x00);
        }
    }
    return crc;
}
/*! ********************************************************
* @name    setup
* @desc    Initialize the whole libary
* *********************************************************/
void setup() {
    delay(10);
    Serial.begin(115200);
    Serial.println("Load lib for CMT2300A");

    if(CMT2300A_Int())
        Serial.println("CMT2300A int ok.");
    else
        Serial.println("CMT2300A not init!");
    
    /* Set the channel width for fast
    manual frequency hopping. Each bit
    increases by about 2.5kHz, and the
    maximum channel width is 2.5x255
    = 637.5 kHz */
    CMT2300A_SetFrequencyStep(0x64); // from Lumapu log it is on time 0x64

    if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY)) 
        Serial.println("standby mode reached");


    uint8_t cfg0[11] = { 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };
    uint8_t cfg1[15] = { 0x56, UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), 0x02, 0x15, 0x21, 0x0f, 0x14, 0xff};
    uint8_t cfg2[15] = { 0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x0f, 0x14, 0xff};
    uint8_t cfg3[11] = { 0x07, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), 0x00, 0xff};
    uint8_t cfg4[15] = { 0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x21, 0x14, 0xff};
    uint8_t i;

    /*cfg0[10] = crc8(cfg0, 10);
    CMT2300A_WriteReg(cfg0, 11);
    dumpBuf("cfg0", cfg0, 11);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 5; i++) {
        CMT2300A_WriteReg(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    CMT2300A_WriteReg(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg3[10] = crc8(cfg3, 10);
    CMT2300A_WriteReg(cfg3, 11);
    dumpBuf("cfg3", cfg3, 11);

    cfg4[14] = crc8(cfg4, 14);
    CMT2300A_WriteReg(cfg4, 15);
    dumpBuf("cfg4", cfg4, 15);

    cfg2[14] = crc8(cfg2, 14);
    CMT2300A_WriteReg(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 3; i++) {
        CMT2300A_WriteReg(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    CMT2300A_WriteReg(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);*/
}

uint8_t time_s = 1;
void loop() {
    delay(time_s * 1000);
    ts += time_s;
    
    if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY)) {
        Serial.println("STBY");

        uint8_t rqst[29] = { 0x7E, 0x15,
        UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), 
        UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), 
        0x80, 0x0B, 0x00, 
        UINT32_BYTE_3(ts), UINT32_BYTE_2(ts), UINT32_BYTE_1(ts), UINT32_BYTE_0(ts), 
        0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x88, 0x90, 0xff, 0x7F
        };

        rqst[sizeof(rqst)-2] = crc8(rqst, sizeof(rqst)-2);
        CMT2300A_WriteFifo(rqst, sizeof(rqst));
        CMT2300A_EnableWriteFifo();

        if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_TX))
        {
            dumpBuf("tx: ", rqst, sizeof(rqst));
        }
    }

    if(isCMT2300ARecived)
    {
        isCMT2300ARecived=false;
        Serial.println("rx packets?");
        Serial.printf("%c", TPMSpkBuf);
    }

    if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_RX))
    {
        Serial.print("CMT: ");
        Serial.println(CMT2300A_GetRssiDBm());
    }
    
    CMT2300A_FastFreqSwitch();
}
