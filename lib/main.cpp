//-----------------------------------------------------------------------------
// 2022 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include "Arduino.h"
#include "lib/EasyCMT2300.h"
#include "utils/crc.h"

extern bool isCMT2300ARecived;
extern uint8_t TPMSpkBuf[128];
extern uint8_t TPMSpklength; 

#define DEF_SCK_PIN			32
#define DEF_SDIO_PIN		12
#define DEF_CS_PIN			27
#define DEF_FCS_PIN			25
#define DEF_GPIO1_PIN		34  //interupt pin

uint16_t cnt;
uint8_t mLastFreq = 0;

bool RX_PKT = false;

// config
uint32_t ts  = 1675269384; // timestamp
uint32_t dtu = 0x80423810;
uint32_t wr  = 0x80724087;  // me - 1164 80724087
            // 0x80423810   // lumapu
            
// end config

#define U32_B3(val) ((uint8_t)((val >> 24) & 0xff))
#define U32_B2(val) ((uint8_t)((val >> 16) & 0xff))
#define U32_B1(val) ((uint8_t)((val >>  8) & 0xff))
#define U32_B0(val) ((uint8_t)((val      ) & 0xff))

void dumpBuf(const char* des, uint8_t buf[], uint8_t len) {
    char dataString[50] = {0};

    Serial.println("------------------");
    Serial.print(String(des));
    for(uint8_t i = 0; i < len; i++) {
        if((0 != i) && (i % 8 == 0))
            Serial.print("");
        sprintf(dataString,"%02X ",buf[i]);    
        Serial.print(dataString);
    }
    Serial.println("");
}




/*! ********************************************************
* @name    setup
* @desc    Initialize the whole libary
* *********************************************************/
void setup() {
    Serial.begin(115200);
    cnt = 0;
    Serial.println("Load lib for CMT2300A");

    while(!CMT2300A_Int())
    {
        Serial.println("CMT2300A not init!");
        delay(1000);
    }
    
    Serial.println("CMT2300A int ok.");
        
    
    /* Set the channel width for fast
    manual frequency hopping. Each bit
    increases by about 2.5kHz, and the
    maximum channel width is 2.5x255
    = 637.5 kHz */
    CMT2300A_SetFrequencyStep(0x64); // from Lumapu log it is on time 0x64

    if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY)) 
        Serial.println("standby mode reached");
}

uint8_t time_s = 1;
String arr[] = {"PUP", "SLEEP", "STBY", "RFS", "TFS", "RX", "TX", "EEPROM", "ERROR", "CAL"};

/* RealTimeRunData_Debug 0x0B HMS-2000:

TX 15 80423810 81001765 80 0B00 6384 DE99 0000 0000 0000 0000 B63A AB
   ^^----------------------------------------------------------------- MainCmd 0x15 REQ_ARW_DAT_ALL
      ^^^^^^^^-------------------------------------------------------- WR Serial ID
               ^^^^^^^^----------------------------------------------- DTU Serial ID
                        ^^-------------------------------------------- MultiFrameID 0x80 = SingleFrame
                           ^^----------------------------------------- SubCmd bzw. DataType: 0x0B = RealTimeRunData_Debug, 0x0C RealTimeRunData_Reality
                             ^^--------------------------------------- rev Protocol Revision ?
                           ^^^^--------------------------------------- Control Mode ? immer zwei Byte im Gen3 Protokoll
                                ^^^^ ^^^^----------------------------- UNIX timestamp
                                          ^^^^------------------------ Gap always 0x0000
                                               ^^^^------------------- 0x0000, nur bei AlarmData: WarnSerNub (Warning Serial Number)
                                                    ^^^^ ^^^^--------  Password always 0x00000000
                                                              ^^^^---- CRC16 / CRC-Modbus über die UserData, excl. Frame ID!
                                                                   ^^- CRC8 
*/

/* TODO: SOF, EOF und die Escape Sequenz (7D) selbst escapen
7D => 7D 5D
7E => 7D 5E
7F => 7D 5F
*/

void loop() {
    CMT2300A_FastFreqSwitch();
    delay(time_s * 5000);
    ts += time_s;
    
    if(CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY)) {
        CMT2300A_goTX();

        Serial.print("ChipStatus: "); Serial.println(arr[CMT2300A_GetChipStatus()]);
        Serial.print("RSSI: "); Serial.println(CMT2300A_GetRssiDBm());

        uint8_t rqst[] = { 0x15,                              // MainCmd 0x15 REQ_ARW_DAT_ALL
            U32_B3(wr), U32_B2(wr), U32_B1(wr), U32_B0(wr),     // WR Serial ID
            U32_B3(dtu), U32_B2(dtu), U32_B1(dtu), U32_B0(dtu),     // WR Serial ID
            0x80,                                                                           // MultiFrameID 0x80 = SingleFrame
            0x0B,                                                                           // SubCmd bzw. DataType: 0x0B = RealTimeRunData_Debug, 0x0C RealTimeRunData_Reality
            0x00,                                                                           // rev Protocol Revision ?
            U32_B3(ts), U32_B2(ts), U32_B1(ts), U32_B0(ts),     // UNIX timestamp
            0x00, 0x00, 0x00, 0x04, 
            0x00, 0x00, 0x00, 0x00,
            //
            0xFF, 0xCC,                                                                     // CRC16
            0x00                                                                            // CRC8
        };

        uint16_t crc = crc16(&rqst[10], sizeof(rqst)-13, 0xffff);
        rqst[24] = (crc >> 8);
        rqst[25] = crc & 0xff;

        rqst[sizeof(rqst)-1] = crc8(rqst, sizeof(rqst)-1);

        CMT2300A_WriteFifo(rqst, sizeof(rqst));

        dumpBuf("tx: ", rqst, sizeof(rqst));
    }

    if(isCMT2300ARecived) {
        isCMT2300ARecived=false;
        Serial.println("rx packets?");

        Serial.printf("%.*s",TPMSpklength,TPMSpkBuf);
        Serial.println();
        
        CMT2300A_goRX();
    }
}
