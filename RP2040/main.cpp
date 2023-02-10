//-----------------------------------------------------------------------------
// 2023 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include <Arduino.h>
#include "rp2040.h"
//#include "softSpi3w.h"

#if defined(ESP32)
    #define DEF_SCK_PIN         32
    #define DEF_SDIO_PIN        12
    #define DEF_CS_PIN          27
    #define DEF_FCS_PIN         25
    #define DEF_GPIO1_PIN       34 // interupt pin
#else // 8266
    #define DEF_CS_PIN          16 // D0 - GPIO16
    #define DEF_FCS_PIN         2  // D4 - GPIO2
    #define DEF_SCK_PIN         14 // D5 - GPIO14
    #define DEF_SDIO_PIN        12 // D6 - GPIO12
#endif

#ifdef RP2040
    #define CMT_GPIO3           6  // interupt pin
#endif

//-----------------------------------------------------------------------------
#define CMT2300A_MASK_CFG_RETAIN        0x10
#define CMT2300A_MASK_RSTN_IN_EN        0x20
#define CMT2300A_MASK_LOCKING_EN        0x20
#define CMT2300A_MASK_CHIP_MODE_STA     0x0F

//-----------------------------------------------------------------------------
// detailed register infos from AN142_CMT2300AW_Quick_Start_Guide-Rev0.8.pdf

#define CMT2300A_CUS_MODE_CTL           0x60    // [7] go_switch
                                                // [6] go_tx
                                                // [5] go_tfs
                                                // [4] go_sleep
                                                // [3] go_rx
                                                // [2] go_rfs
                                                // [1] go_stby
                                                // [0] n/a

#define CMT2300A_CUS_MODE_STA           0x61    // [3:0] 0x00 IDLE
                                                //       0x01 SLEEP
                                                //       0x02 STBY
                                                //       0x03 RFS
                                                //       0x04 TFS
                                                //       0x05 RX
                                                //       0x06 TX
                                                //       0x08 UNLOCKED/LOW_VDD
                                                //       0x09 CAL
#define CMT2300A_CUS_EN_CTL             0x62
#define CMT2300A_CUS_FREQ_CHNL          0x63

#define CMT2300A_CUS_IO_SEL             0x65    // [5:4] GPIO3
                                                //       0x00 CLKO
                                                //       0x01 DOUT / DIN
                                                //       0x02 INT2
                                                //       0x03 DCLK
                                                // [3:2]  GPIO2
                                                //       0x00 INT1
                                                //       0x01 INT2
                                                //       0x02 DOUT / DIN
                                                //       0x03 DCLK
                                                // [1:0]  GPIO1
                                                //       0x00 DOUT / DIN
                                                //       0x01 INT1
                                                //       0x02 INT2
                                                //       0x03 DCLK

#define CMT2300A_CUS_INT1_CTL           0x66    // [4:0] INT1_SEL
                                                //       0x00 RX active
                                                //       0x01 TX active
                                                //       0x02 RSSI VLD
                                                //       0x03 Pream OK
                                                //       0x04 SYNC OK
                                                //       0x05 NODE OK
                                                //       0x06 CRC OK
                                                //       0x07 PKT OK
                                                //       0x08 SL TMO
                                                //       0x09 RX TMO
                                                //       0x0A TX DONE
                                                //       0x0B RX FIFO NMTY
                                                //       0x0C RX FIFO TH
                                                //       0x0D RX FIFO FULL
                                                //       0x0E RX FIFO WBYTE
                                                //       0x0F RX FIFO OVF
                                                //       0x10 TX FIFO NMTY
                                                //       0x11 TX FIFO TH
                                                //       0x12 TX FIFO FULL
                                                //       0x13 STATE IS STBY
                                                //       0x14 STATE IS FS
                                                //       0x15 STATE IS RX
                                                //       0x16 STATE IS TX
                                                //       0x17 LED
                                                //       0x18 TRX ACTIVE
                                                //       0x19 PKT DONE

#define CMT2300A_CUS_INT2_CTL           0x67    // [4:0] INT2_SEL

#define CMT2300A_CUS_INT_EN             0x68    // [7] SL TMO EN
                                                // [6] RX TMO EN
                                                // [5] TX DONE EN
                                                // [4] PREAM OK EN
                                                // [3] SYNC_OK EN
                                                // [2] NODE OK EN
                                                // [1] CRC OK EN
                                                // [0] PKT DONE EN

#define CMT2300A_CUS_FIFO_CTL           0x69    // [7] TX DIN EN
                                                // [6:5] TX DIN SEL
                                                //     0x00 SEL GPIO1
                                                //     0x01 SEL GPIO2
                                                //     0x02 SEL GPIO3
                                                // [4] FIFO AUTO CLR DIS
                                                // [3] FIFO TX RD EN
                                                // [2] FIFO RX TX SEL
                                                // [1] FIFO MERGE EN
                                                // [0] SPI FIFO RD WR SEL

#define CMT2300A_CUS_INT_CLR1           0x6A // clear interrupts Bank1
#define CMT2300A_CUS_INT_CLR2           0x6B // clear interrupts Bank2
#define CMT2300A_CUS_FIFO_CLR           0x6C

#define CMT2300A_CUS_INT_FLAG           0x6D    // [7] LBD FLG
                                                // [6] COL ERR FLG
                                                // [5] PKT ERR FLG
                                                // [4] PREAM OK FLG
                                                // [3] SYNC OK FLG
                                                // [2] NODE OK FLG
                                                // [1] CRC OK FLG
                                                // [0] PKT OK FLG

#define CMT2300A_CUS_RSSI_DBM           0x70

#define CMT2300A_GO_SWITCH              0x80
#define CMT2300A_GO_TX                  0x40
#define CMT2300A_GO_TFS                 0x20
#define CMT2300A_GO_SLEEP               0x10
#define CMT2300A_GO_RX                  0x08
#define CMT2300A_GO_RFS                 0x04
#define CMT2300A_GO_STBY                0x02
#define CMT2300A_GO_EEPROM              0x01

#define CMT2300A_STA_IDLE               0x00
#define CMT2300A_STA_SLEEP              0x01
#define CMT2300A_STA_STBY               0x02
#define CMT2300A_STA_RFS                0x03
#define CMT2300A_STA_TFS                0x04
#define CMT2300A_STA_RX                 0x05
#define CMT2300A_STA_TX                 0x06
#define CMT2300A_STA_EEPROM             0x07
#define CMT2300A_STA_ERROR              0x08
#define CMT2300A_STA_CAL                0x09

#define CMT2300A_INT_SEL_TX_DONE        0x0A

#define CMT2300A_MASK_TX_DONE_FLG       0x08
#define CMT2300A_MASK_PKT_OK_FLG        0x01

#define CRC8_INIT                       0x00
#define CRC8_POLY                       0x01

#define CRC16_MODBUS_POLYNOM            0xA001

#define U32_B3(val) ((uint8_t)((val >> 24) & 0xff))
#define U32_B2(val) ((uint8_t)((val >> 16) & 0xff))
#define U32_B1(val) ((uint8_t)((val >>  8) & 0xff))
#define U32_B0(val) ((uint8_t)((val      ) & 0xff))

#ifdef RP2040
    typedef RP2040Spi3w SpiType;
#else
    typedef SoftSpi3w<DEF_CS_PIN, DEF_FCS_PIN, DEF_SCK_PIN, DEF_SDIO_PIN> SpiType;
#endif

SpiType spi3w;
uint32_t cnt;

uint8_t mLastFreq;
uint8_t mStartFreq;
uint8_t mEndFreq;
bool mFound;
uint32_t mPrevMillis;
bool mTxVerify;

// config
uint32_t ts  = 1675597502; //0x6384DF00; // timestamp
uint32_t dtu = 0x83266790; //0x81001756; // ;
uint32_t wr  = 0x80423810;
// end config

void rxLoop(void);

uint8_t getChipStatus(void) {
    return spi3w.readReg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
}

//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
uint16_t crc16(uint8_t buf[], uint8_t len, uint16_t start) {
    uint16_t crc = start;
    uint8_t shift = 0;

    for(uint8_t i = 0; i < len; i ++) {
        crc = crc ^ buf[i];
        for(uint8_t bit = 0; bit < 8; bit ++) {
            shift = (crc & 0x0001);
            crc = crc >> 1;
            if(shift != 0)
                crc = crc ^ CRC16_MODBUS_POLYNOM;
        }
    }
    return crc;
}

//-----------------------------------------------------------------------------
void dumpBuf(const char* des, uint8_t buf[], uint8_t len) {
    Serial.println("------------------");
    Serial.println(String(des));
    for(uint8_t i = 0; i < len; i++) {
        if((0 != i) && (i % 8 == 0))
            Serial.println("");
        Serial.print(String(buf[i], HEX) + " ");
    }
    Serial.println("");
}

//-----------------------------------------------------------------------------
bool cmtSwitchStatus(uint8_t cmd, uint8_t waitFor, uint16_t cycles = 50000) {
    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, cmd);
    while(cycles--) {
        delayMicroseconds(10);
        if(waitFor == getChipStatus())
            return true;
    }
    Serial.println("status wait for: " + String(waitFor, HEX) + " read: " + String(getChipStatus(), HEX));
    return false;
}

//-----------------------------------------------------------------------------
void freqSetting(uint8_t setting, bool send7A = true) {
    spi3w.writeReg(0x18, 0x42);
    spi3w.writeReg(0x19, 0x6D);
    spi3w.writeReg(0x1A, 0x80);
    spi3w.writeReg(0x1B, 0x86);
    spi3w.writeReg(0x1C, 0x42);
    spi3w.writeReg(0x1D, 0x62);
    spi3w.writeReg(0x1E, 0x27);
    spi3w.writeReg(0x1F, 0x16);

    /*if(0 == setting) { // 871MHz
        spi3w.writeReg(0x18, 0x42);
        spi3w.writeReg(0x19, 0xA9);
        spi3w.writeReg(0x1A, 0xA4);
        spi3w.writeReg(0x1B, 0x8C);
        spi3w.writeReg(0x1C, 0x42);
        spi3w.writeReg(0x1D, 0x9E);
        spi3w.writeReg(0x1E, 0x4B);
        spi3w.writeReg(0x1F, 0x1C);
    } else if(1 == setting) { // 872MHz
        spi3w.writeReg(0x18, 0x42);
        spi3w.writeReg(0x19, 0x0b);
        spi3w.writeReg(0x1A, 0xcc);
        spi3w.writeReg(0x1B, 0x82);
        spi3w.writeReg(0x1C, 0x42);
        spi3w.writeReg(0x1D, 0x00);
        spi3w.writeReg(0x1E, 0x73);
        spi3w.writeReg(0x1F, 0x12);
    }*/
    if(send7A)
        spi3w.writeReg(0x27, 0x0A);
}

//-----------------------------------------------------------------------------
void switchFreq(void) {
    if(++mLastFreq > mEndFreq)
        mLastFreq = mStartFreq;

    //mLastFreq = 0x00;

    // 0: 868.00MHz
    // 1: 868.23MHz
    // 2: 868.46MHz
    // 3: 868.72MHz
    // 4: 868.97MHz
    spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, mLastFreq);
}

//-----------------------------------------------------------------------------
int8_t checkRx() {
    int8_t rssi = -128;
    /*if(CMT2300A_INT_SEL_TX_DONE != spi3w.readReg(CMT2300A_CUS_INT1_CTL))
        spi3w.writeReg(CMT2300A_CUS_INT1_CTL, CMT2300A_INT_SEL_TX_DONE);*/
    spi3w.readReg(CMT2300A_CUS_INT1_CTL);
    spi3w.writeReg(CMT2300A_CUS_INT1_CTL, CMT2300A_INT_SEL_TX_DONE);

    //if(0x00 == spi3w.readReg(CMT2300A_CUS_INT_FLAG))
    spi3w.readReg(CMT2300A_CUS_INT_FLAG);
    {
        // no data received
        uint8_t tmp = spi3w.readReg(CMT2300A_CUS_INT_CLR1);
        if(0x08 == tmp) // first time after TX this reg is 0x08
            spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x04);
        else
            spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);

        if(0x10 == tmp)
            spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x10);
        else
            spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);

        spi3w.readReg(CMT2300A_CUS_FIFO_CTL); // necessary? -> if 0x02 last was read
                                              //                  0x07 last was write
        spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x02);

        spi3w.writeReg(CMT2300A_CUS_FIFO_CLR, 0x02);
        spi3w.writeReg(0x16, 0x0C); // [4:3]: RSSI_DET_SEL, [2:0]: RSSI_AVG_MODE

        switchFreq();

        if(!cmtSwitchStatus(CMT2300A_GO_RX, CMT2300A_STA_RX))
            Serial.println("warn: cant reach RX mode!");

        uint8_t state = 0x00;
        for(uint8_t i = 0; i < 52; i++) {
            state = spi3w.readReg(CMT2300A_CUS_INT_FLAG);
            if(0x00 != state)
                break;
        }

        #ifdef RP2040
        if((state & 0x10) == 0x10) {
            uint16_t timeout = 5000;
            while(0 == gpio_get(CMT_GPIO3)) {
                usleep(10);
                if(0 == --timeout) {
                    //Serial.println("GPIO timeout");
                    break;
                }
            }
            if(0 != timeout) {
                uint16_t timeout2 = 5000;
                while(0x18 != (state & 0x18)) {
                    state = spi3w.readReg(CMT2300A_CUS_INT_FLAG);
                    if(0 == timeout2--) {
                        //Serial.println("state timeout");
                        break;
                    }
                }
            }
        }
        #endif


        if(0 != gpio_get(CMT_GPIO3)) {
            uint32_t loops = 0;
            while((state & 0x1b) != 0x1b) {
                state = spi3w.readReg(CMT2300A_CUS_INT_FLAG);

                //if(state == 0x18)
                //    delay(23);

                if((state & 0x20) == 0x20) {
                    Serial.println("receive PKT ERROR");
                    break;
                }
                else if((state & 0x40) == 0x40) {
                    Serial.println("receive COL ERROR");
                    break;
                }

                if(++loops > 5000) {
                    //Serial.println("receive error, loops, state: " + String(state, HEX));
                    break;
                }
            }
        }


        // receive ok (pream, sync, node, crc)
        uint8_t buf[27] = {0};
        if((state & 0x1b) == 0x1b) {
            if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
                Serial.println("warn: not switched to standby mode!");

            spi3w.readFifo(buf, 27);

            rssi = spi3w.readReg(CMT2300A_CUS_RSSI_DBM) - 128;

            if(!cmtSwitchStatus(CMT2300A_GO_SLEEP, CMT2300A_STA_SLEEP))
                Serial.println("warn: not switched to sleep mode!");
        }

        //if(0 != timeout)
        //    Serial.println("RX DONE!!! RSSI: " + String(rssi) + " remain cnt: " + String(timeout));

        /*if((state & 0x1b) == 0x1b) {
            // only print buffer if one field is not equal 0
            Serial.println("mLastFreq: " + String(mLastFreq, HEX));
            for(uint8_t i = 0; i < 27; i++) {
                if(buf[i] != 0) {
                    dumpBuf("RX", buf, 27);
                    mFound = true;
                    break;
                }
            }
        }*/
    }


    if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("warn: not switched to standby mode!");

    return rssi;
}

//-----------------------------------------------------------------------------
void txData(uint8_t buf[], uint8_t len, bool calcCrc16 = true, bool calcCrc8 = true, bool verify = false) {
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
    //dumpBuf("TX", buf, len);


    // verify that write pipe is empty
    if(verify == true) {
        spi3w.writeReg(0x16, 0x04); // [4:3]: RSSI_DET_SEL, [2:0]: RSSI_AVG_MODE
        spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, 0x00);

        if(!cmtSwitchStatus(CMT2300A_GO_RX, CMT2300A_STA_RX))
            Serial.println("warn: cant reach RX mode!");

        for(uint8_t i = 0; i < 50; i++) {
            spi3w.readReg(CMT2300A_CUS_RSSI_DBM);
            spi3w.readReg(CMT2300A_CUS_INT1_CTL);
            if(0x00 != spi3w.readReg(CMT2300A_CUS_INT_FLAG)) {
                Serial.println("TX: CMT2300A_CUS_INT_FLAG is not 0!: " + String(spi3w.readReg(CMT2300A_CUS_INT_FLAG), HEX));

                spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);
                spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);
                return;
            }
            spi3w.readReg(CMT2300A_CUS_INT_CLR1);
            spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);
            spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);
        }

        if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
            Serial.println("warn: not switched to standby mode!");
        // verify end
    }

    //if(CMT2300A_INT_SEL_TX_DONE != spi3w.readReg(CMT2300A_CUS_INT1_CTL))
    //    spi3w.writeReg(CMT2300A_CUS_INT1_CTL, CMT2300A_INT_SEL_TX_DONE);
    spi3w.readReg(CMT2300A_CUS_INT1_CTL);
    spi3w.writeReg(CMT2300A_CUS_INT1_CTL, CMT2300A_INT_SEL_TX_DONE);

    //if(0x00 == spi3w.readReg(CMT2300A_CUS_INT_FLAG))
    {
        // no data received
        spi3w.readReg(CMT2300A_CUS_INT_CLR1);
        spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);
        spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);

        spi3w.readReg(CMT2300A_CUS_FIFO_CTL); // necessary?
        spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x07);

        spi3w.writeReg(CMT2300A_CUS_FIFO_CLR, 0x01);

        if(0x01 != spi3w.readReg(0x45))
            Serial.println("reg 0x45 must be 0x01!");
        spi3w.writeReg(0x45, 0x01);
        spi3w.writeReg(0x46, len); // payload length

        spi3w.writeFifo(buf, len);

        //spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, mLastFreq);
        spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, 0x00);

        if(!cmtSwitchStatus(CMT2300A_GO_TX, CMT2300A_STA_TX))
            Serial.println("warn: cant reach TX mode!");

        // wait for tx done
        while(CMT2300A_MASK_TX_DONE_FLG != spi3w.readReg(CMT2300A_CUS_INT_CLR1)) {
            yield();
            //sleep_us(10);
        }

        delay(1);

        if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
            Serial.println("warn: not switched to standby mode!");
    }
}

//-----------------------------------------------------------------------------
void resetCMT(void) {
    spi3w.writeReg(0x7f, 0xff); // soft reset
    delay(30);

    // go to standby mode
    if(cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("standby mode reached");

    if(0xAA != spi3w.readReg(0x48))
        spi3w.writeReg(0x48, 0xAA);
    spi3w.readReg(0x48);
    spi3w.writeReg(0x4c, 0x00);

    if(0x52 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) // 0x62
        spi3w.writeReg(CMT2300A_CUS_MODE_STA, 0x52);
    if(0x20 != spi3w.readReg(0x62))
        spi3w.writeReg(0x62, 0x20);
    spi3w.readReg(0x0D);
    spi3w.writeReg(0x0F, 0x00);


    ///////////////////

    spi3w.writeReg(0x00, 0x00);
    spi3w.writeReg(0x01, 0x66);
    spi3w.writeReg(0x02, 0xEC);
    spi3w.writeReg(0x03, 0x1D);
    spi3w.writeReg(0x04, 0x70);
    spi3w.writeReg(0x05, 0x80);
    spi3w.writeReg(0x06, 0x14);
    spi3w.writeReg(0x07, 0x08);
    spi3w.writeReg(0x08, 0x91);
    spi3w.writeReg(0x09, 0x02);
    spi3w.writeReg(0x0A, 0x02);
    spi3w.writeReg(0x0B, 0xD0);
    spi3w.writeReg(0x0C, 0xAE);
    spi3w.writeReg(0x0D, 0xE0);
    spi3w.writeReg(0x0E, 0x35);
    spi3w.writeReg(0x0F, 0x00);

    spi3w.writeReg(0x10, 0x00);
    spi3w.writeReg(0x11, 0xF4);
    spi3w.writeReg(0x12, 0x10);
    spi3w.writeReg(0x13, 0xE2);
    spi3w.writeReg(0x14, 0x42);
    spi3w.writeReg(0x15, 0x20);
    spi3w.writeReg(0x16, 0x0C); // [4:3]: RSSI_DET_SEL, [2:0]: RSSI_AVG_MODE
    spi3w.writeReg(0x17, 0x81);
    spi3w.writeReg(0x18, 0x42);
    spi3w.writeReg(0x19, 0xCF);
    spi3w.writeReg(0x1A, 0xA7);
    spi3w.writeReg(0x1B, 0x8C);
    spi3w.writeReg(0x1C, 0x42);
    spi3w.writeReg(0x1D, 0xC4);
    spi3w.writeReg(0x1E, 0x4E);
    spi3w.writeReg(0x1F, 0x1C);

    spi3w.writeReg(0x20, 0xA6);
    spi3w.writeReg(0x21, 0xC9);
    spi3w.writeReg(0x22, 0x20);
    spi3w.writeReg(0x23, 0x20);
    spi3w.writeReg(0x24, 0xD2);
    spi3w.writeReg(0x25, 0x35);
    spi3w.writeReg(0x26, 0x0C);
    spi3w.writeReg(0x27, 0x0A);
    spi3w.writeReg(0x28, 0x9F);
    spi3w.writeReg(0x29, 0x4B);
    spi3w.writeReg(0x2A, 0x29);
    spi3w.writeReg(0x2B, 0x29);
    spi3w.writeReg(0x2C, 0xC0);
    spi3w.writeReg(0x2D, 0x14);
    spi3w.writeReg(0x2E, 0x05);
    spi3w.writeReg(0x2F, 0x53);

    spi3w.writeReg(0x30, 0x10);
    spi3w.writeReg(0x31, 0x00);
    spi3w.writeReg(0x32, 0xB4);
    spi3w.writeReg(0x33, 0x00);
    spi3w.writeReg(0x34, 0x00);
    spi3w.writeReg(0x35, 0x01);
    spi3w.writeReg(0x36, 0x00);
    spi3w.writeReg(0x37, 0x00);
    spi3w.writeReg(0x38, 0x12);
    spi3w.writeReg(0x39, 0x1E);
    spi3w.writeReg(0x3A, 0x00);
    spi3w.writeReg(0x3B, 0xAA);
    spi3w.writeReg(0x3C, 0x06);
    spi3w.writeReg(0x3D, 0x00);
    spi3w.writeReg(0x3E, 0x00);
    spi3w.writeReg(0x3F, 0x00);

    spi3w.writeReg(0x40, 0x00);
    spi3w.writeReg(0x41, 0xD6);
    spi3w.writeReg(0x42, 0xD5);
    spi3w.writeReg(0x43, 0xD4);
    spi3w.writeReg(0x44, 0x2D);
    spi3w.writeReg(0x45, 0x01);
    spi3w.writeReg(0x46, 0x1D);
    spi3w.writeReg(0x47, 0x00);
    spi3w.writeReg(0x48, 0x00);
    spi3w.writeReg(0x49, 0x00);
    spi3w.writeReg(0x4A, 0x00);
    spi3w.writeReg(0x4B, 0x00);
    spi3w.writeReg(0x4C, 0xC3);
    spi3w.writeReg(0x4D, 0x00);
    spi3w.writeReg(0x4E, 0x00);
    spi3w.writeReg(0x4F, 0x60);

    spi3w.writeReg(0x50, 0xFF);
    spi3w.writeReg(0x51, 0x00);
    spi3w.writeReg(0x52, 0x00);
    spi3w.writeReg(0x53, 0x1F);
    spi3w.writeReg(0x54, 0x10);
    spi3w.writeReg(0x55, 0x70);
    spi3w.writeReg(0x56, 0x4D);
    spi3w.writeReg(0x57, 0x06);
    spi3w.writeReg(0x58, 0x00);
    spi3w.writeReg(0x59, 0x07);
    spi3w.writeReg(0x5A, 0x50);
    spi3w.writeReg(0x5B, 0x00);
    spi3w.writeReg(0x5C, 0x8A);
    spi3w.writeReg(0x5D, 0x18);
    spi3w.writeReg(0x5E, 0x3F);
    spi3w.writeReg(0x5F, 0x7F);

    if(0x02 != spi3w.readReg(0x09))
        spi3w.writeReg(0x09, 0x02);

    spi3w.writeReg(CMT2300A_CUS_IO_SEL, 0x20); // -> GPIO3_SEL[1:0] = 0x02

    // interrupt 1 control selection to TX DONE
    if(CMT2300A_INT_SEL_TX_DONE != spi3w.readReg(CMT2300A_CUS_INT1_CTL))
        spi3w.writeReg(CMT2300A_CUS_INT1_CTL, CMT2300A_INT_SEL_TX_DONE);

    // select interrupt 2
    if(0x07 != spi3w.readReg(CMT2300A_CUS_INT2_CTL))
        spi3w.writeReg(CMT2300A_CUS_INT2_CTL, 0x07);

    // interrupt enable (TX_DONE, PREAM_OK, SYNC_OK, CRC_OK, PKT_DONE)
    spi3w.writeReg(CMT2300A_CUS_INT_EN, 0x3B);

    spi3w.writeReg(0x41, 0x48);
    spi3w.writeReg(0x42, 0x5A);
    spi3w.writeReg(0x43, 0x48);
    spi3w.writeReg(0x44, 0x4D);
    spi3w.writeReg(0x64, 0x64);

    if(0x00 == spi3w.readReg(CMT2300A_CUS_FIFO_CTL))
        spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x02); // FIFO_MERGE_EN

    if(!cmtSwitchStatus(CMT2300A_GO_SLEEP, CMT2300A_STA_SLEEP))
        Serial.println("warn: not switched to sleep mode!");

    sleep_us(95);

    if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("warn: not switched to standby mode!");

    freqSetting(0, false);
    spi3w.writeReg(0x22, 0x20);
    spi3w.writeReg(0x23, 0x20);
    spi3w.writeReg(0x24, 0xD2);
    spi3w.writeReg(0x25, 0x35);
    spi3w.writeReg(0x26, 0x0C);
    spi3w.writeReg(0x27, 0x0A);
    spi3w.writeReg(0x28, 0x9F);
    spi3w.writeReg(0x29, 0x4B);
    spi3w.writeReg(0x27, 0x0A);

    if(!cmtSwitchStatus(CMT2300A_GO_SLEEP, CMT2300A_STA_SLEEP))
        Serial.println("warn: not switched to sleep mode!");

    if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("warn: not switched to standby mode!");

    spi3w.writeReg(0x03, 0x1D);
    spi3w.writeReg(0x5C, 0x8A);
    spi3w.writeReg(0x5D, 0x18);

    if(!cmtSwitchStatus(CMT2300A_GO_SLEEP, CMT2300A_STA_SLEEP))
        Serial.println("warn: not switched to sleep mode!");

    if(!cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("warn: not switched to standby mode!");

    mTxVerify = true;
}

//-----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    cnt        = 0;
    mStartFreq = 0x00;
    mEndFreq   = 0x0F;
    mLastFreq  = mStartFreq;
    mFound     = false;
    spi3w.setup();

    #ifdef RP2040
    gpio_init(CMT_GPIO3);
    gpio_set_dir(CMT_GPIO3, false); // INPUT
    #endif

    delay(1000);
    Serial.println("start");

    spi3w.writeReg(0x00, 0xff);
    delay(20);

    resetCMT();

    mPrevMillis = millis();
}

//-----------------------------------------------------------------------------
/*void rxLoop(void) {
    int8_t rssi = rxInit();
    int8_t rssiNew = rssi;
    for(uint8_t i = 0; i < 100; i++) {
        rssiNew = checkRx();
        if(rssiNew != rssi) {
            if(-128 != rssiNew) {
                rssi = rssiNew;
                Serial.println("RSSI: " + String(rssi) + " Freq: " + mLastFreq);
            }
        }
    }
}*/

//-----------------------------------------------------------------------------
void loop() {
    if ((millis() - mPrevMillis) > 1000) {
        mPrevMillis = millis();
        cnt++;

        Serial.println("#" + String(cnt) + ": ");
        if(!mFound) {
            if(((cnt % 10) == 0) || (cnt < 28)) {
                uint8_t cfg1[15] = {
                    0x56, U32_B3(wr), U32_B2(wr), U32_B1(wr), U32_B0(wr), U32_B3(dtu), U32_B2(dtu), U32_B1(dtu),
                    U32_B0(dtu), 0x02, 0x15, 0x21, 0x0f, 0x14, 0x62
                };
                txData(cfg1, 15, false, true, mTxVerify);
                checkRx();
                checkRx();
                checkRx();
                txData(cfg1, 15, false, true, mTxVerify);
                mTxVerify = false;
            }
        }

        if(cnt >= 28) {
            if((!mFound) && ((cnt % 10) == 0))
                return;

            uint8_t rqst[27] = {
                0x15, U32_B3(wr), U32_B2(wr), U32_B1(wr), U32_B0(wr), U32_B3(dtu), U32_B2(dtu), U32_B1(dtu),
                U32_B0(dtu), 0x80, 0x0B, 0x00, U32_B3(ts), U32_B2(ts), U32_B1(ts), U32_B0(ts),
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00
            };
            txData(rqst, 27);
            checkRx();
            checkRx();
            checkRx();
            txData(rqst, 27);
            ts++;
        }

        /*if(!mFound) {
            //if(cnt == 30)
            //    freqSetting(1);

            if(cnt % 10 == 0) {
                if(cnt % 5 == 0) {
                    mStartFreq = 0x0E;
                    mEndFreq = 0x10;
                }
                else {
                    mStartFreq = 0x20;
                    mEndFreq = 0x22;
                }
            }
        }*/
    }
    if(cnt == 5)
        resetCMT();
    checkRx();
}
