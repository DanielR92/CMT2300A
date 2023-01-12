//-----------------------------------------------------------------------------
// 2022 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include <Arduino.h>
//#include "SPI.h"
#include "softSpi3w.h"

#define DEF_CS_PIN                      16 // D0 - GPIO16
#define DEF_FCS_PIN                     2  // D4 - GPIO2
#define DEF_SCK_PIN                     14 // D5 - GPIO14
#define DEF_SDIO_PIN                    12 // D6 - GPIO12

#define CMT2300A_MASK_CFG_RETAIN        0x10
#define CMT2300A_MASK_RSTN_IN_EN        0x20
#define CMT2300A_MASK_LOCKING_EN        0x20
#define CMT2300A_MASK_CHIP_MODE_STA     0x0F

#define CMT2300A_CUS_MODE_CTL           0x60
#define CMT2300A_CUS_MODE_STA           0x61
#define CMT2300A_CUS_EN_CTL             0x62
#define CMT2300A_CUS_INT_CLR1           0x6A
#define CMT2300A_CUS_FIFO_CLR           0x6C
#define CMT2300A_CUS_INT_FLAG           0x6D

#define CMT2300A_GO_STBY                0x02
#define CMT2300A_GO_RX                  0x08
#define CMT2300A_GO_TX                  0x40

#define CMT2300A_STA_STBY               0x02
#define CMT2300A_STA_TX                 0x06
#define CMT2300A_STA_RX                 0x05

#define CMT2300A_MASK_TX_DONE_FLG       0x08
#define CMT2300A_MASK_PKT_OK_FLG        0x01

#define CRC8_INIT                       0x00
#define CRC8_POLY                       0x01

#define UINT32_BYTE_3(val) ((uint8_t)((val >> 24) & 0xff))
#define UINT32_BYTE_2(val) ((uint8_t)((val >> 16) & 0xff))
#define UINT32_BYTE_1(val) ((uint8_t)((val >>  8) & 0xff))
#define UINT32_BYTE_0(val) ((uint8_t)((val      ) & 0xff))

typedef SoftSpi3w<DEF_CS_PIN, DEF_FCS_PIN, DEF_SCK_PIN, DEF_SDIO_PIN> SpiType;

SpiType spi3w;
uint16_t cnt;
bool txOk;

// config
uint32_t ts  = 0x63BFDBE1; // timestamp
uint32_t dtu = 0x83266790;
uint32_t wr  = 0x80423810;
// end config


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
bool cmtSwitchStatus(uint8_t cmd, uint8_t waitFor) {
    uint8_t i = 10;
    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, cmd);
    while(i--) {
        delayMicroseconds(1);
        if(waitFor == getChipStatus())
            return true;

        if(CMT2300A_GO_TX == cmd) {
            delayMicroseconds(1);

            if(CMT2300A_MASK_TX_DONE_FLG & spi3w.readReg(CMT2300A_CUS_INT_CLR1))
                return true;
        } else if(CMT2300A_GO_RX == cmd) {
            delayMicroseconds(1);

            if(CMT2300A_MASK_PKT_OK_FLG & spi3w.readReg(CMT2300A_CUS_INT_FLAG))
                return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    cnt = 0;
    txOk = false;
    spi3w.setup();

    delay(1000);
    Serial.println("start");

    spi3w.writeReg(0x7f, 0xff); // soft reset
    delay(20);

    // go to standby mode
    if(cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("standby mode reached");
    uint8_t tmp;
/*    tmp = spi3w.readReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;         // Enable CFG_RETAIN
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN;        // Disable RSTN_IN
    spi3w.writeReg(CMT2300A_CUS_MODE_STA, tmp);
*/
    spi3w.writeReg(CMT2300A_CUS_MODE_STA, 0x52);

    tmp  = spi3w.readReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN;         // Enable LOCKING_EN
    spi3w.writeReg(CMT2300A_CUS_EN_CTL, tmp);


    ///////////////////
    spi3w.writeReg(0x0D, 0x00);

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
    spi3w.writeReg(0x16, 0x0C);
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

    spi3w.writeReg(0x09, 0x02);
    spi3w.writeReg(0x65, 0x20);
    spi3w.writeReg(0x66, 0x0A);
    spi3w.writeReg(0x67, 0x07);
    spi3w.writeReg(0x68, 0x3B);

    spi3w.writeReg(0x41, 0x48);
    spi3w.writeReg(0x42, 0x5A);
    spi3w.writeReg(0x43, 0x48);
    spi3w.writeReg(0x44, 0x4D);
    spi3w.writeReg(0x64, 0x64);
    spi3w.writeReg(0x69, 0x02);
    spi3w.writeReg(0x60, 0x10);
    spi3w.writeReg(0x60, 0x02);
    spi3w.writeReg(0x18, 0x42);
    spi3w.writeReg(0x19, 0xA9);
    spi3w.writeReg(0x1A, 0xA4);
    spi3w.writeReg(0x1B, 0x8C);
    spi3w.writeReg(0x1C, 0x42);
    spi3w.writeReg(0x1D, 0x9E);
    spi3w.writeReg(0x1E, 0x4B);
    spi3w.writeReg(0x1F, 0x1C);

    spi3w.writeReg(0x22, 0x20);
    spi3w.writeReg(0x23, 0x20);
    spi3w.writeReg(0x24, 0xD2);
    spi3w.writeReg(0x25, 0x35);
    spi3w.writeReg(0x26, 0x0C);
    spi3w.writeReg(0x27, 0x0A);
    spi3w.writeReg(0x60, 0x10);
    spi3w.writeReg(0x60, 0x02);
    spi3w.writeReg(0x03, 0x1D);
    spi3w.writeReg(0x5C, 0x8A);
    spi3w.writeReg(0x5D, 0x18);
    spi3w.writeReg(0x60, 0x10);
    spi3w.writeReg(0x60, 0x02);

    spi3w.writeReg(0x6A, 0x00);
    spi3w.writeReg(0x6B, 0x00);
    spi3w.writeReg(0x69, 0x02);
    spi3w.writeReg(0x6C, 0x02);
    spi3w.writeReg(0x16, 0x0C);
    spi3w.writeReg(0x63, 0x01);
    spi3w.writeReg(0x60, 0x08);

    uint8_t cfg0[11] = {
        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xff
    };
    uint8_t cfg1[15] = {
        0x56, UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr),
        UINT32_BYTE_0(wr), 0x02, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg2[15] = {
        0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg3[11] = {
        0x07, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x00, 0xff
    };
    uint8_t cfg4[15] = {
        0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x21, 0x14, 0xff
    };
    uint8_t i;

    cfg0[10] = crc8(cfg0, 10);
    spi3w.writeFifo(cfg0, 11);
    dumpBuf("cfg0", cfg0, 11);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 5; i++) {
        spi3w.writeFifo(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg3[10] = crc8(cfg3, 10);
    spi3w.writeFifo(cfg3, 11);
    dumpBuf("cfg3", cfg3, 11);

    cfg4[14] = crc8(cfg4, 14);
    spi3w.writeFifo(cfg4, 15);
    dumpBuf("cfg4", cfg4, 15);

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 3; i++) {
        spi3w.writeFifo(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);
}

//-----------------------------------------------------------------------------
void loop() {
    delay(1000);
    ts++;

    if((++cnt % 5) == 0) {
        /*spi3w.writeReg(0x6A, 0x00);
        spi3w.writeReg(0x6B, 0x00);
        spi3w.writeReg(0x60, 0x02);
        spi3w.writeReg(0x69, 0x07);
        spi3w.writeReg(0x45, 0x01);
        spi3w.writeReg(0x46, 0x1B);
        spi3w.writeReg(0x63, 0x21);
        spi3w.writeReg(0x60, 0x40);*/

        //spi3w.writeReg(CMT2300A_CUS_FIFO_CLR, 0x01);

        if(cmtSwitchStatus(CMT2300A_GO_TX, CMT2300A_STA_TX)) {
            Serial.println("tx mode ok");
            txOk = true;

            uint8_t rqst[27] = {
                0x15, UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), 0x00, 0x00, 0x00,
                0x01, 0x80, 0x0B, 0x00, UINT32_BYTE_3(ts), UINT32_BYTE_2(ts), UINT32_BYTE_1(ts), UINT32_BYTE_0(ts),
                0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
                0x88, 0x90, 0xff
            };
            rqst[26] = crc8(rqst, 26);
            spi3w.writeFifo(rqst, 27);
            dumpBuf("request data", rqst, 27);
        }
        else {
            txOk = false;
            Serial.println("can't switch to tx");
        }
    }

    //if(txOk) {
        if(cmtSwitchStatus(CMT2300A_GO_RX, CMT2300A_STA_RX)) {
            //Serial.println("rx mode ok");
            uint8_t buf[27] = {0};
            spi3w.readFifo(buf, 27);

            // only print buffer if some fields are not equal 0
            for(uint8_t i = 0; i < 27; i++) {
                if(buf[i] != 0) {
                    dumpBuf("fifo", buf, 27);
                    break;
                }
            }
        }
    //}
}
