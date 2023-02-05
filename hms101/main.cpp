//-----------------------------------------------------------------------------
// 2023 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include <Arduino.h>

#define TRIG    5 // D1 - GPIO5

#define CRC8_INIT                       0x00
#define CRC8_POLY                       0x01

#define CRC16_MODBUS_POLYNOM            0xA001

#define U32_B3(val) ((uint8_t)((val >> 24) & 0xff))
#define U32_B2(val) ((uint8_t)((val >> 16) & 0xff))
#define U32_B1(val) ((uint8_t)((val >>  8) & 0xff))
#define U32_B0(val) ((uint8_t)((val      ) & 0xff))

// config
uint32_t ts  = 0x6384ddbb; // timestamp
uint32_t dtu = 0x81001756; //0x83266790;
uint32_t wr  = 0x80423810;


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
void wrbuf(uint8_t buf[], uint8_t len, bool doCrc8 = true, bool doCrc16 = true) {
    uint8_t tmp[256];
    if(doCrc16) {
        buf[len-4] = 0x00;
        buf[len-3] = 0x00;
        buf[len-2] = 0x00;
        uint16_t crc2 = crc16(&buf[10], len-13, 0xffff);
        buf[len-3] = (crc2 >> 8 & 0xff);
        buf[len-2] = (crc2      & 0xff);
    }
    if(doCrc8)
        buf[len-1] = crc8(buf, len-1);

    tmp[0] = 0x7e;
    memcpy(&tmp[1], buf, len);
    tmp[len+1] = 0x7f;

    digitalWrite(TRIG, HIGH);
    Serial.write(tmp, len+2);
    digitalWrite(TRIG, LOW);
}



//-----------------------------------------------------------------------------
void setup() {
    Serial.begin(125000);
    pinMode(TRIG, OUTPUT);
    digitalWrite(TRIG, LOW);

    uint8_t cfg0[11] = {
        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xff
    };
    uint8_t cfg1[15] = {
        0x56, U32_B3(wr), U32_B2(wr), U32_B1(wr), U32_B0(wr), U32_B3(wr), U32_B2(wr), U32_B1(wr),
        U32_B0(wr), 0x02, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg2[15] = {
        0x56, U32_B3(dtu), U32_B2(dtu), U32_B1(dtu), U32_B0(dtu), U32_B3(dtu), U32_B2(dtu), U32_B1(dtu),
        U32_B0(dtu), 0x01, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg3[11] = {
        0x07, U32_B3(dtu), U32_B2(dtu), U32_B1(dtu), U32_B0(dtu), U32_B3(dtu), U32_B2(dtu), U32_B1(dtu),
        U32_B0(dtu), 0x00, 0xff
    };
    uint8_t cfg4[15] = {
        0x56, U32_B3(dtu), U32_B2(dtu), U32_B1(dtu), U32_B0(dtu), U32_B3(dtu), U32_B2(dtu), U32_B1(dtu),
        U32_B0(dtu), 0x01, 0x15, 0x21, 0x21, 0x14, 0xff
    };

    delay(5000);
    wrbuf(cfg0, 11, true, false);
    for(uint8_t i = 0; i < 5 ; i++) {
        delay(10);
        wrbuf(cfg1, 15, true, false);
    }
    delay(10);
    wrbuf(cfg2, 15, true, false);
    delay(10);
    wrbuf(cfg3, 11, true, false);
    delay(10);
    wrbuf(cfg4, 15, true, false);
    for(uint8_t i = 0; i < 3 ; i++) {
        delay(10);
        wrbuf(cfg1, 15, true, false);
    }
    delay(10);
    wrbuf(cfg2, 15, true, false);

}

//-----------------------------------------------------------------------------
void loop() {
    delay(500);
    uint8_t rqst[27] = {
        0x15, U32_B3(wr), U32_B2(wr), U32_B1(wr), U32_B0(wr), U32_B3(wr), U32_B2(wr), U32_B1(wr),
        U32_B0(wr), 0x80, 0x0B, 0x00, U32_B3(ts), U32_B2(ts), U32_B1(ts), U32_B0(ts),
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xff
    };
    //wrbuf(rqst, 27, true, true);
    // delay(200);
    wrbuf(rqst, 27, true, true);
    delay(1500);
    ts++;
}
