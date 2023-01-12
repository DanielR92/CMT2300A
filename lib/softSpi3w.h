#ifndef __SOFT_SPI_3W__
#define __SOFT_SPI_3W__

#include <Arduino.h>


#if defined(ESP8266)
    #define PIN_HIGH(pin)\
     ({\
        if(pin < 16)\
            GPOS = (1 << pin);\
        else\
            digitalWrite(pin, HIGH);\
    })
    #define PIN_LOW(pin)\
     ({\
        if(pin < 16)\
            GPOC = (1 << pin);\
        else\
            digitalWrite(pin, LOW);\
    })
    #define PIN_GET(pin) ((GPI >> pin) & 0x01)
#else
    #define PIN_HIGH(pin) digitalWrite(pin, HIGH)
    #define PIN_LOW(pin) digitalWrite(pin, LOW)
    #define PIN_GET(pin) digitalRead(pin)
#endif

template<uint8_t PIN_CSB, uint8_t PIN_FCS, uint8_t PIN_SCK, uint8_t PIN_SDIO>
class SoftSpi3w {
    public:
        SoftSpi3w() :
            mCsb(PIN_CSB),
            mFcs(PIN_FCS),
            mSck(PIN_SCK),
            mSdio(PIN_SDIO) {}

        void setup() {
            pinMode(mCsb, OUTPUT);
            pinMode(mFcs, OUTPUT);
            pinMode(mSck, OUTPUT);
            pinMode(mSdio, OUTPUT);
            PIN_HIGH(mCsb);
            PIN_HIGH(mFcs);
            PIN_LOW(mSck);
            PIN_HIGH(mSdio);
        }

        void writeReg(uint8_t addr, uint8_t reg) {
            PIN_LOW(mCsb);
            writeByte(addr);
            writeByte(reg);
            PIN_LOW(mSck);
            PIN_HIGH(mCsb);
            delayMicroseconds(30);
        }

        uint8_t readReg(uint8_t addr) {
            uint8_t reg = 0x00;
            PIN_LOW(mCsb);
            writeByte(addr | 0x80); // bit 7 marks r/w
            reg = readByte();
            PIN_HIGH(mCsb);
            delayMicroseconds(30);
            return reg;
        }

        void writeFifo(uint8_t buf[], uint8_t len) {
            for(uint8_t i = 0; i < len; i++) {
                PIN_LOW(mFcs);
                delayMicroseconds(2);
                writeByte(buf[i]);
                PIN_LOW(mSck);
                delayMicroseconds(3);
                PIN_HIGH(mFcs);
                delayMicroseconds(6);
            }
        }

        void readFifo(uint8_t buf[], uint8_t len) {
            pinMode(mSdio, INPUT);
            for(uint8_t i = 0; i < len; i++) {
                PIN_LOW(mFcs);
                delayMicroseconds(2);
                buf[i] = readByte(false);
                PIN_LOW(mSck);
                delayMicroseconds(3);
                PIN_HIGH(mFcs);
                delayMicroseconds(12);
            }
            pinMode(mSdio, OUTPUT);
        }

    private:
        inline void writeByte(uint8_t val) {
            PIN_HIGH(mSdio);
            for(int8_t i = 7; i >= 0; i--) {
                PIN_LOW(mSck);
                if((val >> i) & 0x01)
                    PIN_HIGH(mSdio);
                else
                    PIN_LOW(mSdio);
                delayMicroseconds(1);
                PIN_HIGH(mSck);
                delayMicroseconds(1);
            }
            delayMicroseconds(1);
        }

        inline uint8_t readByte(bool changePinMode = true) {
            uint8_t val = 0x00;
            if(changePinMode)
                pinMode(mSdio, INPUT);
            for(int8_t i = 7; i >= 0; i--) {
                PIN_LOW(mSck);
                delayMicroseconds(1);
                PIN_HIGH(mSck);
                val |= (PIN_GET(mSdio) << i);
                delayMicroseconds(1);
            }
            PIN_LOW(mSck);
            delayMicroseconds(2);
            if(changePinMode)
                pinMode(mSdio, OUTPUT);
            PIN_HIGH(mSdio);

            return val;
        }

        uint8_t mCsb, mFcs, mSck, mSdio;
};

#endif /*__SOFT_SPI_3W__*/
