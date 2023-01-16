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

template<uint8_t PIN_CSB, uint8_t PIN_FCS, uint8_t PIN_SCK, uint8_t PIN_SDIO, uint8_t PIN_GPIO1>
class SoftSpi3w {
    public:
        SoftSpi3w() :
            mCsb(PIN_CSB),
            mFcs(PIN_FCS),
            mSck(PIN_SCK),
            mGpio(PIN_GPIO1),
            mSdio(PIN_SDIO) {}

        void setup() {
            pinMode(mSck, OUTPUT);
            pinMode(mSdio, OUTPUT);
            pinMode(mFcs, OUTPUT);
            pinMode(mCsb, OUTPUT);
            pinMode(mGpio, INPUT);

            PIN_HIGH(mCsb); /* CSB has an internal pull-up resistor */
            PIN_LOW(mSck); /* SCLK has an internal pull-down resistor */
            PIN_HIGH(mSdio);
            PIN_HIGH(mFcs);  /* FCSB has an internal pull-up resistor */
            delayMicroseconds(1);
        }

        /*! ********************************************************
        * @name    WriteReg
        * @desc    Write the CMT2300A register at the specified address.
        * @param   addr: register address
        *          dat: register value
        * *********************************************************/
        void writeReg(uint8_t addr, uint8_t reg) {
            pinMode(mSdio, OUTPUT);
            PIN_HIGH(mSdio);
            PIN_LOW(mSck);
            PIN_HIGH(mFcs);
            PIN_LOW(mCsb);

            delayMicroseconds(1);  // > 0.5 SCLK  = >2us

            writeByte(addr & 0x7F); // r/w = 0
            writeByte(reg);

            PIN_LOW(mSck);
            delayMicroseconds(1);  // > 0.5 SCLK cycle TODO: Check
            PIN_HIGH(mCsb); 
            PIN_HIGH(mSdio);
            pinMode(mSdio, INPUT);
            PIN_HIGH(mFcs);


            /*PIN_LOW(mCsb);
            //writeByte(addr);
            writeByte(addr & 0x7F); // r/w = 0
            writeByte(reg);
            PIN_LOW(mSck);
            PIN_HIGH(mCsb);
            delayMicroseconds(30);*/
        }

        /*! ********************************************************
        * @name    CMT2300A_ReadReg
        * @desc    Read the CMT2300A register at the specified address.
        * @param   addr: register address
        * @return  Register value
        * *********************************************************/
        uint8_t readReg(uint8_t addr) {
            uint8_t dat = 0xFF;

            pinMode(mSdio, OUTPUT);
            PIN_HIGH(mSdio);
            PIN_LOW(mSck);
            PIN_HIGH(mFcs);
            PIN_LOW(mCsb);

            delayMicroseconds(1);  // > 0.5 SCLK  >2us
            writeByte(addr | 0x80); // bit 7 marks r/w

            pinMode(mSdio, INPUT);  /* Must set SDIO to input before the falling edge of SCLK */ 
            dat = readByte();
            PIN_LOW(mSck);
            delayMicroseconds(1);  // > 0.5 SCLK  >2us
            PIN_HIGH(mCsb);
            PIN_HIGH(mSdio);
            PIN_HIGH(mFcs);
            pinMode(mSdio, INPUT);
            PIN_HIGH(mFcs);
            return dat;
        }

        /*! ********************************************************
        * @name    WriteFifo
        * @desc    Writes the buffer contents to the CMT2300A FIFO.
        * @param   buf: buffer containing data to be put on the FIFO
        *          len: number of bytes to be written to the FIFO
        * *********************************************************/
        void writeFifo(uint8_t buf[], uint8_t len) {
            uint16_t i;
            
            PIN_HIGH(mFcs);
            PIN_HIGH(mCsb);
            PIN_LOW(mSck);
            pinMode(mSdio, OUTPUT);

            for(i=0; i < len; i++)
            {
                PIN_LOW(mFcs);
                delayMicroseconds(1);   /* > 1 SCLK cycle */
                delayMicroseconds(1);
                delayMicroseconds(1);

                writeByte(buf[i]);
                
                PIN_LOW(mSck);     
                delayMicroseconds(1);/* > 2 us */
                delayMicroseconds(1);
                delayMicroseconds(1);
                PIN_HIGH(mFcs);     
                delayMicroseconds(1);/* > 4 us */
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
            }
            pinMode(mSdio, INPUT);
            PIN_HIGH(mFcs);
        }

        uint8_t cmt_spi3_recv(void) {
            uint8_t i;
            uint8_t data8 = 0xFF;
            for(i=0; i<8; i++)
            {
                PIN_LOW(mSck);
                delayMicroseconds(1);
                data8 <<= 1;
                PIN_HIGH(mSck);     
                if(PIN_GET(mSdio))/* Read byte on the rising edge of SCLK */
                    data8 |= 0x01;
                else
                    data8 &= ~0x01;
                delayMicroseconds(1);
            }
            return data8;
        }

        /*! ********************************************************
        * @name    ReadFifo
        * @desc    Reads the contents of the CMT2300A FIFO.
        * @param   buf: buffer where to copy the FIFO read data
        *          len: number of bytes to be read from the FIFO
        * *********************************************************/
        void readFifo(uint8_t buf[], uint8_t len) {
            uint16_t i;
            PIN_HIGH(mFcs);
            PIN_HIGH(mCsb);
            PIN_LOW(mSck);
            pinMode(mSdio, INPUT);

            for(i = 0; i < len; i++) {
                PIN_LOW(mFcs);
                /* > 1 SCLK cycle */
                delayMicroseconds(1);
                delayMicroseconds(1);

                buf[i] = cmt_spi3_recv();
                PIN_LOW(mSck);
                /* > 2 us */
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                PIN_HIGH(mFcs);
                /* > 4 us */
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
                delayMicroseconds(1);
            }

            pinMode(mSdio, INPUT);
            PIN_HIGH(mFcs);
        }

    private:
        inline void writeByte(uint8_t val) {
            PIN_HIGH(mSdio);
            PIN_HIGH(mSdio);
            PIN_LOW(mSck);
            PIN_HIGH(mFcs);
            PIN_LOW(mCsb);
            delay(0);
            
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

        uint8_t mCsb, mFcs, mSck, mSdio, mGpio;
};

#endif /*__SOFT_SPI_3W__*/
