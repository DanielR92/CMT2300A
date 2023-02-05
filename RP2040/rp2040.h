#include <Arduino.h>
#include "spi_3wire.pio.h"
#include "hardware/uart.h"

#ifndef GPIO_OUT
#define GPIO_OUT true
#endif

#define RP2040

//-----------------------------------------------------------------------------
// PINOUT
#define SPI_FCSB        2
#define SPI_CSB         3
#define SPI_SCK         4
#define SPI_SDIO        5

#define CSB_MANUAL

class RP2040Spi3w {
    public:
        void setup() {
            mPio = pio0;

            gpio_init(SPI_FCSB);
            gpio_set_dir(SPI_FCSB, GPIO_OUT);
            gpio_put(SPI_FCSB, 1);
            #ifdef CSB_MANUAL
                gpio_init(SPI_CSB);
                gpio_set_dir(SPI_CSB, GPIO_OUT);
                gpio_put(SPI_CSB, 1);
            #endif

            #ifdef CSB_MANUAL
                uint offset_cfg  = pio_add_program(mPio, &spi3w_cfg2_program);
                spi3w_cfg2_init(mPio, 0, offset_cfg, 14.0f, SPI_SCK, SPI_SDIO);
            #else
                uint offset_cfg  = pio_add_program(mPio, &spi3w_cfg_program);
                spi3w_cfg_init(mPio, 0, offset_cfg, 14.0f, SPI_CSB, SPI_SCK, SPI_SDIO);
            #endif
            uint offset_fifo = pio_add_program(mPio, &spi3w_fifo_program);
            spi3w_fifo_init(mPio, 1, offset_fifo, 14.0f, SPI_SCK, SPI_SDIO);

            /*for(uint16_t i = 0; i < 65500; i++) {
                transferFifo(((i & 0x01) == 0x01), (i >> 1) & 0xff);
            }*/
        }

        void writeReg(uint8_t addr, uint8_t reg) {
            transferCfg(addr, reg);
        }

        uint8_t readReg(uint8_t addr) {
            return transferCfg(addr | 0x80, 0xff);
        }

        void writeFifo(uint8_t buf[], uint8_t len) {
            for(uint8_t i = 0; i < len; i++) {
                transferFifo(false, buf[i]);
            }
        }

        void readFifo(uint8_t buf[], uint8_t len) {
            for(uint8_t i = 0; i < len; i++) {
                buf[i] = transferFifo(true, 0xff);
            }
        }

    private:
        inline uint8_t transferCfg(uint8_t addr, uint8_t data) {
            uint32_t rx = 0;
            #ifdef CSB_MANUAL
                gpio_put(SPI_CSB, 0);
                sleep_us(4);
            #endif
            pio_sm_put_blocking(mPio, 0, (uint32_t)((addr << 24u) | (addr & 0x80) << 16u) | (data << 15u));
            if(addr & 0x80) {
                rx = pio_sm_get_blocking(mPio, 0);
                //Serial.println(String(addr, HEX) + ": " + String(rx, HEX));
            }
            #ifdef CSB_MANUAL
                sleep_us(8);
                gpio_put(SPI_CSB, 1);
                sleep_us(10);
            #else
                sleep_us(12);
            #endif
            return (rx & 0xff); //(rx >> 24u);
        }

        inline uint8_t transferFifo(bool read, uint8_t data) {
            uint32_t rx = 0;
            gpio_put(SPI_FCSB, 0);
            sleep_us(2);
            pio_sm_clear_fifos(mPio, 1);
            pio_sm_put_blocking(mPio, 1, (uint32_t)(((read) ? 0x00 : 0x03) << 30u) | (data << 23u));
            if(read) {
                rx = pio_sm_get_blocking(mPio, 1);
                sleep_us(2);
            }
            else
                sleep_us(5);

            gpio_put(SPI_FCSB, 1);
            sleep_us(1);
            return (rx & 0xff); //(rx >> 24u);
        }

        PIO mPio;
};
