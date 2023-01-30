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

class RP2040Spi3w {
    public:
        void setup() {
            mPio = pio0;

            gpio_init(SPI_FCSB);
            gpio_set_dir(SPI_FCSB, GPIO_OUT);
            gpio_put(SPI_FCSB, 1);

            uint offset_cfg  = pio_add_program(mPio, &spi3w_cfg_program);
            uint offset_fifo = pio_add_program(mPio, &spi3w_fifo_program);
            spi3w_cfg_init(mPio, 0, offset_cfg, 14.0f, SPI_CSB, SPI_SCK, SPI_SDIO);
            spi3w_fifo_init(mPio, 1, offset_fifo, 14.0f, SPI_SCK, SPI_SDIO);
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
            pio_sm_put_blocking(mPio, 0, (uint32_t)((addr << 24u) | (addr & 0x80) << 16u) | (data << 15u));
            if(addr & 0x80) {
                rx = pio_sm_get_blocking(mPio, 0);
                //Serial.println(String(addr, HEX) + ": " + String(rx, HEX));
            }
            sleep_us(12);
            return (rx & 0xff); //(rx >> 24u);
        }

        inline uint8_t transferFifo(bool read, uint8_t data) {
            uint32_t rx = 0;
            gpio_put(SPI_FCSB, 0);
            sleep_us(3);
            pio_sm_clear_fifos(mPio, 1);
            pio_sm_put_blocking(mPio, 1, (uint32_t)(((read) ? 0x01 : 0x00) << 31u) | (data << 23u));
            if(read)
                rx = pio_sm_get_blocking(mPio, 1);
            sleep_us(7);
            gpio_put(SPI_FCSB, 1);
            sleep_us(10);
            return (rx & 0xff); //(rx >> 24u);
        }

        PIO mPio;
};
