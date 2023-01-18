#include <Arduino.h>

//#include <stdio.h>
//#include "pico/stdlib.h"

#include "spi_3wire.pio.h"

#ifndef GPIO_OUT
#define GPIO_IN  false
#define GPIO_OUT true
#endif

#define SPI_FCSB    2
#define SPI_CSB     3
#define SPI_SCK     4
#define SPI_SDIO    5

uint8_t val = 0;

void setup() {
    gpio_init(SPI_FCSB);
    gpio_set_dir(SPI_FCSB, GPIO_OUT);
    gpio_put(SPI_FCSB, 1);

    PIO pio = pio0;
    uint offset_cfg  = pio_add_program(pio, &spi3w_cfg_program);
    uint offset_fifo = pio_add_program(pio, &spi3w_fifo_program);
    spi3w_cfg_init(pio, 0, offset_cfg, 14.0f, SPI_CSB, SPI_SCK, SPI_SDIO);
    spi3w_fifo_init(pio, 1, offset_fifo, 14.0f, SPI_SCK, SPI_SDIO);
}

uint8_t transferCfg(uint8_t addr, uint8_t data) {
    uint32_t rx;
    pio_sm_put_blocking(pio0, 0, (uint32_t)((addr << 24u) | (addr & 0x80) << 16u) | (data << 15u));
    if(addr & 0x80)
        rx = pio_sm_get_blocking(pio0, 0);
    sleep_us(12);
    return (rx >> 24u);
}

uint8_t transferFifo(bool read, uint8_t data) {
    uint32_t rx;
    gpio_put(SPI_FCSB, 0);
    pio_sm_put_blocking(pio0, 1, (uint32_t)(((read) ? 0x01 : 0x00) << 31u) | (data << 23u));
    if(read)
        rx = pio_sm_get_blocking(pio0, 1);
    sleep_us(5);
    gpio_put(SPI_FCSB, 1);
    sleep_us(7);
    return (rx >> 24u);
}

void loop() {
    sleep_ms(1);
    transferCfg(val, 0x13);
    transferFifo(false, 0xAe);
    //transferFifo(true, 0xFF);
    val++;
}
