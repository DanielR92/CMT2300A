#ifndef __CMT_SPI3_H
#define __CMT_SPI3_H

#include "typedefs.h"

void cmt_spi3_init(void);

void cmt_spi3_write(u8 addr, u8 dat);
u8 cmt_spi3_read(u8 addr);

void cmt_spi3_write_fifo(const u8* p_buf, u16 len);
void cmt_spi3_read_fifo(u8* p_buf, u16 len);

#endif
