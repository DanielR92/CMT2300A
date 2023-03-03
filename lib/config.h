#pragma once

// CMT pin config
#define CMT_PIN_CLK   18
#define CMT_PIN_SDIO  23
#define CMT_PIN_CS    5
#define CMT_PIN_FCS   4
#define CMT_PIN_GPIO3 15

// DTU config
#define INV_SERIAL 0x116480724087 // 12 digit inverter serial
#define DTU_SERIAL 0x81111111     // imagined last 8 digits of dtu serial
#define WORK_FREQ  863000         // desired work freqency between inverter and dtu in kHz (250 kHz steps)
                                  // !!! inverter should be DC off and back on bevor use this code as dtu btw. no other dtu should be connected this day, because no channel hopping is implemented !!!