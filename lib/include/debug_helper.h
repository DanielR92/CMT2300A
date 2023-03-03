#pragma once

#include <Arduino.h>
#include "config.h"

//-----------------------------------------------------------------------------
void dumpBuf(const char* des, uint8_t buf[], uint8_t len, bool newline = true) {
    //Serial.println("------------------");
    Serial.print(String(des));
    for(uint8_t i = 0; i < len; i++) {
        if((0 != i) && (i % 32 == 0))
            Serial.println("");
        if(buf[i] < 16)
            Serial.print(" 0" + String(buf[i], HEX));
        else
            Serial.print(" " + String(buf[i], HEX));
    }
    if(newline)
        Serial.println("");
}

//-----------------------------------------------------------------------------
void printVal(uint8_t pay[], uint8_t offs, uint8_t len, uint16_t div, const char* des, const char* unit, uint8_t isSpecial = 0) {
    uint32_t tmp = pay[offs];
    for(uint8_t i = 1; i < len; i++) {
        tmp <<= 8;
        tmp |= pay[offs+i];
    }
    char info[30] = {0};
    if(isSpecial == 0) // unsigned 16 bit
        snprintf(info, 30, "%s: %.2f%s", des, (float)tmp/(float)div, unit);
    else if(isSpecial == 1) // signed 16 bit
        snprintf(info, 30, "%s: %.2f%s", des, ((float)((int16_t)tmp))/(float)div, unit);
    else if(isSpecial == 2) // special for PF
        snprintf(info, 30, "%s: %.3f%s", des, ((tmp>0 && tmp<div/2) ? (float)tmp-div : (float)tmp)/(float)div, unit);
    Serial.println(String(info));
}

void printData(uint8_t payload[], uint8_t length) {
    uint64_t inv_serial = INV_SERIAL;
    uint16_t preSerial = (inv_serial >> 32) & 0xffff;
    switch(preSerial) {
        case 0x1124: // HMS 1 channel
            printVal(payload,  2, 2,   10, "CH1 - U  ", "V");
            printVal(payload,  4, 2,  100, "CH1 - I  ", "A");
            printVal(payload,  6, 2,   10, "CH1 - P  ", "W");
            printVal(payload,  8, 4, 1000, "CH1 - YT ", "kWh");
            printVal(payload, 12, 2,    1, "CH1 - YD ", "Wh");

            printVal(payload, 14, 2,   10, "CH0 - U  ", "V");
            printVal(payload, 16, 2,  100, "CH0 - F  ", "Hz");
            printVal(payload, 18, 2,   10, "CH0 - P  ", "W");
            printVal(payload, 20, 2,   10, "CH0 - Q  ", "var", 1);
            printVal(payload, 22, 2,  100, "CH0 - I  ", "A");
            printVal(payload, 24, 2, 1000, "CH0 - PF ", "", 2);
            printVal(payload, 26, 2,   10, "CH0 - T  ", "°C", 1);
            printVal(payload, 28, 2,    1, "CH0 - EVT", "");
            break;
        case 0x1144: // HMS 2 channels
            printVal(payload,  2, 2,   10, "CH1 - U  ", "V");
            printVal(payload,  4, 2,   10, "CH2 - U  ", "V");
            printVal(payload,  6, 2,  100, "CH1 - I  ", "A");
            printVal(payload,  8, 2,  100, "CH2 - I  ", "A");
            printVal(payload, 10, 2,   10, "CH1 - P  ", "W");
            printVal(payload, 12, 2,   10, "CH2 - P  ", "W");
            printVal(payload, 14, 4, 1000, "CH1 - YT ", "kWh");
            printVal(payload, 18, 4, 1000, "CH2 - YT ", "kWh");
            printVal(payload, 22, 2,    1, "CH1 - YD ", "Wh");
            printVal(payload, 24, 2,    1, "CH2 - YD ", "Wh");

            printVal(payload, 26, 2,   10, "CH0 - U  ", "V");
            printVal(payload, 28, 2,  100, "CH0 - F  ", "Hz");
            printVal(payload, 30, 2,   10, "CH0 - P  ", "W");
            printVal(payload, 32, 2,   10, "CH0 - Q  ", "var", 1);
            printVal(payload, 34, 2,  100, "CH0 - I  ", "A");
            printVal(payload, 36, 2, 1000, "CH0 - PF ", "", 2);
            printVal(payload, 38, 2,   10, "CH0 - T  ", "°C", 1);
            printVal(payload, 40, 2,    1, "CH0 - EVT", "");
            break;
        case 0x1164: // HMS 4 channels
            printVal(payload,  2, 2,   10, "CH1 - U  ", "V");
            printVal(payload,  4, 2,   10, "CH2 - U  ", "V");
            printVal(payload,  6, 2,  100, "CH1 - I  ", "A");
            printVal(payload,  8, 2,  100, "CH2 - I  ", "A");
            printVal(payload, 10, 2,   10, "CH1 - P  ", "W");
            printVal(payload, 12, 2,   10, "CH2 - P  ", "W");
            printVal(payload, 14, 4, 1000, "CH1 - YT ", "kWh");
            printVal(payload, 18, 4, 1000, "CH2 - YT ", "kWh");
            printVal(payload, 22, 2,    1, "CH1 - YD ", "Wh");
            printVal(payload, 24, 2,    1, "CH2 - YD ", "Wh");

            printVal(payload, 26, 2,   10, "CH3 - U  ", "V");
            printVal(payload, 28, 2,   10, "CH4 - U  ", "V");
            printVal(payload, 30, 2,  100, "CH3 - I  ", "A");
            printVal(payload, 32, 2,  100, "CH4 - I  ", "A");
            printVal(payload, 34, 2,   10, "CH3 - P  ", "W");
            printVal(payload, 36, 2,   10, "CH4 - P  ", "W");
            printVal(payload, 38, 4, 1000, "CH3 - YT ", "kWh");
            printVal(payload, 42, 4, 1000, "CH4 - YT ", "kWh");
            printVal(payload, 46, 2,    1, "CH3 - YD ", "Wh");
            printVal(payload, 48, 2,    1, "CH4 - YD ", "Wh");

            printVal(payload, 50, 2,   10, "CH0 - U  ", "V");
            printVal(payload, 52, 2,  100, "CH0 - F  ", "Hz");
            printVal(payload, 54, 2,   10, "CH0 - P  ", "W");
            printVal(payload, 56, 2,   10, "CH0 - Q  ", "var", 1);
            printVal(payload, 58, 2,  100, "CH0 - I  ", "A");
            printVal(payload, 60, 2, 1000, "CH0 - PF ", "", 2);
            printVal(payload, 62, 2,   10, "CH0 - T  ", "°C", 1);
            printVal(payload, 64, 2,    1, "CH0 - EVT", "");
            break;
        case 0x1382: // HMT 6 channels
            printVal(payload,  2, 2,   10, "CH1+2 - U    ", "V");
            printVal(payload,  4, 2,  100, "CH1   - I    ", "A");
            printVal(payload,  6, 2,  100, "CH2   - I    ", "A");
            printVal(payload,  8, 2,   10, "CH1   - P    ", "W");
            printVal(payload, 10, 2,   10, "CH2   - P    ", "W");
            printVal(payload, 12, 4, 1000, "CH1   - YT   ", "kWh");
            printVal(payload, 16, 4, 1000, "CH2   - YT   ", "kWh");
            printVal(payload, 20, 2,    1, "CH1   - YD   ", "Wh");
            printVal(payload, 22, 2,    1, "CH2   - YD   ", "Wh");

            printVal(payload, 24, 2,   10, "CH3+4 - U    ", "V");
            printVal(payload, 26, 2,  100, "CH3   - I    ", "A");
            printVal(payload, 28, 2,  100, "CH4   - I    ", "A");
            printVal(payload, 30, 2,   10, "CH3   - P    ", "W");
            printVal(payload, 32, 2,   10, "CH4   - P    ", "W");
            printVal(payload, 34, 4, 1000, "CH3   - YT   ", "kWh");
            printVal(payload, 38, 4, 1000, "CH4   - YT   ", "kWh");
            printVal(payload, 42, 2,    1, "CH3   - YD   ", "Wh");
            printVal(payload, 44, 2,    1, "CH4   - YD   ", "Wh");

            printVal(payload, 46, 2,   10, "CH5+6 - U    ", "V");
            printVal(payload, 48, 2,  100, "CH5   - I    ", "A");
            printVal(payload, 50, 2,  100, "CH6   - I    ", "A");
            printVal(payload, 52, 2,   10, "CH5   - P    ", "W");
            printVal(payload, 54, 2,   10, "CH6   - P    ", "W");
            printVal(payload, 56, 4, 1000, "CH5   - YT   ", "kWh");
            printVal(payload, 60, 4, 1000, "CH6   - YT   ", "kWh");
            printVal(payload, 64, 2,    1, "CH5   - YD   ", "Wh");
            printVal(payload, 66, 2,    1, "CH6   - YD   ", "Wh");

            printVal(payload, 68, 2,   10, "CH0   - U_1-N", "V");
            printVal(payload, 70, 2,   10, "CH0   - U_2-N", "V");
            printVal(payload, 72, 2,   10, "CH0   - U_3-N", "V");
            printVal(payload, 74, 2,   10, "CH0   - U_1-2", "V");
            printVal(payload, 76, 2,   10, "CH0   - U_2-3", "V");
            printVal(payload, 78, 2,   10, "CH0   - U_3-1", "V");
            printVal(payload, 80, 2,  100, "CH0   - F    ", "Hz");
            printVal(payload, 82, 2,   10, "CH0   - P    ", "W");
            printVal(payload, 84, 2,   10, "CH0   - Q    ", "var", 1);
            printVal(payload, 86, 2,  100, "CH0   - I1   ", "A");
            printVal(payload, 88, 2,  100, "CH0   - I2   ", "A");
            printVal(payload, 90, 2,  100, "CH0   - I3   ", "A");
            printVal(payload, 92, 2, 1000, "CH0   - PF   ", "", 2);
            printVal(payload, 94, 2,   10, "CH0   - T    ", "°C", 1);
            printVal(payload, 96, 2,    1, "CH0   - EVT  ", "");
            break;
        default:
            for(uint8_t i = 0; i < length; i+=2)
                printVal(payload, i, 2, 1, "CH? -> ", "");
    }
}