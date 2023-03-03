#include <Arduino.h>
#include "radio.h"
#include "config.h"

const uint64_t inv_serial = INV_SERIAL;
const uint32_t dtu_serial = DTU_SERIAL;
const uint32_t work_freq = WORK_FREQ;

uint32_t ts  = 0x63FC0000; // unix timestamp for testing

uint32_t mPrevMillis;
bool mToggle;

void setup() {
    Serial.begin(115200);

    if(!RF_Init()) {
        Serial.println("Init CMT2300A failed!");
        while(1);
    }

    cmtSwitchDtuFreq(work_freq); // start dtu at work freqency, for fast Rx if inverter is already on and frequency switched
    mToggle = true;

    mPrevMillis = millis();
}

void loop() {
    if ((millis() - mPrevMillis) > 2000) { // every 2 seconds
        mPrevMillis = millis();

        if (mToggle || cmtFirstDataReceived()) {
            uint8_t rqst[] = { // RealTimeRunData_Debug
                0x15, U32_B3(inv_serial), U32_B2(inv_serial), U32_B1(inv_serial), U32_B0(inv_serial), U32_B3(dtu_serial), U32_B2(dtu_serial), U32_B1(dtu_serial), U32_B0(dtu_serial), 
                0x80, 0x0B, 0x00, U32_B3(ts), U32_B2(ts), U32_B1(ts), U32_B0(ts), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };
            cmtTxData(rqst, sizeof(rqst));
        }
        else
            cmtSwitchInvAndDtuFreq(HOY_BOOT_FREQ / 1000, work_freq, inv_serial, dtu_serial);

        mToggle = !mToggle;
        ts++;
    }

    switch(RF_Process()) // CMT loop
    {
        case RF_RX_TIMEOUT:
            Serial.println("RF_RX_TIMEOUT");
            break;

        case RF_ERROR:
            Serial.println("RF_ERROR");
            break;

        default:
            break;
    }
}
