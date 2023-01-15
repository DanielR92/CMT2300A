//-----------------------------------------------------------------------------
// 2022 Ahoy, https://www.mikrocontroller.net/topic/525778
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//-----------------------------------------------------------------------------

#include <Arduino.h>
//#include "SPI.h"
#include "softSpi3w.h"
#include "cmt2300a_defs.h"
#include "cmt2300a_params.h"

#define ENABLE_AUTO_SWITCH_CHIP_STATUS   /* Enable the auto switch chip status */

#define DEF_CS_PIN                      16 // D0 - GPIO16
#define DEF_FCS_PIN                     2  // D4 - GPIO2
#define DEF_SCK_PIN                     14 // D5 - GPIO14
#define DEF_SDIO_PIN                    12 // D6 - GPIO12

#define CRC8_INIT                       0x00
#define CRC8_POLY                       0x01

#define UINT32_BYTE_3(val) ((uint8_t)((val >> 24) & 0xff))
#define UINT32_BYTE_2(val) ((uint8_t)((val >> 16) & 0xff))
#define UINT32_BYTE_1(val) ((uint8_t)((val >>  8) & 0xff))
#define UINT32_BYTE_0(val) ((uint8_t)((val      ) & 0xff))

typedef SoftSpi3w<DEF_CS_PIN, DEF_FCS_PIN, DEF_SCK_PIN, DEF_SDIO_PIN> SpiType;

SpiType spi3w;
uint16_t cnt;
bool txOk;

// config
uint32_t ts  = 0x63BFDBE1; // timestamp
uint32_t dtu = 0x83266790;
uint32_t wr  = 0x80423810;
// end config


uint8_t CMT2300A_GetChipStatus(void) {
    return spi3w.readReg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
}

/*! ********************************************************
* @name    CMT2300A_AutoSwitchStatus
* @desc    Auto switch the chip status, and 10 ms as timeout.
* @param   nGoCmd: the chip next status
* @return  TRUE or FALSE
* *********************************************************/
bool CMT2300A_AutoSwitchStatus(u8 nGoCmd)
{
#ifdef ENABLE_AUTO_SWITCH_CHIP_STATUS
//    u32 nBegTick = CMT2300A_GetTickCount();
	uint8_t nBegTick = 3;
    u8  nWaitStatus;
    
    switch(nGoCmd)
    {
    case CMT2300A_GO_SLEEP: nWaitStatus = CMT2300A_STA_SLEEP; break;
    case CMT2300A_GO_STBY : nWaitStatus = CMT2300A_STA_STBY ; break;
    case CMT2300A_GO_TFS  : nWaitStatus = CMT2300A_STA_TFS  ; break;
    case CMT2300A_GO_TX   : nWaitStatus = CMT2300A_STA_TX   ; break;
    case CMT2300A_GO_RFS  : nWaitStatus = CMT2300A_STA_RFS  ; break;
    case CMT2300A_GO_RX   : nWaitStatus = CMT2300A_STA_RX   ; break;
    }
    
    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, nGoCmd);
    
//    while(CMT2300A_GetTickCount()-nBegTick < 10)
	while(nBegTick--)
    {
        delay(1);
        
        if(nWaitStatus==CMT2300A_GetChipStatus())
            return true;
        
        Serial.print("ChipStatus: ");
        Serial.println(nWaitStatus);

        if(CMT2300A_GO_TX==nGoCmd) {
            delay(1);
            if(CMT2300A_MASK_TX_DONE_FLG & spi3w.readReg(CMT2300A_CUS_INT_CLR1))
                return true;
        }
        
        if(CMT2300A_GO_RX==nGoCmd) {
            delay(1);
            if(CMT2300A_MASK_PKT_OK_FLG & spi3w.readReg(CMT2300A_CUS_INT_FLAG))
                return true;
        }
    }
    
    return false;
    
#else
    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, nGoCmd);
    return true;
#endif
}
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
void dumpBuf(const char* des, uint8_t buf[], uint8_t len) {
    Serial.println("------------------");
    Serial.println(String(des));
    for(uint8_t i = 0; i < len; i++) {
        if((0 != i) && (i % 8 == 0))
            Serial.println("");
        Serial.print(String(buf[i], HEX) + " ");
    }
    Serial.println("");
}
//-----------------------------------------------------------------------------
bool cmtSwitchStatus(uint8_t cmd, uint8_t waitFor) {
    uint8_t i = 10;
    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, cmd);
    while(i--) {
        delayMicroseconds(1);
        if(waitFor == CMT2300A_GetChipStatus())
            return true;

        /*if(CMT2300A_GO_TX == cmd) {
            delayMicroseconds(1);

            if(CMT2300A_MASK_TX_DONE_FLG & spi3w.readReg(CMT2300A_CUS_INT_CLR1))
                return true;
        } else if(CMT2300A_GO_RX == cmd) {
            delayMicroseconds(1);

            if(CMT2300A_MASK_PKT_OK_FLG & spi3w.readReg(CMT2300A_CUS_INT_FLAG))
                return true;
        }*/
    }
    return false;
}
//-----------------------------------------------------------------------------
/*! ********************************************************
* @name    CMT2300A_ConfigRegBank
* @desc    Config one register bank.
* *********************************************************/
bool CMT2300A_ConfigRegBank(u8 base_addr, const u8 bank[], u8 len)
{
    u8 i;
    for(i=0; i<len; i++)
        spi3w.writeReg(i+base_addr, bank[i]);

    return true;
}
/*! ********************************************************
* @name    CMT2300A_SoftReset
* @desc    Soft reset.
* *********************************************************/
void CMT2300A_SoftReset(void)
{
    spi3w.writeReg(0x7F, 0xFF);
    delay(20);
}
void IntRegBank()
{
    CMT2300A_ConfigRegBank(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank       , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank    , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank  , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank        , CMT2300A_TX_BANK_SIZE        );    
	u8 tmp = (~0x07) & spi3w.readReg(CMT2300A_CUS_CMT10);// xosc_aac_code[2:0] = 2
    spi3w.writeReg(CMT2300A_CUS_CMT10, tmp|0x02);
}
/*! ********************************************************
* @name    CMT2300A_IsExist
* @desc    Chip indentify.
* @return  TRUE: chip is exist, FALSE: chip not found
* *********************************************************/
bool CMT2300A_IsExist(void)
{
    u8 back, dat;

    back = spi3w.readReg(CMT2300A_CUS_PKT17);
    spi3w.writeReg(CMT2300A_CUS_PKT17, 0xAA);

    dat = spi3w.readReg(CMT2300A_CUS_PKT17);
    spi3w.writeReg(CMT2300A_CUS_PKT17, back);

    if(0xAA != dat)
    {
        Serial.println("CMT2300A... not reachable!");
        return false;
    }
    return true;
}
/*! ********************************************************
* @name    CMT2300A_GoStby
* @desc    Entry Sleep mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoStby(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY);
}
/*! ********************************************************
* @name    CMT2300A_ClearInterruptFlags
* @desc    Clear all interrupt flags.
* @return  Some interrupt flags
*            CMT2300A_MASK_SL_TMO_EN    |
*            CMT2300A_MASK_RX_TMO_EN    |
*            CMT2300A_MASK_TX_DONE_EN   |
*            CMT2300A_MASK_PREAM_OK_FLG |
*            CMT2300A_MASK_SYNC_OK_FLG  |
*            CMT2300A_MASK_NODE_OK_FLG  |
*            CMT2300A_MASK_CRC_OK_FLG   |
*            CMT2300A_MASK_PKT_OK_FLG
* *********************************************************/
u8 CMT2300A_ClearInterruptFlags(void)
{
    u8 nFlag1, nFlag2;
    u8 nClr1 = 0;
    u8 nClr2 = 0;
    u8 nRet  = 0;
    u8 nIntPolar;
    
    nIntPolar = spi3w.readReg(CMT2300A_CUS_INT1_CTL);
    nIntPolar = (nIntPolar & CMT2300A_MASK_INT_POLAR) ?1 :0;

    nFlag1 = spi3w.readReg(CMT2300A_CUS_INT_FLAG);
    nFlag2 = spi3w.readReg(CMT2300A_CUS_INT_CLR1);
    
    if(nIntPolar) {
        /* Interrupt flag active-low */
        nFlag1 = ~nFlag1;
        nFlag2 = ~nFlag2;
    }

    if(CMT2300A_MASK_LBD_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_LBD_CLR;         /* Clear LBD_FLG */
    }

    if(CMT2300A_MASK_COL_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;    /* Clear COL_ERR_FLG by PKT_DONE_CLR */
    }

    if(CMT2300A_MASK_PKT_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;    /* Clear PKT_ERR_FLG by PKT_DONE_CLR */
    }

    if(CMT2300A_MASK_PREAM_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PREAM_OK_CLR;    /* Clear PREAM_OK_FLG */
        nRet  |= CMT2300A_MASK_PREAM_OK_FLG;    /* Return PREAM_OK_FLG */
    }

    if(CMT2300A_MASK_SYNC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_SYNC_OK_CLR;    /* Clear SYNC_OK_FLG */
        nRet  |= CMT2300A_MASK_SYNC_OK_FLG;    /* Return SYNC_OK_FLG */
    }

    if(CMT2300A_MASK_NODE_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_NODE_OK_CLR;    /* Clear NODE_OK_FLG */
        nRet  |= CMT2300A_MASK_NODE_OK_FLG;    /* Return NODE_OK_FLG */
    }

    if(CMT2300A_MASK_CRC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_CRC_OK_CLR;    /* Clear CRC_OK_FLG */
        nRet  |= CMT2300A_MASK_CRC_OK_FLG;    /* Return CRC_OK_FLG */
    }

    if(CMT2300A_MASK_PKT_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;  /* Clear PKT_OK_FLG */
        nRet  |= CMT2300A_MASK_PKT_OK_FLG;    /* Return PKT_OK_FLG */
    }    

    if(CMT2300A_MASK_SL_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_SL_TMO_CLR;    /* Clear SL_TMO_FLG */
        nRet  |= CMT2300A_MASK_SL_TMO_EN;     /* Return SL_TMO_FLG by SL_TMO_EN */
    }

    if(CMT2300A_MASK_RX_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_RX_TMO_CLR;    /* Clear RX_TMO_FLG */
        nRet  |= CMT2300A_MASK_RX_TMO_EN;     /* Return RX_TMO_FLG by RX_TMO_EN */
    }

    if(CMT2300A_MASK_TX_DONE_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_TX_DONE_CLR;   /* Clear TX_DONE_FLG */
        nRet  |= CMT2300A_MASK_TX_DONE_EN;    /* Return TX_DONE_FLG by TX_DONE_EN */
    }
    
    spi3w.writeReg(CMT2300A_CUS_INT_CLR1, nClr1);
    spi3w.writeReg(CMT2300A_CUS_INT_CLR2, nClr2);

    if(nIntPolar) {
        /* Interrupt flag active-low */
        nRet = ~nRet;
    }

    return nRet;
}
/*! ********************************************************
* @name    CMT2300A_Init
* @desc    Initialize chip status.
* *********************************************************/
void CMT2300A_Init(void)
{
    CMT2300A_SoftReset();
    delay(20);
    
    Serial.print("CMT2300A_GoStby: ");
    Serial.println(CMT2300A_GoStby());

    u8 tmp  = spi3w.readReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;         /* Enable CFG_RETAIN */
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN;        /* Disable RSTN_IN */
    spi3w.writeReg(CMT2300A_CUS_MODE_STA, tmp);

    tmp  = spi3w.readReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN;         /* Enable LOCKING_EN */
    spi3w.writeReg(CMT2300A_CUS_EN_CTL, tmp);
    CMT2300A_ClearInterruptFlags();
}
bool CMT2300A_Int(void)
{
    if(!CMT2300A_IsExist())
        return false;

    //IntGPIO();
    //cmt_spi3_init();
    CMT2300A_Init();

    //IntRegBank();
    //IntRegInterupt();

    //pinMode(CMT2300A_GPIO1_PIN, INPUT);
    //attachInterrupt(CMT2300A_GPIO1_PIN, GPIO1_interrupt_callback, RISING);
    /*if(!CMT2300A_goRX())
        return false;*/
    return true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Load lib for CMT2300A");

    cnt = 0;
    txOk = false;
    spi3w.setup();

    delay(1000);

    CMT2300A_Int();

    Serial.println("start");

    // soft reset
    CMT2300A_SoftReset(); 
    
    // go to standby mode
    if(cmtSwitchStatus(CMT2300A_GO_STBY, CMT2300A_STA_STBY))
        Serial.println("standby mode reached");

    spi3w.readReg(CMT2300A_CUS_MODE_STA); // 0x61
    spi3w.writeReg(CMT2300A_CUS_MODE_STA, 0x52);

    if(0x00 != spi3w.readReg(CMT2300A_CUS_EN_CTL))
        Serial.println("CUS_EN_CTL: read error 2");
    spi3w.writeReg(CMT2300A_CUS_EN_CTL, 0x20);

    if(0xE0 != spi3w.readReg(CMT2300A_CUS_SYS2))
        Serial.println("error 3");
    spi3w.writeReg(CMT2300A_CUS_SYS2, 0x00);

/*    uint8_t tmp;
    tmp = spi3w.readReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;         // Enable CFG_RETAIN
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN;        // Disable RSTN_IN
    spi3w.writeReg(CMT2300A_CUS_MODE_STA, tmp);
*/
    /*spi3w.writeReg(CMT2300A_CUS_MODE_STA, 0x52);

    tmp  = spi3w.readReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN;         // Enable LOCKING_EN
    spi3w.writeReg(CMT2300A_CUS_EN_CTL, tmp);*/


    ///////////////////


    IntRegBank();

    if(0x02 != spi3w.readReg(CMT2300A_CUS_CMT10))
        Serial.println("error 4");
    spi3w.writeReg(CMT2300A_CUS_CMT10, 0x02);

    spi3w.writeReg(CMT2300A_CUS_IO_SEL, 0x20);

    if(0x00 != spi3w.readReg(CMT2300A_CUS_INT1_CTL))
        Serial.println("error 5");
    spi3w.writeReg(CMT2300A_CUS_INT1_CTL, 0x0A);

    if(0x00 != spi3w.readReg(CMT2300A_CUS_INT2_CTL))
        Serial.println("error 6");
    spi3w.writeReg(CMT2300A_CUS_INT2_CTL, 0x07);

    spi3w.writeReg(CMT2300A_CUS_INT_EN, 0x3B);

    spi3w.writeReg(CMT2300A_CUS_PKT10, 0x48);
    spi3w.writeReg(CMT2300A_CUS_PKT11, 0x5A);
    spi3w.writeReg(CMT2300A_CUS_PKT12, 0x48);
    spi3w.writeReg(CMT2300A_CUS_PKT13, 0x4D);
    spi3w.writeReg(CMT2300A_CUS_PKT15, 0x64);

    if(0x00 != spi3w.readReg(CMT2300A_CUS_FIFO_CTL))
        Serial.println("error 7");
    spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x02);

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x10);
    while(0x51 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x02);
    while(0x52 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    spi3w.writeReg(CMT2300A_CUS_RF1, 0x42);
    spi3w.writeReg(CMT2300A_CUS_RF2, 0xA9);
    spi3w.writeReg(CMT2300A_CUS_RF3, 0xA4);
    spi3w.writeReg(CMT2300A_CUS_RF4, 0x8C);
    spi3w.writeReg(CMT2300A_CUS_RF5, 0x42);
    spi3w.writeReg(CMT2300A_CUS_RF6, 0x9E);
    spi3w.writeReg(CMT2300A_CUS_RF7, 0x4B);
    spi3w.writeReg(CMT2300A_CUS_RF8, 0x1C);
    spi3w.writeReg(CMT2300A_CUS_RF11, 0x20);
    spi3w.writeReg(CMT2300A_CUS_RF12, 0x20);

    spi3w.writeReg(CMT2300A_CUS_FSK1, 0xD2);
    spi3w.writeReg(CMT2300A_CUS_FSK2, 0x35);
    spi3w.writeReg(CMT2300A_CUS_FSK3, 0x0C);
    spi3w.writeReg(CMT2300A_CUS_FSK4, 0x0A);

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x10);
    while(0x51 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x02);
    while(0x52 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    spi3w.writeReg(CMT2300A_CUS_CMT4, 0x1D);
    spi3w.writeReg(CMT2300A_CUS_TX8, 0x8A);
    spi3w.writeReg(CMT2300A_CUS_TX9, 0x18);

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x10);
    while(0x51 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x02);
    while(0x52 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
        yield();
    }

    if(0x0A != spi3w.readReg(0x66))
        Serial.println("error 8");

    spi3w.writeReg(CMT2300A_CUS_INT_FLAG, 0x00);

    if(0x00 != spi3w.readReg(CMT2300A_CUS_INT_CLR1))
        Serial.println("error 9");
    spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);

    spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);
    if(0x02 != spi3w.readReg(CMT2300A_CUS_FIFO_CTL))
        Serial.println("error 10");
    spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x02);
    spi3w.writeReg(CMT2300A_CUS_FIFO_CLR, 0x02);
    spi3w.writeReg(CMT2300A_CUS_SYS11, 0x0C);
    spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, 0x01);

    uint8_t cfg0[11] = {
        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xff
    };
    uint8_t cfg1[15] = {
        0x56, UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr),
        UINT32_BYTE_0(wr), 0x02, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg2[15] = {
        0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x0f, 0x14, 0xff
    };
    uint8_t cfg3[11] = {
        0x07, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x00, 0xff
    };
    uint8_t cfg4[15] = {
        0x56, UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu), UINT32_BYTE_0(dtu), UINT32_BYTE_3(dtu), UINT32_BYTE_2(dtu), UINT32_BYTE_1(dtu),
        UINT32_BYTE_0(dtu), 0x01, 0x15, 0x21, 0x21, 0x14, 0xff
    };
    uint8_t i;

    cfg0[10] = crc8(cfg0, 10);
    spi3w.writeFifo(cfg0, 11);
    dumpBuf("cfg0", cfg0, 11);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 5; i++) {
        spi3w.writeFifo(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg3[10] = crc8(cfg3, 10);
    spi3w.writeFifo(cfg3, 11);
    dumpBuf("cfg3", cfg3, 11);

    cfg4[14] = crc8(cfg4, 14);
    spi3w.writeFifo(cfg4, 15);
    dumpBuf("cfg4", cfg4, 15);

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);

    cfg1[14] = crc8(cfg1, 14);
    for(i = 0; i < 3; i++) {
        spi3w.writeFifo(cfg1, 15);
        dumpBuf("cfg1", cfg1, 15);
    }

    cfg2[14] = crc8(cfg2, 14);
    spi3w.writeFifo(cfg2, 15);
    dumpBuf("cfg2", cfg2, 15);
}

//-----------------------------------------------------------------------------
void loop() {
    delay(1000);
    ts++;

    if((++cnt % 5) == 0) {
        spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);
        spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);
        spi3w.writeReg(CMT2300A_CUS_MODE_CTL, 0x02);
        while(0x52 != spi3w.readReg(CMT2300A_CUS_MODE_STA)) {
            yield();
        }

        if(0x0A != spi3w.readReg(CMT2300A_CUS_INT1_CTL))
            Serial.println("error 20");
        if(0x00 != spi3w.readReg(CMT2300A_CUS_INT_FLAG))
            Serial.println("error 21");
        if(0x00 != spi3w.readReg(CMT2300A_CUS_INT_CLR1))
            Serial.println("error 22");


        spi3w.writeReg(CMT2300A_CUS_INT_CLR1, 0x00);
        spi3w.writeReg(CMT2300A_CUS_INT_CLR2, 0x00);

        if(0x02 != spi3w.readReg(CMT2300A_CUS_FIFO_CTL))
            Serial.println("error 23");
        spi3w.writeReg(CMT2300A_CUS_FIFO_CTL, 0x07);

        spi3w.writeReg(CMT2300A_CUS_FIFO_CLR, 0x01);

        if(0x01 != spi3w.readReg(CMT2300A_CUS_PKT14))
            Serial.println("error 24");

        spi3w.writeReg(CMT2300A_CUS_PKT14, 0x01);
        spi3w.writeReg(CMT2300A_CUS_PKT15, 0x1B);

        spi3w.writeReg(CMT2300A_CUS_FREQ_CHNL, 0x21); //0x63
        if(cmtSwitchStatus(CMT2300A_GO_TX, CMT2300A_STA_TX)) {
            Serial.println("tx mode ok");
            txOk = true;

            uint8_t rqst[27] = {
                0x15, UINT32_BYTE_3(wr), UINT32_BYTE_2(wr), UINT32_BYTE_1(wr), UINT32_BYTE_0(wr), 0x00, 0x00, 0x00,
                0x01, 0x80, 0x0B, 0x00, UINT32_BYTE_3(ts), UINT32_BYTE_2(ts), UINT32_BYTE_1(ts), UINT32_BYTE_0(ts),
                0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
                0x88, 0x90, 0xff
            };
            rqst[26] = crc8(rqst, 26);
            spi3w.writeFifo(rqst, 27);
            dumpBuf("request data", rqst, 27);
        }
        else {
            txOk = false;
            Serial.println("can't switch to tx");
        }
    }

    //if(txOk) {
        if(cmtSwitchStatus(CMT2300A_GO_RX, CMT2300A_STA_RX)) {
            //Serial.println("rx mode ok");
            uint8_t buf[27] = {0};
            spi3w.readFifo(buf, 27);

            // only print buffer if some fields are not equal 0
            for(uint8_t i = 0; i < 27; i++) {
                if(buf[i] != 0) {
                    dumpBuf("fifo", buf, 27);
                    break;
                }
            }
        }
    //}
}
