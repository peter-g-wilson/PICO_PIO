#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

#include "PICO_PIO_F007T_WH1080.pio.h"
#include "queues_for_msgs_and_bits.h"
#include "PICO_PIO_WH1080.h"
#include "output_format.h"
#include "uart_IO.h"
#include "string.h"

/*-----------------------------------------------------------------*/
#define WH1080_MAXMSGBYTS  11
typedef struct WH1080msgRecStuct {
    uint32_t mRecMsgVrfdDltaTim;
    uint16_t mRecChanID;
    bool     mRecVerified;
    uint32_t mRecSmplTimPrv;    
    uint32_t mRecSmplTimDlta;    
    uint     mRecSmplVrfyCnt;
    uint     mRecSmplUnVrfyCnt;
    uint     mRecSmplHdrHitsCnt;
    uint     mRecSmplChkErrsCnt;
    uint32_t mRecBitSSmplTimDlta;    
    uint     mRecBitsSmplTotCnt;
    uint     mRecBitsSmplOnesCnt;
    uint8_t  mRecBuff[WH1080_MAXMSGBYTS];
} WH1080msgRec_t;

uint8_t  msgPrvWH1080[WH1080_MAXMSGBYTS];

#define MAX_WH1080_BUFMSGS 16
WH1080msgRec_t msgRecsWH1080[MAX_WH1080_BUFMSGS];
#define MAX_WH1080_BUFWRDS 32
volatile uint32_t WH1080rxWrdsBuf[MAX_WH1080_BUFWRDS];

volatile bitQue_t WH1080bitQ = {
    .pio_id         = pio0,
    .sm_id          = 1,
    .pin_rx         = 18,
    .rxWrdTail      = 0,
    .rxWrdHead      = 0,
    .rxWrdCntr      = 0,
    .rxWrdHiWater   = 0,
    .rxWrdOvrRun    = 0,
    .rxWrdUndRun    = 0,
    .rxBitUndRun    = 0,
    .rxBitCnt       = 0,
    .rxBitsBuf      = 0,
    .fifoHiWater    = 0,
    .totBitsCntr    = 0,
    .totBitsCntr    = 0,
    .oneBitsCntr    = 0,
    .bQueSmplTotCnt  = 0,
    .bQueSmplOnesCnt  = 0,
    .bQueSmplTimDlta = 0,
    .bQueSmplTimPrv      = 0,
    .rxWrdsMax      = MAX_WH1080_BUFWRDS,
    .rxWrdBuff      = (void *)&WH1080rxWrdsBuf[0]
};  

volatile msgQue_t WH1080msgQ = {
    .msgTail     = 0,
    .msgHead     = 0,
    .msgCntr     = 0,
    .msgOvrRun   = 0,
    .msgUndRun   = 0,
    .msgsHiWater = 0,
    .msgPrvChk   = 0,
    .msgPrvTime  = 0,
    .msgHadPrv   = false,
    .msgPrevID   = 0,
    .msgCrntID   = 0,
    .msgVrfyCnt  = 0,
    .msgUnVrfyCnt= 0,
    .mQueHdrHits = 0,
    .mQueChkErr  = 0,
    .mQueSmplTimPrv = 0,    
    .mQueSmplTimDlta = 0,    
    .mQueSmplChkErrsCnt = 0,
    .mQueSmplHdrHitsCnt = 0,
    .mQueSmplVrfyCnt   = 0, 
    .mQueSmplUnVrfyCnt = 0,
    .msgBitQueP  = &WH1080bitQ,
    .msgMaxRecs  = MAX_WH1080_BUFMSGS,
    .msgRecBuff  = (void *)&msgRecsWH1080[0],
    .msgPrvMsg  = &msgPrvWH1080[0]
};

/*-----------------------------------------------------------------*/
bool parseWH1080bits_callback(struct repeating_timer *t) {
    static const uint8_t crctab[256] = {
        0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
        0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
        0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
        0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
        0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
        0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
        0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
        0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
        0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
        0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
        0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
        0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
        0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
        0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
        0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
        0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
    };
#define WH1080_HDR_MASK    ((uint32_t)0x0003FEFF)
#define WH1080_HDR_MATCH   ((uint32_t)0x0003FAFD)
#define WH1080_HDR_FIXBITS ((uint32_t)0x000C0000)

    static uint32_t   header     = 0;
    static uint       waitMsgHdr = true;
    static uint       bytCnt;
    static uint       bitCnt;
    static uint8_t    chkSum;
    static volatile uint8_t        * msgP;
    static volatile WH1080msgRec_t * msgRecP;
    volatile        WH1080msgRec_t * msgRecs;
    volatile        msgQue_t       * msgQ = (volatile msgQue_t *)t->user_data;
    volatile        bitQue_t       * bitQ = msgQ->msgBitQueP;
    while (tryBitBuf( bitQ )) {
        bool nxtBitIsSet = getNxtBit_isSet( bitQ );
        if (waitMsgHdr) {
            header <<= 1;
            if (nxtBitIsSet) header |= 1;
            if ((uint32_t)(header & WH1080_HDR_MASK) == WH1080_HDR_MATCH) {
                msgQ->mQueHdrHits++;
                msgRecs    = (volatile WH1080msgRec_t *)msgQ->msgRecBuff;
                msgRecP    = &msgRecs[msgQ->msgHead];
                msgP       = &msgRecP->mRecBuff[0];
                header    |= WH1080_HDR_FIXBITS;
                msgP[0]    = (uint8_t)((header & 0x000FF000) >> 12);
                msgP[1]    = (uint8_t)((header & 0x00000FF0) >> 4);
                msgP[2]    = (uint8_t)( header & 0x0000000F);
                bytCnt     = 2;
                bitCnt     = 4;
                chkSum     = 0;
                chkSum     = crctab[msgP[1] ^ chkSum];
                header     = 0;
                waitMsgHdr = false;
            }            
        } else {
            msgP[bytCnt] <<= 1;
            if (nxtBitIsSet) msgP[bytCnt] |= 1;
            bitCnt++;
            if (bitCnt >= 8) {
                chkSum = crctab[msgP[bytCnt] ^ chkSum];
                bitCnt = 0;
                bytCnt++;
                if (bytCnt >= WH1080_MAXMSGBYTS) {
                    uint32_t tNow  = to_ms_since_boot( get_absolute_time() );
                    uint32_t tDiff = tNow - msgQ->mQueSmplTimPrv;
                    bool doPutNxtMsg = false;
                    if (chkSum == 0) {
                        uint16_t chanID = 0xFD00 | ((msgP[1] & 0xF0) >> 4);
                        bool prevSame = false;
                        msgQ->msgCrntID     = chanID;
                        msgRecP->mRecChanID = msgQ->msgCrntID;
                        if (msgQ->msgHadPrv && (msgQ->msgCrntID == msgQ->msgPrevID) &&
                                               (msgQ->msgPrvChk == msgP[WH1080_MAXMSGBYTS-1]) ) {
                            prevSame = true; // FFBFDFF7DFFFFFFD71BFFF-?
                            if (memcmp(&msgQ->msgPrvMsg[0],(const void *)&msgP[0],WH1080_MAXMSGBYTS) != 0) {
                                //prevSame = false;
                                printf("??? prev chksum same but message different ");
                                for (uint j = 0; j < WH1080_MAXMSGBYTS; j++) printf("%02X",msgQ->msgPrvMsg[j]);
                                printf("\n");
                            }
                        }
                        if (prevSame) {
                            msgRecP->mRecVerified = true;
                            msgRecP->mRecMsgVrfdDltaTim = tNow - msgQ->msgPrvTime;
                            msgQ->msgPrvTime = tNow;
                            msgQ->msgVrfyCnt++;
                            msgQ->msgHadPrv = false;
                            doPutNxtMsg = true;
                        } else {
                            msgRecP->mRecVerified = false;
                            msgQ->msgUnVrfyCnt++;
                            msgQ->msgHadPrv = true;
                            memcpy(&msgQ->msgPrvMsg[0],(const void *)&msgP[0],WH1080_MAXMSGBYTS);
                        }
                        msgQ->msgPrvChk = msgP[WH1080_MAXMSGBYTS-1];
                        msgQ->msgPrevID = msgQ->msgCrntID;
                    } else {
                        msgQ->mQueChkErr++;
                        msgQ->msgHadPrv = false;
                    }
                    if (tDiff > MSGQUE_SMPL_PERIOD_MINS*60*1000) {
                        msgQ->mQueSmplTimPrv  = tNow;
                        msgQ->mQueSmplTimDlta = tDiff;
                        msgQ->mQueSmplHdrHitsCnt = msgQ->mQueHdrHits;
                        msgQ->mQueSmplChkErrsCnt = msgQ->mQueChkErr;
                        msgQ->mQueHdrHits = 0;
                        msgQ->mQueChkErr  = 0;
                        msgQ->mQueSmplVrfyCnt   = msgQ->msgVrfyCnt;
                        msgQ->mQueSmplUnVrfyCnt = msgQ->msgUnVrfyCnt;
                        msgQ->msgVrfyCnt   = 0;
                        msgQ->msgUnVrfyCnt = 0;
                    }
                    msgRecP->mRecBitSSmplTimDlta  = bitQ->bQueSmplTimDlta;
                    msgRecP->mRecBitsSmplTotCnt  = bitQ->bQueSmplTotCnt;
                    msgRecP->mRecBitsSmplOnesCnt = bitQ->bQueSmplOnesCnt;
                    msgRecP->mRecSmplTimPrv      = msgQ->mQueSmplTimPrv;
                    msgRecP->mRecSmplTimDlta     = msgQ->mQueSmplTimDlta;
                    msgRecP->mRecSmplHdrHitsCnt  = msgQ->mQueSmplHdrHitsCnt;
                    msgRecP->mRecSmplChkErrsCnt  = msgQ->mQueSmplChkErrsCnt;
                    msgRecP->mRecSmplVrfyCnt     = msgQ->mQueSmplVrfyCnt;
                    msgRecP->mRecSmplUnVrfyCnt   = msgQ->mQueSmplUnVrfyCnt;
                    if (doPutNxtMsg) putNxtMsg(msgQ);
                    waitMsgHdr = true;
                }
            }
        }           
    }
    return true;
}

/*-----------------------------------------------------------------*/
#define WH1080_HEXCODES_LEN  (FRMT_CHAN_ID_LEN + 1 + WH1080_MAXMSGBYTS*2)
#define WH1080_MAX_FD0B_FRMT 27
#define WH1080_MAX_FD0A_FRMT 49
#define WH1080_UNKWN_FRMT     1

void decode_WH1080_msg( volatile msgQue_t * msgQ, outBuff_t outBuff, outArgs_t * outArgsP ) {
    volatile WH1080msgRec_t * msgRecs = (volatile WH1080msgRec_t *)msgQ->msgRecBuff;
    volatile WH1080msgRec_t * msgRecP = &msgRecs[msgQ->msgTail];
    volatile uint8_t        * msgP    = &msgRecP->mRecBuff[0];
    volatile bitQue_t       * bitQ    = msgQ->msgBitQueP;
    uint msgId = msgRecP->mRecChanID;
    bool validVals = false;
    int  msgLen;

    msgLen = snprintf( &outBuff[0], FRMT_CHAN_ID_LEN+2, "%0*X-", FRMT_CHAN_ID_LEN, msgId);
    for (uint i = 0; i < WH1080_MAXMSGBYTS; i++) {
        msgLen += snprintf( &outBuff[(FRMT_CHAN_ID_LEN+1)+(i*2)], 2+1, "%02X", msgP[i] );
    }
    if  (msgId == 0xFD0B) {
        uint8_t *sTyp = ((msgP[2] & 0x0F) == 10) ? "DCF77" : "?????"; // 5 
        uint     tHrs = ((msgP[3] & 0x30) >> 4)*10 + (msgP[3] & 0x0F);
        uint     tMin = ((msgP[4] & 0xF0) >> 4)*10 + (msgP[4] & 0x0F);
        uint     tSec = ((msgP[5] & 0xF0) >> 4)*10 + (msgP[5] & 0x0F);
        uint     tYrs = ((msgP[6] & 0xF0) >> 4)*10 + (msgP[6] & 0x0F) + 2000;
        uint     tMth = ((msgP[7] & 0x10) >> 4)*10 + (msgP[7] & 0x0F);
        uint     tDay = ((msgP[8] & 0xF0) >> 4)*10 + (msgP[8] & 0x0F);
        validVals = !( (tYrs > 2099) || (tMth < 1)  || (tMth > 12) || (tDay < 1) || (tDay > 31) || 
                         (tHrs > 23) || (tMin > 59) || (tSec > 59) ); 
        if (validVals) {
            msgLen += snprintf( &outBuff[WH1080_HEXCODES_LEN], FRMT_TOTAL_LEN - WH1080_HEXCODES_LEN + 1,
                   "%*.*s-i:%5.5s,%04d-%02d-%02d,%02d:%02d:%02d%*.*s",
                   FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2,  FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2,  dash_padding,
                   sTyp, tYrs, tMth, tDay, tHrs, tMin, tSec,
                   FRMT_DECODES_LEN - WH1080_MAX_FD0B_FRMT, FRMT_DECODES_LEN - WH1080_MAX_FD0B_FRMT, dash_padding);
        }
    
    } else if (msgId == 0xFD0A) {
        uint16_t tempRaw    = ((msgP[2] & 0x0F) << 8) | 
                                msgP[3];
        uint8_t  humid      =   msgP[4];
        uint8_t  wndAvg     =   msgP[5];
        uint8_t  wndGst     =   msgP[6];
        uint16_t rainRaw    = ((msgP[7] & 0x0F) << 8) |
                                msgP[8];
        bool     battSts    =  (msgP[9] & 0xF0) > 0;
        uint8_t  wndDir     =   msgP[9] & 0x0F;
        float    tempDegC   = ((float)((int16_t)tempRaw - 400))*0.1F;
        float    rain_mm    =  (float)rainRaw*0.3F;
        float    wndAvg_mps =  (float)wndAvg*0.34F;
        float    wndGst_mps =  (float)wndGst*0.34F;

        if (tempDegC   < -99.0F)  tempDegC =  -99.0F;
        if (tempDegC   > 999.0F)  tempDegC =  999.0F;
        if (rain_mm    > 9999.0F)  rain_mm = 9999.0F;
        if (wndAvg_mps > 99.9F) wndAvg_mps =   99.9F;
        if (wndGst_mps > 99.9F) wndGst_mps =   99.9F;
        validVals = true;

        msgLen += snprintf( &outBuff[WH1080_HEXCODES_LEN], FRMT_TOTAL_LEN - WH1080_HEXCODES_LEN + 1,
               "%*.*s-i:0,b:%1d,t:%05.1f,h:%03d,r:%06.1f,a:%04.1f,g:%04.1f,c:%02d%*.*s",
               FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2,  FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2,  dash_padding,
               battSts, tempDegC, humid, rain_mm, wndAvg_mps, wndGst_mps, wndDir,
               FRMT_DECODES_LEN - WH1080_MAX_FD0A_FRMT, FRMT_DECODES_LEN - WH1080_MAX_FD0A_FRMT, dash_padding);

    }
    if (!validVals) {
        msgLen += snprintf( &outBuff[WH1080_HEXCODES_LEN], FRMT_TOTAL_LEN - WH1080_HEXCODES_LEN + 1,
               "%*.*s-?%*.*s",
               FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2, FRMT_HEXCODS_LEN - WH1080_MAXMSGBYTS*2, dash_padding,
               FRMT_DECODES_LEN - WH1080_UNKWN_FRMT,   FRMT_DECODES_LEN - WH1080_UNKWN_FRMT,   dash_padding);
    }
    msgLen += 3;
    FRMT_PRINT_PATHETIC_EXCUSE(msgId,msgLen);

    outBuff[msgLen-3] = 13;
    outBuff[msgLen-2] = 10;
    outBuff[msgLen-1] = 0;

    outArgsP->oArgMsgLen = msgLen;
    
    outArgsP->oArgChanID = msgId & 0x000F;
    outArgsP->oArgMsgVrfdDltaTim = msgRecP->mRecMsgVrfdDltaTim;

    outArgsP->oArgSmplTimPrv     = msgRecP->mRecSmplTimPrv;
    outArgsP->oArgSmplTimDlta    = msgRecP->mRecSmplTimDlta;
    outArgsP->oArgSmplVrfyCnt    = msgRecP->mRecSmplVrfyCnt;
    outArgsP->oArgSmplUnVrfyCnt  = msgRecP->mRecSmplUnVrfyCnt;
    outArgsP->oArgSmplHdrHitsCnt = msgRecP->mRecSmplHdrHitsCnt;
    outArgsP->oArgSmplChkErrsCnt = msgRecP->mRecSmplChkErrsCnt;

    outArgsP->oArgBitSmplTimDlta = msgRecP->mRecBitSSmplTimDlta;
    outArgsP->oArgBitSmplTotCnt  = msgRecP->mRecBitsSmplTotCnt;
    outArgsP->oArgBitSmplOnesCnt = msgRecP->mRecBitsSmplOnesCnt;

    outArgsP->oArgMsgHiWtr  = msgQ->msgsHiWater;
    outArgsP->oArgWrdHiWtr  = bitQ->rxWrdHiWater;
    outArgsP->oArgFiFoHiWtr = bitQ->fifoHiWater; 
}

/*-----------------------------------------------------------------*/
outBuff_t WH1080msgcods;

bool WH1080_tryMsgBuf( void ) {
    return tryMsgBuf( &WH1080msgQ );;
}

void WH1080_doMsgBuf( void ) {
    outArgs_t outArgs;
    decode_WH1080_msg( &WH1080msgQ, WH1080msgcods, &outArgs );
    freeLastMsg( &WH1080msgQ );
    uartIO_buffSend(&WH1080msgcods[0],outArgs.oArgMsgLen);
    print_msg( WH1080msgcods, &outArgs );
}

/*-----------------------------------------------------------------*/
void WH1080_init( uint32_t parseRptTime, uint32_t fifoRptTime ) {
    volatile msgQue_t * msgQ = &WH1080msgQ;
    volatile bitQue_t * bitQ = msgQ->msgBitQueP;
    //gpio_init(                        bitQ->pin_rx );
    //gpio_set_dir(                     bitQ->pin_rx, GPIO_IN );
    //gpio_pull_down(                   bitQ->pin_rx );
    pio_gpio_init( bitQ->pio_id, bitQ->pin_rx );

    uint offset_prog = pio_add_program( bitQ->pio_id, &PWMpulseBits_program );
    pio_sm_config cfg_prog = PWMpulseBits_program_get_default_config( offset_prog );    

    sm_config_set_in_pins(   &cfg_prog, bitQ->pin_rx );
    sm_config_set_jmp_pin(   &cfg_prog, bitQ->pin_rx );
    sm_config_set_in_shift(  &cfg_prog, false, true, 0 );
    sm_config_set_fifo_join( &cfg_prog, PIO_FIFO_JOIN_RX );
    sm_config_set_clkdiv(    &cfg_prog, 625.0F  );

    pio_sm_init(        bitQ->pio_id, bitQ->sm_id, offset_prog, &cfg_prog );
    pio_sm_clear_fifos( bitQ->pio_id, bitQ->sm_id);
    pio_sm_set_enabled( bitQ->pio_id, bitQ->sm_id, true);

    msgQ->msgQLock = spin_lock_instance( next_striped_spin_lock_num() );
    
    add_repeating_timer_ms( parseRptTime, parseWH1080bits_callback, (void *) &WH1080msgQ, 
                                 (repeating_timer_t *)&msgQ->rptTmr );
    add_repeating_timer_ms( fifoRptTime, poll_FIFO_callback, (void *) &WH1080bitQ, 
                                 (repeating_timer_t *) &bitQ->rptTmr );
}
void WH1080_uninit( void ) {
    volatile msgQue_t * msgQ = &WH1080msgQ;
    volatile bitQue_t * bitQ = msgQ->msgBitQueP;
    bool cancelled = cancel_repeating_timer( (repeating_timer_t *)&bitQ->rptTmr );
    cancelled      = cancel_repeating_timer( (repeating_timer_t *)&msgQ->rptTmr );
    pio_sm_set_enabled( bitQ->pio_id, bitQ->sm_id, false );
}
