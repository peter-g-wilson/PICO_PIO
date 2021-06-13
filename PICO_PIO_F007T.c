#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "PICO_PIO_F007T_WH1080.pio.h"
#include "queues_for_msgs_and_bits.h"
#include "PICO_PIO_F007T.h"
#include "output_format.h"
#include "uart_IO.h"
#include "string.h"

#define MAX_F007T_CHANNELS    8
#define MAX_F007T_KNWNCHANIDX 4
typedef struct F007TchansStruct {
    uint32_t  chanPrvTim;
    uint      chanVrfyCnt;
    uint      chanUnVrfyCnt;
    uint      chanSmplVrfyCnt;
    uint      chanSmplUnVrfyCnt;
} F007Tchan_t;
volatile F007Tchan_t F007Tchans[MAX_F007T_CHANNELS];

#define F007T_MAXMSGBYTS  6
typedef struct F007TmsgRecStuct {
    uint32_t mRecMsgVrfdDltaTim;
    uint16_t mRecChanID;
    uint8_t  mRecChanIdx;
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
    uint8_t  mRecBuff[F007T_MAXMSGBYTS];
} F007TmsgRec_t;

uint8_t  msgPrvF007T[F007T_MAXMSGBYTS];

#define MAX_F007T_BUFMSGS 16
F007TmsgRec_t msgRecsF007T[MAX_F007T_BUFMSGS];
#define MAX_F007T_BUFWRDS 32
volatile uint32_t F007TrxWrdsBuf[MAX_F007T_BUFWRDS];

volatile bitQue_t F007TbitQ = {
    .pio_id         = pio0,
    .sm_id          = 2,
    .pin_rx         = 16,
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
    .oneBitsCntr    = 0,
    .bQueSmplTotCnt  = 0,
    .bQueSmplOnesCnt = 0,
    .bQueSmplTimDlta = 0,
    .bQueSmplTimPrv  = 0,
    .rxWrdsMax      = MAX_F007T_BUFWRDS,
    .rxWrdBuff      = (void *)&F007TrxWrdsBuf[0]
};  

volatile msgQue_t F007TmsgQ = {
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
    .mQueChkErr  = 0,
    .mQueHdrHits = 0,
    .mQueSmplTimPrv = 0,    
    .mQueSmplTimDlta = 0,    
    .mQueSmplVrfyCnt   = 0, 
    .mQueSmplUnVrfyCnt = 0,
    .mQueSmplHdrHitsCnt = 0,
    .mQueSmplChkErrsCnt = 0,
    .msgBitQueP  = &F007TbitQ,
    .msgMaxRecs  = MAX_F007T_BUFMSGS,
    .mQueSubChan = (void *)&F007Tchans[0],
    .msgRecBuff  = (void *)&msgRecsF007T[0],
    .msgPrvMsg  = &msgPrvF007T[0]
};


/*-----------------------------------------------------------------*/
bool parseF007Tbits_callback(struct repeating_timer *t) {
    const uint8_t lsfrMask[(F007T_MAXMSGBYTS-1)*8] = {
        0x3e, 0x1f, 0x97, 0xd3, 0xf1, 0xe0, 0x70, 0x38,
        0x1c, 0x0e, 0x07, 0x9b, 0xd5, 0xf2, 0x79, 0xa4,  
        0x52, 0x29, 0x8c, 0x46, 0x23, 0x89, 0xdc, 0x6e,
        0x37, 0x83, 0xd9, 0xf4, 0x7a, 0x3d, 0x86, 0x43,  
        0xb9, 0xc4, 0x62, 0x31, 0x80, 0x40, 0x20, 0x10
    };
#define F007T_HDR_MASK      ((uint32_t)0x000FFFFF)
#define F007T_HDR_46        ((uint32_t)0x00000046)
#define F007T_HDR_MATCH46  (((uint32_t)0x000FFD00) | F007T_HDR_46)
#define F007T_HDR_MATCH46X (((uint32_t)0x00000100) | F007T_HDR_46)

    static uint32_t header     = 0;
    static uint     waitMsgHdr = true;
    static uint     bytCnt;
    static uint     bitCnt;
    static uint8_t  chkSum;
    static uint8_t  tmpByt;
    static volatile uint8_t       * msgP;
    static volatile F007TmsgRec_t * msgRecP;
    volatile        F007TmsgRec_t * msgRecs;
    volatile        msgQue_t      * msgQ = (volatile msgQue_t *)t->user_data;
    volatile        bitQue_t      * bitQ = msgQ->msgBitQueP;
    while (tryBitBuf( bitQ )) {
        bool nxtBitIsSet = getNxtBit_isSet( bitQ );
        if (waitMsgHdr) {
            header <<= 1;
            if (nxtBitIsSet) header |= 1;
            if ((uint32_t)(header & F007T_HDR_MASK) == F007T_HDR_MATCH46) {
                msgQ->mQueHdrHits++;
                msgRecs  = (volatile F007TmsgRec_t *)msgQ->msgRecBuff;
                msgRecP  = &msgRecs[msgQ->msgHead];
                msgP     = &msgRecP->mRecBuff[0];
                msgP[0]  = (uint8_t)(header & 0x000000FF);
                tmpByt   = msgP[0];
                chkSum   = 100;
                for (uint i = 0; i < 8; i++) {
                    if (tmpByt & 0x80) chkSum ^= lsfrMask[i];
                    tmpByt <<= 1;
                }
                bytCnt     = 1;
                bitCnt     = 0;
                header     = 0;
                waitMsgHdr = false;
            }            
        } else {
            if (bitCnt == 0) tmpByt = 0;
            tmpByt <<= 1;
            if (nxtBitIsSet) {
                tmpByt |= 1;
                if (bytCnt < (F007T_MAXMSGBYTS-1)) {
                    chkSum ^= lsfrMask[bytCnt*8+bitCnt];
                }
            }
            bitCnt++;
            if (bitCnt >= 8) {
                msgP[bytCnt] = tmpByt;
                bitCnt = 0;
                bytCnt++;
                if (bytCnt >= F007T_MAXMSGBYTS) {
                    F007Tchan_t * chanP = (F007Tchan_t *)msgQ->mQueSubChan;
                    uint32_t tNow  = to_ms_since_boot( get_absolute_time() );
                    uint32_t tDiff = tNow - msgQ->mQueSmplTimPrv;
                    bool doPutNxtMsg  = false;
                    uint8_t  chanIdx;
                    if (msgP[F007T_MAXMSGBYTS-1] == chkSum) {
                        bool prevSame = false;
                        chanIdx = ((msgP[2] >> 4) & 7);
                        msgQ->msgCrntID      = (msgP[0] << 8) | chanIdx;
                        msgRecP->mRecChanIdx = chanIdx;
                        msgRecP->mRecChanID  = msgQ->msgCrntID;
                        if (msgQ->msgHadPrv && (msgQ->msgCrntID == msgQ->msgPrevID) &&
                                               (msgQ->msgPrvChk == chkSum) ) {
                            prevSame = true;
                            if (memcmp(&msgQ->msgPrvMsg[0],(const void *)&msgP[0],F007T_MAXMSGBYTS) != 0) {
                                //prevSame = false;
                                printf("??? prev chksum same but message content different ");
                                for (uint j = 0; j < F007T_MAXMSGBYTS; j++) printf("%02X",msgQ->msgPrvMsg[j]);
                                printf("\n");
                            }
                        }
                        if (prevSame) {
                            msgRecP->mRecVerified = true;
                            msgRecP->mRecMsgVrfdDltaTim  = tNow - F007Tchans[chanIdx].chanPrvTim;
                            F007Tchans[chanIdx].chanPrvTim  = tNow;
                            msgQ->msgVrfyCnt++;
                            chanP[chanIdx].chanVrfyCnt++;
                            msgQ->msgHadPrv = false;
                            doPutNxtMsg = true;
                        } else {
                            msgRecP->mRecVerified = false;
                            msgQ->msgUnVrfyCnt++;
                            chanP[chanIdx].chanUnVrfyCnt++;
                            msgQ->msgHadPrv = true;
                            memcpy(&msgQ->msgPrvMsg[0],(const void *)&msgP[0],F007T_MAXMSGBYTS);
                        }
                        msgQ->msgPrvChk = chkSum;
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
                        for (uint chIDX = 0; chIDX < MAX_F007T_CHANNELS; chIDX++) {
                            chanP[chIDX].chanSmplVrfyCnt   = chanP[chIDX].chanVrfyCnt;
                            chanP[chIDX].chanSmplUnVrfyCnt = chanP[chIDX].chanUnVrfyCnt;
                            chanP[chIDX].chanVrfyCnt   = 0;
                            chanP[chIDX].chanUnVrfyCnt = 0;
                        }
                        msgQ->mQueSmplVrfyCnt   = msgQ->msgVrfyCnt;
                        msgQ->mQueSmplUnVrfyCnt = msgQ->msgUnVrfyCnt;
                        msgQ->msgVrfyCnt   = 0;
                        msgQ->msgUnVrfyCnt = 0;
                    }
                    if (doPutNxtMsg) {
                        msgRecP->mRecBitSSmplTimDlta = bitQ->bQueSmplTimDlta;
                        msgRecP->mRecBitsSmplTotCnt  = bitQ->bQueSmplTotCnt;
                        msgRecP->mRecBitsSmplOnesCnt = bitQ->bQueSmplOnesCnt;
                        msgRecP->mRecSmplTimPrv      = msgQ->mQueSmplTimPrv;
                        msgRecP->mRecSmplTimDlta     = msgQ->mQueSmplTimDlta;
                        msgRecP->mRecSmplHdrHitsCnt  = msgQ->mQueSmplHdrHitsCnt;
                        msgRecP->mRecSmplChkErrsCnt  = msgQ->mQueSmplChkErrsCnt;
                        msgRecP->mRecSmplVrfyCnt     = chanP[chanIdx].chanSmplVrfyCnt;
                        msgRecP->mRecSmplUnVrfyCnt   = chanP[chanIdx].chanSmplUnVrfyCnt;
                        putNxtMsg(msgQ);
                    }
                    waitMsgHdr = true;
                }
            }
        }           
    }
    return true;
}

/*-----------------------------------------------------------------*/
#define F007T_HEXCODES_LEN  (FRMT_CHAN_ID_LEN + 1 + F007T_MAXMSGBYTS*2)
#define F007T_MAXMSGFRMT 15

void decode_F007T_msg( volatile msgQue_t * msgQ, outBuff_t outBuff, outArgs_t * outArgsP ) {
    volatile F007TmsgRec_t * msgRecs = (volatile F007TmsgRec_t *)msgQ->msgRecBuff;
    volatile F007TmsgRec_t * msgRecP = &msgRecs[msgQ->msgTail];
    volatile uint8_t       * msgP    = &msgRecP->mRecBuff[0];
    volatile bitQue_t      * bitQ    = msgQ->msgBitQueP;
    uint  msgId   = msgRecP->mRecChanID;
    uint  chanIdx = msgRecP->mRecChanIdx;
    bool  battLow =  (msgP[2] & 0x80) != 0;
    uint  tmpRaw  = ((msgP[2] & 0xF) << 8) | msgP[3];
    uint  tmpX2   = tmpRaw * 2u * 5u / 9u;
    int   tmpCX10 = tmpX2 & 1 ? (tmpX2+1)/2-400 : tmpX2/2-400;
    float tmpDegC = (float)tmpCX10 * 0.1F;
    int   msgLen;    

    if (tmpDegC < -99.0F) tmpDegC = -99.0F;
    if (tmpDegC > 999.0F) tmpDegC = 999.0F;

    msgLen = snprintf( &outBuff[0], FRMT_CHAN_ID_LEN+2, "%0*X-", FRMT_CHAN_ID_LEN, msgId);
    for (uint i = 0; i < F007T_MAXMSGBYTS; i++) {
        msgLen += snprintf( &outBuff[(FRMT_CHAN_ID_LEN+1)+(i*2)], 2+1, "%02X", msgP[i] );
    }
    msgLen += snprintf( &outBuff[F007T_HEXCODES_LEN], FRMT_TOTAL_LEN - F007T_HEXCODES_LEN + 1,
               "%*.*s-i%c%1d,b:%1d,t:%05.1f%*.*s",
               FRMT_HEXCODS_LEN - F007T_MAXMSGBYTS*2, FRMT_HEXCODS_LEN - F007T_MAXMSGBYTS*2, dash_padding,
               chanIdx > MAX_F007T_KNWNCHANIDX ? '?' : ':',chanIdx+1,battLow,tmpDegC,
               FRMT_DECODES_LEN - F007T_MAXMSGFRMT,   FRMT_DECODES_LEN - F007T_MAXMSGFRMT,   dash_padding);

    msgLen += 3;
    FRMT_PRINT_PATHETIC_EXCUSE(msgId,msgLen);

    outBuff[msgLen-3] = 13;
    outBuff[msgLen-2] = 10;
    outBuff[msgLen-1] = 0;

    outArgsP->oArgMsgLen = msgLen;

    outArgsP->oArgChanID = chanIdx + 1;
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
outBuff_t F007Tmsgcods;

bool F007T_tryMsgBuf( void ) {
    return tryMsgBuf( &F007TmsgQ );
}

void F007T_doMsgBuf( void ) {
    outArgs_t outArgs;
    decode_F007T_msg( &F007TmsgQ, F007Tmsgcods, &outArgs );
    freeLastMsg( &F007TmsgQ );
    uartIO_buffSend(&F007Tmsgcods[0],outArgs.oArgMsgLen);
    print_msg( F007Tmsgcods, &outArgs );
}

/*-----------------------------------------------------------------*/
void F007T_init( uint32_t parseRptTime, uint32_t fifoRptTime ) {
    volatile msgQue_t * msgQ = &F007TmsgQ;
    volatile bitQue_t * bitQ = msgQ->msgBitQueP;
    //gpio_init(                        bitQ->pin_rx );
    //gpio_set_dir(                     bitQ->pin_rx, GPIO_IN );
    //gpio_pull_down(                   bitQ->pin_rx );
    pio_gpio_init( bitQ->pio_id, bitQ->pin_rx );

    uint offset_prog = pio_add_program( bitQ->pio_id, &manchWithDelay_program );
    pio_sm_config cfg_prog = manchWithDelay_program_get_default_config( offset_prog );    

    sm_config_set_in_pins(   &cfg_prog, bitQ->pin_rx );
    sm_config_set_jmp_pin(   &cfg_prog, bitQ->pin_rx );
    sm_config_set_in_shift(  &cfg_prog, false, true, 32 );
    sm_config_set_fifo_join( &cfg_prog, PIO_FIFO_JOIN_RX );
    sm_config_set_clkdiv(    &cfg_prog, 2543.0F  );

    pio_sm_init(        bitQ->pio_id, bitQ->sm_id, offset_prog, &cfg_prog );
    pio_sm_clear_fifos( bitQ->pio_id, bitQ->sm_id);
    pio_sm_exec(        bitQ->pio_id, bitQ->sm_id, pio_encode_set(pio_x, 1));
    pio_sm_set_enabled( bitQ->pio_id, bitQ->sm_id, true);

    msgQ->msgQLock = spin_lock_instance( next_striped_spin_lock_num() );
    
    add_repeating_timer_ms( parseRptTime, parseF007Tbits_callback, (void *) &F007TmsgQ, 
                                 (repeating_timer_t *)&msgQ->rptTmr );
    add_repeating_timer_ms( fifoRptTime, poll_FIFO_callback, (void *) &F007TbitQ, 
                                 (repeating_timer_t *) &bitQ->rptTmr );
}
void F007T_uninit( void ) {
    volatile msgQue_t * msgQ = &F007TmsgQ;
    volatile bitQue_t * bitQ = msgQ->msgBitQueP;
    bool cancelled = cancel_repeating_timer( (repeating_timer_t *)&bitQ->rptTmr );
    cancelled      = cancel_repeating_timer( (repeating_timer_t *)&msgQ->rptTmr );
    pio_sm_set_enabled( bitQ->pio_id, bitQ->sm_id, false );
}
