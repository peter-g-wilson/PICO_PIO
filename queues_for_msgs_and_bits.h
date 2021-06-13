
#ifndef  QUEUES_FOR_MSGS_AND_BITS_H

#define MSGQUE_BIT_PERIOD_SECS     60
#define MSGQUE_SMPL_PERIOD_MINS  30
typedef struct bitQueStruct {
    PIO        pio_id;
    const uint sm_id;
    const uint pin_rx;
    repeating_timer_t rptTmr;
    uint     rxWrdTail;
    uint     rxWrdHead;
    uint     rxWrdCntr;
    uint     rxWrdHiWater;
    uint     rxWrdOvrRun;
    uint     rxWrdUndRun;
    uint     rxBitUndRun;
    uint     rxBitCnt;
    uint32_t rxBitsBuf;
    uint     fifoHiWater;
    uint     totBitsCntr;
    uint     oneBitsCntr;
    uint     bQueSmplTotCnt;
    uint     bQueSmplOnesCnt;
    uint32_t bQueSmplTimDlta;
    uint32_t bQueSmplTimPrv;
    uint     rxWrdsMax;
    void   * rxWrdBuff;
} bitQue_t;

typedef struct msgQueStruct {
    uint     msgTail;
    uint     msgHead;
    uint     msgCntr;
    uint     msgOvrRun;
    uint     msgUndRun;
    uint     msgsHiWater;
    uint     msgVrfyCnt;
    uint     msgUnVrfyCnt;
    uint     mQueChkErr;
    uint     mQueHdrHits;
    uint32_t mQueSmplTimPrv;
    uint32_t mQueSmplTimDlta;
    uint     mQueSmplVrfyCnt; 
    uint     mQueSmplUnVrfyCnt;
    uint     mQueSmplChkErrsCnt;
    uint     mQueSmplHdrHitsCnt;
    uint     msgMaxRecs;
    uint8_t  msgPrvChk;
    bool     msgHadPrv;
    uint16_t msgPrevID;
    uint16_t msgCrntID;
    uint32_t msgPrvTime;
    repeating_timer_t   rptTmr;
    spin_lock_t       * msgQLock;
    volatile bitQue_t * msgBitQueP;   
    void *   mQueSubChan;
    void *   msgRecBuff;
    uint8_t * msgPrvMsg;
} msgQue_t;

/*-----------------------------------------------------------------*/
extern void putNxtMsg( volatile msgQue_t * msgQ );
extern void freeLastMsg( volatile msgQue_t * msgQ );
extern bool tryMsgBuf( volatile msgQue_t * msgQ );

/*-----------------------------------------------------------------*/
extern void putNxtWrd( volatile bitQue_t * bitQ, uint32_t nxtWrd );
extern uint32_t getNxtWrd( volatile bitQue_t * bitQ );
extern bool tryWrdBuf( volatile bitQue_t * bitQ );
extern bool poll_FIFO_callback(struct repeating_timer *t);
extern bool tryBitBuf( volatile bitQue_t * bitQ );
extern bool getNxtBit_isSet( volatile bitQue_t * bitQ );

#define QUEUES_FOR_MSGS_AND_BITS_H
#endif
