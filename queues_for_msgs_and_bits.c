#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

#include "queues_for_msgs_and_bits.h"

void putNxtMsg( volatile msgQue_t * msgQ ) {
    uint32_t flags = spin_lock_blocking( msgQ->msgQLock );
    if (msgQ->msgCntr >= msgQ->msgMaxRecs) {
        msgQ->msgOvrRun++;
        msgQ->msgCntr = msgQ->msgMaxRecs;
    } else {
        if (++msgQ->msgHead >= msgQ->msgMaxRecs) msgQ->msgHead = 0;
        msgQ->msgCntr++;
        if (msgQ->msgCntr > msgQ->msgsHiWater) msgQ->msgsHiWater = msgQ->msgCntr;
    }
    spin_unlock(msgQ->msgQLock, flags);
}
void freeLastMsg( volatile msgQue_t * msgQ ) {
    uint32_t flags = spin_lock_blocking( msgQ->msgQLock );
    if (msgQ->msgCntr > 0) {
        msgQ->msgCntr--;
        if (++msgQ->msgTail >= msgQ->msgMaxRecs) msgQ->msgTail = 0;
    } else {
        msgQ->msgUndRun++;
    }
    spin_unlock(msgQ->msgQLock, flags);
}
bool tryMsgBuf( volatile msgQue_t * msgQ ) {
    return msgQ->msgCntr > 0; 
}

/*-----------------------------------------------------------------*/
void putNxtWrd( volatile bitQue_t * bitQ, uint32_t nxtWrd ) {
    volatile uint32_t * wrdsBuf  = (volatile uint32_t *)bitQ->rxWrdBuff;
    wrdsBuf[bitQ->rxWrdHead] = nxtWrd;
    if (++bitQ->rxWrdHead >= bitQ->rxWrdsMax) bitQ->rxWrdHead = 0;
    if (bitQ->rxWrdCntr >= bitQ->rxWrdsMax) {
        bitQ->rxWrdOvrRun++;
        bitQ->rxWrdCntr = bitQ->rxWrdsMax;
        if (++bitQ->rxWrdTail >= bitQ->rxWrdsMax) bitQ->rxWrdTail = 0;
     } else {
        bitQ->rxWrdCntr++;
        if (bitQ->rxWrdCntr > bitQ->rxWrdHiWater) bitQ->rxWrdHiWater = bitQ->rxWrdCntr;
    }
 }
uint32_t getNxtWrd( volatile bitQue_t * bitQ ) {
    volatile uint32_t * wrdsBuf  = (volatile uint32_t *)bitQ->rxWrdBuff;
    uint wrdOffSet = bitQ->rxWrdTail;
    if (++bitQ->rxWrdTail >= bitQ->rxWrdsMax) bitQ->rxWrdTail = 0;
    if (bitQ->rxWrdCntr > 0) bitQ->rxWrdCntr--;
                        else bitQ->rxWrdUndRun++;
    return wrdsBuf[wrdOffSet];
}
bool tryWrdBuf( volatile bitQue_t * bitQ ) {
    return bitQ->rxWrdCntr > 0; 
}
bool poll_FIFO_callback(struct repeating_timer *t) {
    volatile bitQue_t * bitQ = (volatile bitQue_t *)t->user_data;
    uint fifoCntr = 0;
    while (!pio_sm_is_rx_fifo_empty(bitQ->pio_id, bitQ->sm_id)) {
        putNxtWrd( bitQ, pio_sm_get(bitQ->pio_id, bitQ->sm_id) );
        fifoCntr++;
    }
    if (fifoCntr > bitQ->fifoHiWater) bitQ->fifoHiWater = fifoCntr;
    return true;
}
bool tryBitBuf( volatile bitQue_t * bitQ ) {
    if (bitQ->rxBitCnt == 0) {
        if (tryWrdBuf( bitQ )) {
            bitQ->rxBitsBuf = getNxtWrd( bitQ );
            bitQ->rxBitCnt = 32;
            bitQ->totBitsCntr++;
            for (uint i = 0; i < 32; i++)
                if (bitQ->rxBitsBuf & ((uint32_t)1U << i)) bitQ->oneBitsCntr++; 
            uint32_t tNow  = to_ms_since_boot( get_absolute_time() );
            uint32_t tDiff = tNow - bitQ->bQueSmplTimPrv;
            if (tDiff > MSGQUE_BIT_PERIOD_SECS*1000) {
                bitQ->bQueSmplTotCnt  = bitQ->totBitsCntr;
                bitQ->bQueSmplOnesCnt = bitQ->oneBitsCntr;
                bitQ->bQueSmplTimDlta = tDiff;
                bitQ->bQueSmplTimPrv  = tNow;
                bitQ->totBitsCntr = 0;
                bitQ->oneBitsCntr = 0;
            }
        }
    }
    return bitQ->rxBitCnt > 0;
}
bool getNxtBit_isSet( volatile bitQue_t * bitQ ) {
    bool isBitSet = (bitQ->rxBitsBuf & (uint32_t)0x80000000) != 0;
    bitQ->rxBitsBuf <<= 1;
    if (bitQ->rxBitCnt > 0) bitQ->rxBitCnt--;
                        else bitQ->rxBitUndRun++;
    return isBitSet;
}
