#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "output_format.h"
#include "queues_for_msgs_and_bits.h"

const uint ON_BOARD_LED = PICO_DEFAULT_LED_PIN;
const uint8_t dash_padding[41] = "----------------------------------------";

void print_msg( outBuff_t outBuff, outArgs_t * outArgsP ) {
    uint    totSmplMsgs = outArgsP->oArgSmplVrfyCnt + outArgsP->oArgSmplUnVrfyCnt;
    uint    totGoodHits = outArgsP->oArgSmplHdrHitsCnt - outArgsP->oArgSmplChkErrsCnt;
    uint        totBits = outArgsP->oArgBitSmplTotCnt*32;
    float  vrfdSmplPrct = totSmplMsgs == 0 ?
                          0.0F : (float)outArgsP->oArgSmplVrfyCnt*100.0F/(float)totSmplMsgs;
    float  hitsGoodPrct = outArgsP->oArgSmplHdrHitsCnt == 0 ?
                          0.0F : (float)totGoodHits*100.0F/(float)outArgsP->oArgSmplHdrHitsCnt;
    float  onesBitsPrct = totBits == 0 ?
                          0.0F : (float)outArgsP->oArgBitSmplOnesCnt*100.0F/(float)totBits;

    float  msgDlta = (float)outArgsP->oArgMsgVrfdDltaTim*0.001F;
    float  msgRate;
    float  hdrRate;
    if (outArgsP->oArgSmplTimDlta == 0) {
        msgRate = 0.0F;
        hdrRate = 0.0F;
    } else {
        float smplTimDlta = ((float)outArgsP->oArgSmplTimDlta*0.001F)/((float)MSGQUE_SMPL_PERIOD_MINS*60.0F);
        msgRate = (float)outArgsP->oArgSmplVrfyCnt / smplTimDlta;
        hdrRate = (float)outArgsP->oArgSmplHdrHitsCnt / smplTimDlta;
    }
    float    bitRateSec = outArgsP->oArgBitSmplTimDlta == 0 ? 
                          0.0F : (float)totBits/((float)outArgsP->oArgBitSmplTimDlta*0.001F);
    printf( 
        "%08X %*.*s "\
        "Msg %1X %07.1f s %07.1f %05.1f %% "\
        "Hdr %07.1f %05.1f %% "\
        "Bit %07.1f /s %05.1f %% "\
        "HiWtr %02d %02d %02d\n",
        outArgsP->oArgSmplTimPrv, outArgsP->oArgMsgLen-3, outArgsP->oArgMsgLen-3, outBuff,
        outArgsP->oArgChanID,   msgDlta,  msgRate,  vrfdSmplPrct,
        hdrRate,                hitsGoodPrct,
        bitRateSec,             onesBitsPrct,
        outArgsP->oArgMsgHiWtr, outArgsP->oArgWrdHiWtr, outArgsP->oArgFiFoHiWtr ); 
}
