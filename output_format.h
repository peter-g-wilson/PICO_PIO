
#ifndef  OUTPUT_FORMAT_H

#define FRMT_CHAN_ID_LEN  4
#define FRMT_HEXCODS_LEN 22
#define FRMT_DECODES_LEN 49
#define FRMT_TOTAL_LEN (FRMT_CHAN_ID_LEN + 1 + FRMT_HEXCODS_LEN + 1 + FRMT_DECODES_LEN)
#define OUTBUFF_TOTAL_LEN (FRMT_TOTAL_LEN + 2 + 1)

typedef uint8_t outBuff_t[OUTBUFF_TOTAL_LEN];
extern const uint8_t dash_padding[41];

#define FRMT_PRINT_PATHETIC_EXCUSE( id, len ) \
    if (len != sizeof(outBuff_t))\
        printf("There might have already been a TRAP so its possibly too late for this message!\n"\
               "ID %04X: Expected %d but got %d - printf formats are tricky stuff\n",id,sizeof(outBuff_t),len)


typedef struct outArgsStruct {
    int      oArgMsgLen;
    uint8_t  oArgChanID;
    uint32_t oArgMsgVrfdDltaTim;
    uint32_t oArgBitSmplTimDlta;
    uint     oArgBitSmplTotCnt;
    uint     oArgBitSmplOnesCnt;
    uint32_t oArgSmplTimPrv;
    uint32_t oArgSmplTimDlta;
    uint     oArgSmplVrfyCnt;
    uint     oArgSmplUnVrfyCnt;
    uint     oArgSmplHdrHitsCnt;
    uint     oArgSmplChkErrsCnt;
    uint     oArgMsgHiWtr;
    uint     oArgWrdHiWtr;
    uint     oArgFiFoHiWtr;
} outArgs_t;

extern const uint ON_BOARD_LED;
extern void print_msg( uint8_t outBuff[OUTBUFF_TOTAL_LEN], outArgs_t * outArgsP );

/*
<FD0A FFAFD22840090B4C4A08BC
 4600 461284190DCF
 012345678901234567890123456
           1         2  
 i:?,b:0,t: 15.2,h: 64,r: 943.8,a: 3.1,g: 3.7,c: 8>
 i:0,b:1,t: 15.2
78901234567890123456789012345678901234567890123456
   3         4         5         6         7
 5 (   6)  95.5s rate= 124/s ( 5345   735),chkErs= 122,hdrHits= 133,msgHi= 1,wrdHi= 6,fifoHi= 2 

i:%5.5s,%04d-%02d-%02d %02d:%02d:%02d
12    78   23   56   89   12   45   7
           1              2              
i:1,b:%1d,t:%5.1f,h:%3d,r:%6.1f,a:%4.1f,g:%4.1f,c:%2d
123456  7890    5678  1234    0123    7890    4567  9
           1          2       3          4                   
*/

#define OUTPUT_FORMAT_H
#endif
