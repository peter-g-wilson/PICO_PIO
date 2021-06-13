
#ifndef PICO_PIO_WH1080_H

extern void WH1080_init( uint32_t parseRptTime, uint32_t fifoRptTime );
extern void WH1080_uninit( void );
extern bool WH1080_tryMsgBuf( void );
extern void WH1080_doMsgBuf( void );

#define PICO_PIO_WH1080_H
#endif
