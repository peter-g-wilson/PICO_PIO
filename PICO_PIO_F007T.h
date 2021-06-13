
#ifndef PICO_PIO_F007T_H

extern void F007T_init( uint32_t parseRptTime, uint32_t fifoRptTime );
extern void F007T_uninit( void );
extern bool F007T_tryMsgBuf( void );
extern void F007T_doMsgBuf( void );

#define PICO_PIO_F007T_H
#endif
