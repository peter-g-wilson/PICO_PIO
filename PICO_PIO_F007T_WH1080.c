#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "PICO_PIO_WH1080.h"
#include "PICO_PIO_F007T.h"
#include "output_format.h"
#include "uart_IO.h"

uint8_t rxIpBuff[256];
uint charsRxInput = 0;
void rxIpProcess( int ipLen ) {
    charsRxInput = ipLen;
}

void core1_entry() {
    puts("Hello, other world!");
    uartIO_init((uartIO_rxCallBack_t *)rxIpProcess,rxIpBuff,sizeof(rxIpBuff));
    uartIO_rxEnable( true );

    while (true) {
        while (WH1080_tryMsgBuf()) {
            gpio_put(ON_BOARD_LED,1);
            WH1080_doMsgBuf();
            gpio_put(ON_BOARD_LED,0);
        };
        while (F007T_tryMsgBuf()) {
            gpio_put(ON_BOARD_LED,1);
            F007T_doMsgBuf();
            gpio_put(ON_BOARD_LED,0);
        };
        if (charsRxInput > 0) {
            printf("%*.*s",charsRxInput,charsRxInput,&rxIpBuff[0]);
            charsRxInput = 0;
            uartIO_rxEnable(true);
        }
        sleep_ms(1000);
    }
}
/*-----------------------------------------------------------------*/
int main()
{
    stdio_init_all();

    gpio_init(ON_BOARD_LED);
    gpio_set_dir(ON_BOARD_LED,GPIO_OUT);
    gpio_put(ON_BOARD_LED,0);

    puts("Hello, world!");

    WH1080_init( 200, 100 );
    F007T_init(  100,  50 );

    multicore_launch_core1(core1_entry);

    while (1)
        tight_loop_contents();

    F007T_uninit();
    WH1080_uninit();
}
