## PICO_PIO
RF input decoding using PICO PIO
### Why do it?
I had a fridge that kept freezing vegetables. It was interesting to see how bad the overshoot and undershoot of the controller was.
<br>
And why not monitor the freezer too. It had a much tighter controller.
<br>
The attic was getting extra loft insulation and I wondered how cold the water tank now got.
<br>
There is a water softener in the garage that didn't want to get too cold.
<br>
So I got 5 Ambient Weather F007T sensors with base station.
The base station was rubbish, invariably displaying 'HH', impossible temperatures and push button switches from the 1960's.
<br>
Then my youngest son inherited a Fine Offset WH1080 weather station from his grandfather ...
### **Overview**
![overview](https://github.com/peter-g-wilson/PICO_PIO/blob/main/images/overview.png)
### **Code**
* _**PICO_PIO_F007T_WH1080.c**_  has the CPU core 0 main entry point that calls the WH1080 and F007T timer and PIO initialisations and also has the core 1 entry point to handle the 2nd UART and the output of messages that have been received
* _**PICO_PIO_F007T_WH1080.pio**_ has the two state machine programs - both feeding their FIFOs with data bits
* _**PICO_PIO_WH1080.c**_ and _**PICO_PIO_F007T.c**_ use repeating timer callbacks to read the PIO FIFOs and "parse" the data bits looking for their respective messages
* _**queues_for_msgs_and_bits.c**_ has support routines for message and bit queues
* _**uart_IO.c**_ has support routines for the 2nd UART where the message data is sent over RS232
* _**output_format.c**_ prints to std output (1st UART) debug and statistics
### **More details and performance results are in -**
![PICO_PIO_OOK_Manchester_and_PWM.pdf](https://github.com/peter-g-wilson/PICO_PIO/blob/main/pdf/PICO_PIO_OOK_Manchester_and_PWM.pdf)
<br>
