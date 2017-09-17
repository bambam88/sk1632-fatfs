#ifndef _MAIN_H  
#define _MAIN_H

#include <p32xxxx.h>

#define _CS_SETOUT()     TRISBbits.TRISB0 = 0 
#define _CS_LOW()        LATBbits.LATB0 = 0  
#define _CS_HIGH()       LATBbits.LATB0 = 1 

#define _SPIBRG          SPI2BRG       
#define _SPIBUF          SPI2BUF
#define _SPISTATbits     SPI2STATbits   
#define _SPI_CHANNEL     SPI_CHANNEL_2
#define _SPICONbits      SPI2CONbits

#define _SPIxFPB         20000000                   // Peripheral Bus frequency for SPI.      
#define _SPIxMAX_FREQ    SPIxFPB/(2*1000000) - 1   // Maximum SPI clock speed.    
#define _SPIxMIN_FREQ    SPIxFPB/(2*400000) - 1      // Minimum SPI clock speed.

#define _FCLK_SLOW()     SPIBRG = SPIxFPB/(2*400000) - 1
#define _FCLK_FAST()     SPIBRG = SPIxFPB/(2*1000000) - 1

#define _SOCKPORT        PORTB





#endif

