/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <stdio.h>
#include "system/common/sys_module.h"   // SYS function prototypes
#include "./fatfs/ff.h"
#include "main.h"



// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
void uart2_putc(unsigned char inputChar);
void uart2_puts(unsigned char* string);
unsigned char uart2_receiveByte(void);

void delay_ms(unsigned int count);

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );
    __XC_UART = 2;  // Code is configured to use UART2 
    TRISA = 0x0000;
    LATACLR = 0x0001;
    char buffer[256] = { 0 };
    const char text1[] = "Hello World sk1632!\r This file is created with FatFs driver!\r Have a great day!\r\r";
    unsigned int numWrite = 0;   // number of bytes written to file.
    unsigned int numRead = 0;    // number of bytes read from file.
    
    // Pull-ups must be enabled for the SDcard pins!
    CNPUAbits.CNPUA1 = 1; // RA1 pull-up - SDO2.
    CNPUBbits.CNPUB0 = 1; // RB0 pull-up - CS.
    CNPUBbits.CNPUB2 = 1; // RB2 pull-up - SDI2.
    CNPUBbits.CNPUB15 = 1; // RB15 pull-up - SCK2.    
    
    FATFS Fatfs;
    FATFS *fs; /* Pointer to file system object */
    FIL file1;    
    
    printf("sk1632 FatFs test!\r");
    
    // Initialize disk. If SD-card not present or faulty, wait there
    // and blink LED.
    if(disk_initialize(0)) {
        printf("No card or card error!\r");
        while(1) { 
            LATAbits.LATA0 ^= 1;
            delay_ms(250);
        }
    }
    
    // Mounting the file system. If file system invalid, wait there and blink LED.
    if(f_mount(&Fatfs,"",0) == FR_INVALID_DRIVE) {
        printf("Invalid file system!\r");
        while(1) { 
            LATAbits.LATA0 ^= 1;
            delay_ms(250);
        }        
    }
        
    // Create a file here named "hello123.txt":
    // (File is created if it does not exist, and if it is, file is opened instead).
    printf("Creating a file inside for reading and writing!\r");
    f_open(&file1, "hello123.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ); 

    // Write a text (contents inside text1[]) into the file.
    f_write(&file1, &text1, sizeof(text1), &numWrite);
    
    // Rewind the file pointer to the start!
    f_lseek(&file1, 0);
    
    // Read the text and store them into the buffer.
    printf("Reading the file inside...\r");
    f_read(&file1, &buffer, sizeof(text1), &numRead);
    
    // Print the buffer contents to the terminal:
    printf("Printing contents: %s\r", &buffer);
        
    // Close the file after it is done!
    printf("Closing file!\r");
    f_close(&file1);
    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        //SYS_Tasks ( );
        LATAbits.LATA0 ^= 1;
        delay_ms(500);
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

void uart2_putc(unsigned char inputChar) 
{
    while(U2STAbits.UTXBF);
    U2TXREG = inputChar;  
    
}

void uart2_puts(unsigned char* string)
{
    while(*string)
      uart2_putc(*string++);
}

void _mon_putc(char c)
 {
   while (U2STAbits.UTXBF); //Wait till transmission is complete
   U2TXREG = c;
 }

unsigned char uart2_receiveByte(void) {
    unsigned char b;
    while(!U2STAbits.URXDA);
    b = U2RXREG;
    U2STAbits.OERR = 0;
    return b;    
}

void delay_ms(unsigned int count)
{
	T1CON = 0x8030;		// turn on timer, prescaler to 256 (type B timer)
	while(count--)
	{
		TMR1 = 0;
		while(TMR1 < 0x4e);
	}
	T1CONbits.ON = 0;
}

/*******************************************************************************
 End of File
*/

