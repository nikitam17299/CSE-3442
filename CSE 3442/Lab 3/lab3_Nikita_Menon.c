/*
 * lab03_nikita_menon.c
 *
 *  Created on: Sep 17, 2019
 *      Author: Nikita Menon
 */


// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define MAX_CHARS 81
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF;

    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                     // enable TX, RX, and module
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getsUart0(char* str, uint8_t maxChars)
{

    int count =0;


    char c;

   while(count<MAX_CHARS)
{

  c = getcUart0();


  if(count == MAX_CHARS-2)
                  {
                    str[count] = c;
                    str[count+1]=0;
                    return ;
                  }

  if((c == 8 || c == 127) && (count>0))
      {
          count--;
      }


  if(c == 13)
 {

     str[count]= 0;
     return ;
 }

  if((c >= 65) && (c <=90))
         {
            c = c + 32;
            str[count] = c;
            count++;
         }

  else if(c>=32)
     {
         str[count] = c;
         count++;
     }


}

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

  char strInput[MAX_CHARS];
  //char one;
  initHw();

  while(1){
      getsUart0(strInput,MAX_CHARS);
      putsUart0("\r\n");
      putsUart0(strInput);
      putsUart0("\r\n");
      putsUart0("\r\n");
  }


return 0;

}
