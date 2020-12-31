/*
 * lab04a_nikita_menon.c
 *
 *  Created on: Oct 1, 2019
 *      Author: Nikita Menon
 */


// Timing C/ASM Mix Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    25 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define RED_LED_MASK 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 7, creating system clock of 25 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (7 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    // Configure LED pin
    GPIO_PORTF_DIR_R = RED_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = RED_LED_MASK;  // enable LED
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #3");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 4
    __asm("             CBZ  R1, WMS_DONE1");   //
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             B    WMS_LOOP1");
    __asm("             NOP");
    __asm("             NOP");
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             NOP");
    __asm("             B    WMS_LOOP0");
    __asm("WMS_DONE0:");                        // ---
                                                // 25 clocks/us + error
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();

    // Toggle red LED every second
    while(1)
    {

      RED_LED ^= 1;
      waitMicrosecond(1000000);
    }

    return 0;
}
