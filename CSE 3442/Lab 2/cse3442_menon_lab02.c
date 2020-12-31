/*
   CSE 3442 
   1001548454
   nam8454
 * lab02.c
 *
 *  Created on: Sep 10, 2019
 *      Author: Nikita Menon
 */


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PE4 powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // PE4
#define YELLOW_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4))) //PE5
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))    //PE1
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // PE3
#define PUSH_BUTTON_ONE  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) //PE0
#define PUSH_BUTTON_TWO (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))//PE2
#define GREEN_LED_MASK 8
#define RED_LED_MASK 16
#define YELLOW_LED_MASK 32
#define BLUE_LED_MASK 2
#define PUSH_BUTTON_ONE_MASK 1
#define PUSH_BUTTON_TWO_MASK 4

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


// Blocking function that returns only when SW1 is pressed
void waitPbOnePress()
{
	while(PUSH_BUTTON_ONE);
}

void waitPbTwoPress()
{
  while(!PUSH_BUTTON_TWO);
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOE ;

    // Configure LED
    GPIO_PORTE_DIR_R =  RED_LED_MASK | YELLOW_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R =  RED_LED_MASK | YELLOW_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
   GPIO_PORTE_DEN_R =  RED_LED_MASK | YELLOW_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK |PUSH_BUTTON_ONE_MASK|PUSH_BUTTON_TWO_MASK;  // enable LEDs and pushbuttons (only green and yellow)
  	GPIO_PORTE_PUR_R = PUSH_BUTTON_ONE_MASK; // enable internal pull-up for push button 1
   GPIO_PORTE_PDR_R = PUSH_BUTTON_TWO_MASK; // enable internal pull-down for push button 2
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
 {
    // Initialize hardware
    initHw();

 //TURNING OFF ALL LEDS
 RED_LED = 0; //off
 YELLOW_LED = 1; // Off 
 BLUE_LED = 0; // Off
 GREEN_LED = 1; // Off


 //Enable RED LED
  RED_LED = 1;
  	
 //Wait for PB2 press
    waitPbTwoPress();

    // Turn off red LED, turn on green LED
    RED_LED = 0;
    GREEN_LED = 0;

	//Wait for 1 second
	waitMicrosecond(1000000);
	
	//Enable BLUE LED 
	BLUE_LED = 1;
	
	
	
	//Wait for PB 1 press
	waitPbOnePress();
	
	//Wait for 500ms
	waitMicrosecond(500000);
	
	while(1)
	{
	//Toggle YELLOW LED
	YELLOW_LED = 0; 
	waitMicrosecond(500000);
	YELLOW_LED = 1;
	
	waitMicrosecond(500000);
	}


    return 0;
}
