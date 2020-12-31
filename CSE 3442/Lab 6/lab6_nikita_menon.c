/*
 * lab6_nikita_menon.c
 *
 *  Created on: Oct 22, 2019
 *      Author: Nikita Menon
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define INPUT     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // PE4
#define OUTPUT      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4))) //PE5

#define input_mask 16
#define output_mask 32
#define GREEN_LED_MASK 8

#define MAX_CHARS 81

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

    uint32_t load_value;
    uint32_t load_val [] = {54000,
                            72000,
                             28000,
                             16000,
                             18000,
                             18000,
                             14000,
                             26000,
                             28000,
                             26000,
                             44000,
                             46000,
                             28000,
                             18000,
                             16000,
                             18000,
                             18000,
                             18000,
                             20000,
                             16000,
                             16000,
                             18000,
                             18000,
                             18000,
                             18000,
                             20000,
                             20000,
                             16000,
                             16000,
                             18000,
                             20000,
                             18000,
                             26000,
                             26000,
                             18000,
                             18000,
                             18000,
                             20000,
                             16000,
                             24000,
                             36000,
                             30000,
                             16000,
                             24000,
                             28000,
                             18000,
                             28000,
                             36000,
                             26000,
                             18000,
                             16000,
                             18000,
                             20000,
                             20000,
                             17600,
                             17600,
                             17600,
                             17600,
                             17600,
                             17600,
                             17600,
                             17600,
                             17600,
                                17600,
                                17600,
    17600, 17600, 17600, 17600, 17600, 17600};

int count_edge =0;
int count_time =0;

void initHw()
{
      SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

       // Enable clocks
      SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

      SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA| SYSCTL_RCGC2_GPIOE ;

       // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
       // Note UART on port A must use APB
       SYSCTL_GPIOHBCTL_R = 0;

       //GP0 PE5
       GPIO_PORTE_DIR_R |=  output_mask;
       GPIO_PORTE_DEN_R |=  output_mask;

      // OUTPUT =0;
       //GPI PE4
     //  GPIO_PORTE_DIR_R &=  ~input_mask;
   //    GPIO_PORTE_DIR_R |=  output_mask;
       GPIO_PORTE_DEN_R |=   input_mask;


       //GPI PE4
        //Setting interrupts
       GPIO_PORTE_IS_R &= ~(input_mask); //int sense
       GPIO_PORTE_IBE_R &= ~(input_mask);// int both edges
       GPIO_PORTE_IEV_R &= ~(input_mask);//int event
       GPIO_PORTE_ICR_R |= input_mask;// int clear
       NVIC_EN0_R |= 1 << (INT_GPIOE-16); //turn-on interrupt 20 (GPIOE)
       GPIO_PORTE_IM_R |= input_mask;// int mask

//       GPIO_PORTF_DATA_R |= GREEN_LED_MASK;
      TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
       TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
       TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;
       TIMER1_TAILR_R = 54000;
       //TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
       NVIC_EN0_R |= 1 <<(INT_TIMER1A-16);


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


void IDENTIFY_BUTTON(uint16_t button_pressed)
{
    if(button_pressed == 42662)
    {
        putsUart0("Center button");
        putcUart0(10);
        putcUart0(13);

    }


    else if(button_pressed == 43349)
        {
            putsUart0("Down button");
            putcUart0(10);
            putcUart0(13);
        }

    else if(button_pressed == 42665)
        {
            putsUart0("Right button");
            putcUart0(10);
            putcUart0(13);
        }

    else if(button_pressed == 42666)
        {
            putsUart0("Left button");
            putcUart0(10);
            putcUart0(13);
        }

    else if(button_pressed == 43350)
            {
                putsUart0("Up button");
                putcUart0(10);
                putcUart0(13);
            }

    else if(button_pressed == 42661)
            {
                putsUart0("Back button");
                putcUart0(10);
                putcUart0(13);
            }

    else if(button_pressed == 38486)
              {
                  putsUart0("Play button");
                  putcUart0(10);
                  putcUart0(13);
              }

    else
    {
        putsUart0("Error");
    }


}




void edgeISR()
{
    GPIO_PORTE_IM_R &= ~input_mask; //Turn off interrupt
    GPIO_PORTE_ICR_R |= input_mask;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;//Turning on timer
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;// Turning on timer interrupt

    TIMER1_TAILR_R = load_val[0];
    //count_edge++;




    OUTPUT = 1;





}

void timerISR()
{
    //char strInput[MAX_CHARS];
    OUTPUT ^=1 ;
   // int x = 16;
   static uint32_t bin=0;
   count_time = count_time +1;
   static int count_data = 0;

    //putcUart0(INPUT);
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAILR_R  = load_val[count_time];
    TIMER1_CTL_R |= TIMER_CTL_TAEN;


    TIMER1_ICR_R =1;



    if(count_time > 54 && count_time < 71)
    {
       // putsUart0("BITS = ");
        bin  = bin<<1;
        bin += (INPUT);

        putcUart0(INPUT+48);
        count_data = count_data +1;
        if(count_data == 16)
        {
            IDENTIFY_BUTTON(bin);
        }


       // putsUart0("bin = ");
       // putsUart0(bin+48);
        //bin = bin<<1;

    }

    if(count_time >= 70)
    {
            putcUart0(10);
            putcUart0(13);
            count_time =0;
           count_data = 0;
            OUTPUT =0;
            bin =0;
            TIMER1_TAILR_R  = load_val[count_time];

            TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
          //  TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            GPIO_PORTE_ICR_R |= input_mask;
            GPIO_PORTE_IM_R |= input_mask;




    }


}


int main()

{
    initHw();

    while(1){

    }

    return 0;
}
