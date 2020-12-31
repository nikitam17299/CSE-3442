#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"

#define RED     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*7))) // PC7
#define BLUE     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*6))) // PC6
#define YELLOW     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*5))) // PC5
#define WHITE     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // PC4
#define SPEAKER     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 4*0))) // PDO
#define SENSOR     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 4*1))) // PD1
#define INPUT     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // PE4
#define OUTPUT      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4))) //PE5
#define PB1         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))//PF1
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))//PF3

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define RED_MASK 128
#define BLUE_MASK 64
#define YELLOW_MASK 32
#define WHITE_MASK 16
#define SPEAKER_MASK 1
#define SENSOR_MASK 2
#define INPUT_MASK 16
#define OUTPUT_MASK 32
#define PB1_MASK 16


#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define delay6Cycles() __asm(" NOP\n NOP\n NOP\n NOP\n NOP\n NOP")
#define MAX_CHARS 81

 uint32_t load_value;
 uint32_t load_val [] = {54000,72000,28000,16000,18000,18000,14000,26000,28000,26000,
                         44000,46000,28000,18000,16000,18000,18000,18000,20000,16000,
                         16000,18000,18000,18000,18000,20000,20000,16000,16000,18000,
                         20000,18000,26000,26000,18000,18000,18000,20000,16000,24000,
                         36000,30000,16000,24000,28000,18000,28000,36000,26000,18000,
                         16000,18000,20000,20000,17600,17600,17600,17600,17600,17600,
                         17600,17600,17600,17600,17600,17600, 17600, 17600, 17600, 17600,
                         17600};

int count_edge =0;
int count_time =0;
int count_pass =0;
int password;
int is_locked = 2;
int is_beeper_on =2;
int is_local_on = 2;
int is_up = 2;
int is_down = 2;
int is_right =2;
int is_left =2;
int is_center = 2;
int is_play =2;
int is_back =2;
uint16_t counter =0;
int is_password  =2;
static int password_store[8];
int password_count = 0;
int key_up = 1;
int key_down = 2;
int key_right = 3;
int key_left = 4;
int key_center = 5;
int key_play = 6;
int key_back = 7;
int auto_enabled = 2;
int is_led =2;
int is_jammed=2;

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
        SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
        SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
         SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;

    // Enable GPIO port C, D and A peripherals
         SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOA| SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

   //EEPROM
//       SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;
//       delay6Cycles();
//       while((EEPROM_EEDONE_R) && (EEPROM_EEDONE_WORKING));
//
//       if((EEPROM_EESUPP_R) && ((EEPROM_EESUPP_PRETRY ) || (EEPROM_EESUPP_R) && (EEPROM_EESUPP_ERETRY)))
//      {
//          return false;
//
//      }
//
//       SYSCTL_SREEPROM_R = SYSCTL_SREEPROM_R0;
//       SYSCTL_SREEPROM_R = !SYSCTL_SREEPROM_R0;
//
//       delay6Cycles();
//
//       while((EEPROM_EEDONE_R) & (EEPROM_EEDONE_WORKING));
//
//       if((EEPROM_EESUPP_R) && ((EEPROM_EESUPP_PRETRY ) || (EEPROM_EESUPP_R) && (EEPROM_EESUPP_ERETRY)))
//      {
//          return false;
//
//      }

    // Configure MOTOR
         GPIO_PORTC_DIR_R |= RED_MASK | BLUE_MASK| WHITE_MASK | YELLOW_MASK;  // make bit an output
         GPIO_PORTC_DR2R_R |= RED_MASK | BLUE_MASK| WHITE_MASK | YELLOW_MASK;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
         GPIO_PORTC_DEN_R |= RED_MASK | BLUE_MASK| WHITE_MASK | YELLOW_MASK ;  // enable LED

    // Configure SPEAKER AND SENSOR
        GPIO_PORTD_DIR_R |= SPEAKER_MASK;  // make bit an output
        GPIO_PORTD_DIR_R &= ~(SENSOR_MASK);
        GPIO_PORTD_DR2R_R |= SPEAKER_MASK | SENSOR_MASK;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTD_DEN_R |= SPEAKER_MASK | SENSOR_MASK ;  // enable LED
        GPIO_PORTD_PUR_R |= SENSOR_MASK;

      //Confiuring UART pins
        GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
        GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
        GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
        GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
        GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;// select UART0 to drive pins PA0 and PA1: default, added for clarity


     // Configure UART0 to 115200 baud, 8N1 format
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
        delay4Cycles();                                  // wait 4 clock cycles
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


      //GP0 PE5
        GPIO_PORTE_DIR_R |= OUTPUT_MASK;
        GPIO_PORTE_DEN_R |= OUTPUT_MASK;


      //GPI PE4
        GPIO_PORTE_DEN_R |= INPUT_MASK;

      //Setting interrupts
        GPIO_PORTE_IS_R &= ~(INPUT_MASK); //int sense
        GPIO_PORTE_IBE_R &= ~(INPUT_MASK);// int both edges
        GPIO_PORTE_IEV_R &= ~(INPUT_MASK);//int event
        GPIO_PORTE_ICR_R |= INPUT_MASK;// int clear
        NVIC_EN0_R |= 1 << (INT_GPIOE-16); //turn-on interrupt 20 (GPIOE)
        GPIO_PORTE_IM_R |= INPUT_MASK;// int mask


        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;
        TIMER1_TAILR_R = 54000;
        NVIC_EN0_R |= 1 <<(INT_TIMER1A-16);

     //PB1 PF4
     //Setting interrupts
        GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
        GPIO_PORTF_CR_R  |= PB1_MASK;
        GPIO_PORTF_PUR_R |= PB1_MASK;
        GPIO_PORTF_IS_R &= ~(PB1_MASK); //int sense
        GPIO_PORTF_IBE_R &= ~(PB1_MASK);// int both edges
        GPIO_PORTF_IEV_R &= ~(PB1_MASK);//int event
        GPIO_PORTF_ICR_R |= PB1_MASK;// int clear
        NVIC_EN0_R |= 1 << (INT_GPIOF-16); //turn-on interrupt 20 (GPIOF)
        GPIO_PORTF_IM_R |= PB1_MASK;// int mask

    //GREEN LED AND RED LED Configure LED pins
        GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;
        GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK | PB1_MASK;  // enable LEDs

        //Hibernate Module & Hibernate Interrupt
        NVIC_EN1_R |= 1 << (INT_HIBERNATE-16-32);

        //HIB_IM_R |= HIB_IM_RTCALT0;
        while(!(HIB_CTL_R & HIB_CTL_WRC));
        HIB_CTL_R |= HIB_CTL_CLK32EN;
}

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

//int readEEPROM()
//{
//    //static int counter =0;
//
//    EEPROM_EEBLOCK_R = 1;
//    EEPROM_EEOFFSET_R = count_pass;
//    count_pass++;
//    return EEPROM_EERDWR_R;
//
//
//
//}

//void writeEEPROM(uint16_t counter ,int button_pressed)
//{
//
//       // static int counter =0;
//
//        EEPROM_EEBLOCK_R = 0;
//        EEPROM_EEOFFSET_R =counter;
//        while((EEPROM_EEDONE_R) && (EEPROM_EEDONE_WORKING));
//        EEPROM_EERDWR_R = button_pressed;
//
//}

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

    if(is_beeper_on == 1)
        {
            speaker_on();
        }

    if(button_pressed == 43349)
        {   if(is_down == 0)
            {
            putsUart0("Down button");
            putcUart0(10);
            putcUart0(13);
            password_store[password_count] = button_pressed;
            password_count++;



            }
        is_center = 0;
        is_up = 0;
        is_down = 1;
        is_left = 0;
        is_right = 0;
        is_center =0;
        is_back = 0;
        is_play = 0;
        counter++;

        if(is_beeper_on == 1)
               {
                   speaker_on();
               }
       }

    else if(button_pressed == 42662)
        {
            if(is_center == 0)
            {
            putsUart0("Center button");
            putcUart0(10);
            putcUart0(13);
            is_center =1;
            }
            is_center = 1;
            is_up = 0;
            is_down = 0;
            is_left = 0;
            is_right = 0;
            is_center =0;
            is_back = 0;
            is_play = 0;
            counter++;

            if(is_beeper_on == 1)
                           {
                               speaker_on();
                           }
        }

    else if(button_pressed == 42665)
        {
            if(is_right ==0)
            {
                putsUart0("Right button");
                putcUart0(10);
                putcUart0(13);
            }

            is_center = 0;
            is_up = 0;
            is_down = 0;
            is_left = 0;
            is_right = 1;
            is_center =0;
            is_back = 0;
            is_play = 0;
            counter++;

            if(is_beeper_on == 1)
                           {
                               speaker_on();
                           }
        }

    else if(button_pressed == 42666)
        {   if(is_left ==0)
            {
            putsUart0("Left button");
            putcUart0(10);
            putcUart0(13);

            }
        if(is_beeper_on == 1)
                       {
                           speaker_on();
                       }
            is_center = 0;
            is_up = 0;
            is_down = 0;
            is_left = 1;
            is_right = 0;
            is_center =0;
            is_back = 0;
            is_play = 0;
        }
    else if(button_pressed == 43350)
            {
                if(is_up ==0)
                {
                putsUart0("Up button");
                putcUart0(10);
                putcUart0(13);

                }
                if(is_beeper_on == 1)
                               {
                                   speaker_on();
                               }
                is_center = 0;
                is_up = 1;
                is_down = 0;
                is_left = 0;
                is_right = 0;
                is_center =0;
                is_back = 0;
                is_play = 0;
            }
    else if(button_pressed == 42661)
            {
                if(is_back ==0)
                {
                putsUart0("Back button");
                putcUart0(10);
                putcUart0(13);

                }
                if(is_beeper_on == 1)
                               {
                                   speaker_on();
                               }
                is_center = 0;
                is_up = 0;
                is_down = 0;
                is_left = 0;
                is_right = 0;
                is_center =0;
                is_back = 1;
                is_play = 0;
            }
    else if(button_pressed == 38486)
              {
                 if(is_play == 0)
                 {
                  putsUart0("Play button");
                  putcUart0(10);
                  putcUart0(13);
                 }
                 if(is_beeper_on == 1)
                                {
                                    speaker_on();
                                }
                 is_center = 0;
                 is_up = 0;
                 is_down = 0;
                 is_left = 0;
                 is_right = 0;
                 is_center =0;
                 is_back = 0;
                 is_play = 1;
              }

    else
    {
        putsUart0("Error");
        return;
    }

//    if(is_password == 1)
//    {
//        writeEEPROM(counter ,button_pressed);
//    }
//    counter++;
}
void motor_lock()
{
    int i =0;

    //MOTOR LOCK
    for(i=0;i<30;i++)
                   {

                       YELLOW =1 ;
                       waitMicrosecond(10000);
                       YELLOW =0;
                       waitMicrosecond(10000);


                       RED =1;
                       waitMicrosecond(10000);
                       RED =0;
                       waitMicrosecond(10000);

                       BLUE = 1;
                       waitMicrosecond(10000);
                       BLUE =0;
                       waitMicrosecond(10000);


                       WHITE =1 ;
                       waitMicrosecond(10000);
                       WHITE =0;
                       waitMicrosecond(10000);





                   }

    if(is_beeper_on == 1)
    {
        speaker_on();
    }

    is_locked = 1;
    if(is_led == 1)
            {
            GREEN_LED = 0;
            RED_LED = 1;
            }






}



void motor_unlock()
{

    int i =0;

    //MOTOR unlock
             for(i=0;i<30;i++)
            {

                WHITE =1 ;
                waitMicrosecond(10000);
                WHITE =0;
                waitMicrosecond(10000);

                BLUE = 1;
                waitMicrosecond(10000);
                BLUE =0;
                waitMicrosecond(10000);


                RED =1;
                waitMicrosecond(10000);
                RED =0;
                waitMicrosecond(10000);

                YELLOW =1 ;
                waitMicrosecond(10000);
                YELLOW =0;
                waitMicrosecond(10000);

            }

         if(is_beeper_on == 1)
           {
              speaker_on();
           }

         is_locked = 0;
         if(is_led == 1)
         {
         RED_LED = 0;
         GREEN_LED = 1;
         }
}



void edgePB()
{

   if(SENSOR == 1)
   {
       motor_unlock();
   }

   else if(SENSOR == 0)
   {
          motor_lock();
   }

   GPIO_PORTF_ICR_R |= PB1_MASK;
}



void edgeISR()
{

    GPIO_PORTE_IM_R &= ~(INPUT_MASK); //Turn off interrupt
    GPIO_PORTE_ICR_R |= INPUT_MASK;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;//Turning on timer
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;// Turning on timer interrupt

    TIMER1_TAILR_R = load_val[0];

    OUTPUT = 1;

}

void timerISR()
{

    OUTPUT ^=1 ;

   static uint32_t bin=0;
   count_time = count_time +1;
   static int count_data = 0;


    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAILR_R  = load_val[count_time];
    TIMER1_CTL_R |= TIMER_CTL_TAEN;


    TIMER1_ICR_R =1;


    //last 16
    if(count_time > 54 && count_time < 71)
    {

        bin  = bin<<1;
        bin += (INPUT);

        putcUart0(INPUT+48);
        count_data = count_data +1;
        if(count_data == 16)
        {
            IDENTIFY_BUTTON(bin);
        }


   }

   //resetting
    if(count_time >= 70)
    {
          putcUart0(10);
          putcUart0(13);
          count_time =0;
          count_data =0;
          OUTPUT =0;
          bin =0;
          TIMER1_TAILR_R  = load_val[count_time];
          TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
          GPIO_PORTE_ICR_R |= INPUT_MASK;
          GPIO_PORTE_IM_R |= INPUT_MASK;


    }


}


void sensor()
{
    if(SENSOR == 1)
    {
        putsUart0("Locked\r\n");
          if(is_beeper_on == 1)
          {
              speaker_on();
          }
    }

    if(SENSOR == 0)
    {
        putsUart0("Unlocked\r\n");
        if(is_beeper_on == 1)
        {
            speaker_on();
        }
    }
}


void speaker_on()
{
    int i;
    for(i = 0; i <= 50; i++)
    {
    SPEAKER = 1;

    waitMicrosecond(1000);

    SPEAKER = 0;

    waitMicrosecond(1000);
    }
}

void status()
{
    if(SENSOR == 1 && is_locked == 1 && is_jammed == 0)
    {
        putsUart0("Locked\r\n");
        if(is_beeper_on == 1)
        {
            speaker_on();
        }
    }

    if(SENSOR == 0 && is_locked == 0 && is_jammed == 0)
    {
        putsUart0("Unlocked\r\n");
        if(is_beeper_on == 1)
        {
            speaker_on();
        }
    }


    if(SENSOR == 1 && is_locked == 0 )
    {
        putsUart0("Jammed\r\n");
    }

    if(SENSOR == 0 && is_locked == 1 )
        {
            putsUart0("Jammed\r\n");
        }


}

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

void setRTC()
{
    char hour[3];
    char minute[3];
    int hours = 0;
    int minutes = 0;

    putsUart0("What hour would you like it locked? (24-hour clock)\r\n");
    getsUart0(hour, 3);
    putsUart0("What minute would you like it locked?\r\n");
    getsUart0(minute, 3);

    hours = atoi(hour);
    minutes = atoi(minute);

    hours = hours * 60 * 60;
    minutes = minutes * 60;

    while(!(HIB_CTL_R & HIB_CTL_WRC));

        HIB_RTCM0_R = hours + minutes;

    while(!(HIB_CTL_R & HIB_CTL_WRC));

        HIB_IM_R |= HIB_IM_RTCALT0;
}

void setTime()
{
    char hour[3];
    char minute[3];
    int hours = 0;
    int minutes = 0;

    putsUart0("What hour is it? (24-hour clock)\r\n");
    getsUart0(hour, 3);
    putsUart0("What minute is it?\r\n");
    getsUart0(minute, 3);

    hours = atoi(hour);
    minutes = atoi(minute);

    hours = hours * 60 * 60;
    minutes = minutes * 60;

    while(!(HIB_CTL_R & HIB_CTL_WRC));

    HIB_CTL_R |= HIB_CTL_RTCEN;

    while(!(HIB_CTL_R & HIB_CTL_WRC));

    HIB_RTCLD_R = 0;

    while(!(HIB_CTL_R & HIB_CTL_WRC));

    HIB_RTCLD_R = hours + minutes;
}

void hibernateRTC()
{
    if(auto_enabled == 1)
    {
        if(HIB_MIS_R & 1)
        {
            motor_lock();
        }
    }
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_IM_R = 1;
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_IC_R = 0b10001;
}

void main()
  {
  int i=0;
  auto_enabled = 2;
  initHw();
  putsUart0("Hello");
  putsUart0("\r\n");
  RED_LED = 1;
  waitMicrosecond(500000);
  RED_LED =0;
  is_local_on = 0;

  char strInput[MAX_CHARS];

     if(SENSOR == 1)
     {
         is_locked = 1;
         is_jammed =0;
         putsUart0("Set is_locked\r\n");

     }

     if(SENSOR == 0)
     {
         is_locked = 0;
         is_jammed =0;
         putsUart0("Set is_locked\r\n");
     }

    is_beeper_on =0;

    while (1)
   {

        getsUart0(strInput,MAX_CHARS);
        putsUart0("\r\n");
        putsUart0(strInput);
        putsUart0("\r\n");
        putsUart0("\r\n");

      if(strcmp(strInput,"lock")==0)
      {
          if(SENSOR == 1 && is_locked == 0){
             is_jammed =1;
          }

          else if(SENSOR == 0 && is_locked ==1)
          {
              is_jammed =1;
          }
              if(is_locked == 1 && is_jammed ==0 && SENSOR == 1)
              {
                  putsUart0("Already Locked\r\n");
              }

              else if (is_locked == 0 && is_jammed ==0 && SENSOR ==0)
              {
                  motor_lock();
                  is_locked = 1;

              }

               if(SENSOR == 0 && is_jammed ==1){
                 motor_lock();
                 is_jammed =0;
                  //putsUart0("Jammed\r\n");
              }

              //else if(SENSOR == 1 && is_jammed ==){
                //               motor_lock();
                  //             is_jammed =0;
                                //putsUart0("Jammed\r\n");
                           // }


      }
      else if(strcmp(strInput,"unlock")==0)
      {
          if(SENSOR == 0 && is_locked == 1){
                       is_jammed =1;
                    }

                    else if(SENSOR == 1 && is_locked ==0)
                    {
                        is_jammed =1;
                    }

          if(is_locked == 0 && is_jammed == 0 && SENSOR ==0 )
          {
              putsUart0("Already Unlocked\r\n");
          }

          else if (is_locked == 1 && is_jammed == 0 && SENSOR == 1)
            {
              motor_unlock();
              is_locked = 0;

            }

          else if(SENSOR == 1 && is_jammed ==1){
                           motor_unlock();
                           is_jammed =0;
                            //putsUart0("Jammed\r\n");
                        }
      }


      else if(strcmp(strInput,"status")==0)
      {
          status();
      }


      else if(strcmp(strInput,"local off")==0)
      {
          if(is_local_on == 0)
          {
              putsUart0("Local already off\r\n");
          }

          else if(is_local_on == 1)
          {
              is_local_on = 0;
              GPIO_PORTF_IM_R &= ~(PB1_MASK);
          }
      }


      else if(strcmp(strInput,"local on")==0)
      {
          if(is_local_on == 0)
          {
              is_local_on = 1;
              GPIO_PORTF_IM_R |= PB1_MASK;

          }

          else if(is_local_on == 1)
          {
              putsUart0("Local already on\r\n");

          }
      }

      else if(strcmp(strInput,"beeper on")==0)
      {
          if(is_beeper_on == 1)
          {
              putsUart0("Beeper already on\r\n");
          }
          else if(is_beeper_on == 0)
          {
              is_beeper_on =1;
          }
      }

      else if(strcmp(strInput,"beeper off")==0)
      {
          if(is_beeper_on == 0)
          {
              putsUart0("Beeper already off\r\n");
          }
          else if(is_beeper_on == 1)
          {
              is_beeper_on =0;
          }
      }

      else if(strcmp(strInput,"set password")==0)
      {
          is_password = 1;
          waitMicrosecond(10000000); //wait ten seconds
          is_password = 0;
      }

      else if(strcmp(strInput, "time") == 0)
      {
          setTime();
      }

      else if(strcmp(strInput, "auto set") == 0)
      {
          if(auto_enabled == 1)
          {
              setRTC();
          }

          else
          {
              putsUart0("The auto lock is disabled.");
          }
      }

      else if(strcmp(strInput, "auto enable") == 0)
      {
          auto_enabled = 1;
          putsUart0("The auto lock is enabled.");
      }

      else if(strcmp(strInput, "led enable") == 0)
            {
                is_led = 1;
                putsUart0("LED enabled.");
            }

      else if(strcmp(strInput, "led disable") == 0)
                  {
                      is_led = 0;
                      RED_LED =0;
                      GREEN_LED =0;
                      putsUart0("LED disabled.");

                  }

      else if(strcmp(strInput, "auto disable") == 0)
      {
          auto_enabled = 0;
          putsUart0("The auto lock is now disabled.");
      }

      else
      {
          putsUart0("Please enter a valid command.");
      }
   }

return 0;
}
