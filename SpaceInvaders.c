// SpaceInvaders.c
// Runs on LM4F120/TM4C123
// Jonathan Valvano and Daniel Valvano
// This is a starter project for the edX Lab 15
// In order for other students to play your game
// 1) You must leave the hardware configuration as defined
// 2) You must not add/remove any files from the project
// 3) You must add your code only this this C file
// I.e., if you wish to use code from sprite.c or sound.c, move that code in this file
// 4) It must compile with the 32k limit of the free Keil

// April 10, 2014
// http://www.spaceinvaders.de/
// sounds at http://www.classicgaming.cc/classics/spaceinvaders/sounds.php
// http://www.classicgaming.cc/classics/spaceinvaders/playguide.php
/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

   "Embedded Systems: Introduction to Arm Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// ******* Required Hardware I/O connections*******************
// Slide pot pin 1 connected to ground
// Slide pot pin 2 connected to PE2/AIN1
// Slide pot pin 3 connected to +3.3V 
// fire button connected to PE0
// special weapon fire button connected to PE1
// 8*R resistor DAC bit 0 on PB0 (least significant bit)
// 4*R resistor DAC bit 1 on PB1
// 2*R resistor DAC bit 2 on PB2
// 1*R resistor DAC bit 3 on PB3 (most significant bit)
// LED on PB4
// LED on PB5

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// SSI0Fss       (SCE, pin 3) connected to PA3
// Reset         (RST, pin 4) connected to PA7
// Data/Command  (D/C, pin 5) connected to PA6
// SSI0Tx        (DN,  pin 6) connected to PA5
// SSI0Clk       (SCLK, pin 7) connected to PA2
// back light    (LED, pin 8) not connected, consists of 4 white LEDs which draw ~80mA total

#include "GPIO.h"
#include "GPIO_cnfg.h"
#include "Nokia5110.h"
#include "Random.h"
#include "TExaS.h"
#include "UART.h"
#include <stdio.h>
#include "tm4c123gh6pm.h"
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define IMAGEW      ((unsigned char)image[18])
#define IMAGEH      ((unsigned char)image[22])
	
	
	
// *************************** Capture image dimensions out of BMP**********

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
void UART_Init(void);
volatile int adcData;
unsigned char UART_InChar(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  while((UART0_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART0_DR_R&0xFF));
}
unsigned char UART_InCharNonBlocking(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  if((UART0_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART0_DR_R&0xFF));
  } else{
    return 0;
	}
}
void UART_OutChar(unsigned char data){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}
void Delay100ms(unsigned long count){unsigned long volatile time;
  while(count>0){
    time = 227240;  // 0.1sec at 80 MHz
   while(time){
	  	time--;
    }
    count--;
  }
}


void WaitForInterrupt(void);  // low power mode
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Timer2_Init(unsigned long period);


unsigned long TimerCount;
unsigned long Semaphore;
void PortF_Init(void);
long ypos = 28;

unsigned long SW1,SW2;  // input from PF4,PF0
char character;


// *************************** Images ***************************
const unsigned char rCar[] ={
 0x42, 0x4D, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x60, 0x06, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x26, 0x66, 0x66, 0x26, 0x00, 0x00, 0x02,
 0x66, 0x62, 0x66, 0x26, 0x62, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x20, 0x00, 0x00, 0x00, 0x02, 0x66, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF,
};

// large explosion that can be used upon the demise of the player's ship (second frame)
// width=18 x height=8
const unsigned char BigExplosion1[] = {
 0x42, 0x4D, 0xD6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x0E, 0x00, 0x09, 0x00, 0x09, 0x00, 0xB0, 0x00, 0xA0, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA9, 0x00, 0x00, 0x00, 0x00, 0x90,
 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x0A, 0x00, 0x90, 0x00, 0xB0, 0x00, 0x09, 0x00, 0x00, 0x00, 0xFF};

// decleration of function 
////////////////////////////////////
void ADC_Init(void){
// we work on ADC 1 (IN VEDIO), sequencer 3 (One simple/ trigger) , PE1(AN2) "IN vedio also "	
 int delay;
//Enable the ADC clock using the RCGCADC register 
  SYSCTL_RCGCADC_R |= (1<<1); // Enable ADC module 1


//Enable the clock to the appropriate GPIO modules via the RCGCGPIO register
 SYSCTL_RCGCGPIO_R |=(1<<4)//Enable port E // Enable port F 
                   |(1<<5);
	delay=SYSCTL_RCGCGPIO_R;
//Set the GPIO AFSEL for PORT E --> enable alternative function PE1
  GPIO_PORTE_AFSEL_R |= (1<<1);

// Clear the GPIO_DEN register == Disable digital functionality for PE1
   GPIO_PORTE_DEN_R &= ~(1<<1);
 
// SET the GPIO_AMSEL Register == Enable analog functionality for PE1
   GPIO_PORTE_AMSEL_R |= (1<<1);
//------------------------------------------------------------config of sequencer------------------------------------
//Sample Sequencer 3 is disabled.
ADC1_ACTSS_R &= ~(1<<3);

// Sample Sequencer 3 is Always running (continuously sample)
// We shift to bit 12 to select SS3 what we work on
ADC1_EMUX_R = (0xf<<12);

//Sample Sequencer 3 reads from AIN2(PE1)
  ADC1_SSMUX3_R= 2;

ADC1_SSCTL3_R |= (1<< 1)// End bit must be set 
                | (1<< 2); // Enable interuput for SS3 

//Enable interrupt Mask for SS3 
ADC1_IM_R |= (1<<3);

ADC1_RIS_R |= (1<<3);
ADC1_ACTSS_R |= (1<<3);
NVIC_EN2_R |=(1<<3);
}		



// draw opponent cars 
volatile int carPos=0;
int x=0;

int rCarWidth=16;
int rCarHeight=10;


// systick timer init/handler
void SysTick_Init(void)
{
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_RELOAD_R =  23992399;      // reload value for 300ms  at 80MHz
  NVIC_ST_CURRENT_R = 0;            // clear current count
  NVIC_ST_CTRL_R |= 0x07;           // enable SysTick with system clock
}

void Draw_Cars(){
   	Nokia5110_ClearBuffer();
		Nokia5110_PrintBMP(68, ypos , rCar, 0);
		Nokia5110_DisplayBuffer();
	 Nokia5110_PrintBMP(x, carPos+10, rCar, 0);
    Nokia5110_DisplayBuffer();
	 Nokia5110_PrintBMP(x, carPos+18, rCar, 0);
    Nokia5110_DisplayBuffer();
    Nokia5110_PrintBMP(x, carPos+26, rCar, 0);
    Nokia5110_DisplayBuffer();
	
}
int flag=0;
int yCheck;
int lives=1;
unsigned char xc,yc;
int Check_Col(){
	char message[100];
	yCheck=carPos+(rCarHeight*3);
	if((ypos<yCheck && ypos>carPos  )&& x>=60 && flag==0){
			if(lives==1){
					Nokia5110_Clear();
				Nokia5110_SetCursor(1, 1);
				Nokia5110_OutString("GAME OVER");
				Nokia5110_SetCursor(1, 2);
				Nokia5110_OutString("Nice try,");
				flag=1;
				lives--;
				Delay100ms(20);
				return 1;
			} else {
			Nokia5110_Clear();
		  sprintf(message, "Ooooops!!   collosion!!  You got %d   Lives left ", --lives);
				Nokia5110_Clear();
				Nokia5110_OutString(message);
				Delay100ms(20);
			}
			return 0;
	}
	return 0;
}
void SysTick_Handler(void)
{
		if(Check_Col()==1){
				return;
		}
		if(!flag){
					Draw_Cars();
		  if(x>=60){
			carPos=(carPos+10)%30;
		}
				x=(x+3) % 62;
	}

	
}
int main(void){
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
  Random_Init(1);
  ADC_Init();
	Nokia5110_Init();
	SysTick_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer
	PortF_Init();
	EnableInterrupts();  // The grader uses interrupts
  WaitForInterrupt();

	while(1){
			if(!flag){
							Nokia5110_PrintBMP(68, ypos , rCar, 0);
							Nokia5110_DisplayBuffer();
							character = UART_InChar();
							UART_OutChar(character); 
						 if(character == 'a'){
									ypos = ypos + 1;
									if(ypos >=47) {	
										ypos = 47;
									}
							}
						else if(character== 'b'){
									ypos = ypos - 1;
									if(ypos <= 9) {
											ypos = 9;
									}
							}
						Nokia5110_ClearBuffer();
						Draw_Cars();
						}
				else if(flag==1) {
					Nokia5110_Clear();
					Nokia5110_OutString("Press       The Button To Play Again :)");
					flag=2;
				} 
				else{
					
				}
				
				}
		
}




void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}
void UART_Init(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
//                 switching from PC5,PC4 to PA1,PA0
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 43;                    // IBRD = int(80,000,000 / (16 * 115200)) = int(43.402778)
  UART0_FBRD_R = 26;                    // FBRD = round(0.402778 * 64) = 26
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1,PA0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1,PA0
                                        // configure PA1,PA0 as UART0
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA1,PA0
}



