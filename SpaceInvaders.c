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
#include "Timer.h"
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
	


// Collision macros
#define DEAD (1)
#define SAFE (0)
#define WAITING (2)
unsigned long ADC0_InSeq3(void);
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
void game();
// ********************** Variables **********************
// external variables 
extern Gpt_ConfigType Timer2;
extern GPIO_pins_config_t PORTF_ARR; 
extern GPIO_pins_config_t PORTb;

// internal vars
unsigned int TimerCount=0;
unsigned long Semaphore;
void PortF_Init(void);
long ypos = 28;
unsigned long SW1,SW2;  // input from PF4,PF0
char character;
float ratio =0;
volatile int carPos=0;
int x=0;
int rCarWidth=16;
int rCarHeight=10;
int game_state=SAFE;
int yCheck;
int lives=3;
int status=0;
int game_started=0;


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

void ADC0_Init(void){ volatile unsigned long delay;
	// work on ADC0 , SS3 ,PE2
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x04;      // 2) make PE2 input
  GPIO_PORTE_AFSEL_R |= 0x04;     // 3) enable alternate function on PE2
  GPIO_PORTE_DEN_R &= ~0x04;      // 4) disable digital I/O on PE2
  GPIO_PORTE_AMSEL_R |= 0x04;     // 5) enable analog function on PE2
  SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
  ADC0_SSMUX3_R = (ADC0_SSMUX3_R&0xFFFFFFF0)+1; // 11) channel Ain1 (PE2)
  ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  ADC0_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3
	//Enable interrupt Mask for SS3
	ADC1_IM_R |= (1<<3);
  ADC1_RIS_R |= (1<<3);
  ADC1_ACTSS_R |= (1<<3);
  NVIC_EN2_R |=(1<<3);
}

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

int Check_Col(){
	char message[100];
	yCheck=carPos+(rCarHeight*3);
	if((ypos<yCheck && ypos>carPos  )&& x>=58){
			if(lives==1){
				Nokia5110_PrintBMP(60,ypos , BigExplosion1, 0);
				Nokia5110_DisplayBuffer();
				Delay100ms(5);
				Nokia5110_Clear();
				GPIO_write_pin(GPIOF,PIN_1,1);
				Nokia5110_SetCursor(1, 1);
				Nokia5110_OutString("GAME OVER");
				Nokia5110_SetCursor(1, 2);
				sprintf(message, "Nice try,your score is: %u",TimerCount);
				Nokia5110_OutString(message);
				lives--;
				Delay100ms(20);
				CLEAR_BIT_PERIPH_BAND(GPIOF->DATA,PIN_1);
				return DEAD;
			} 
			else {
				Nokia5110_PrintBMP(60,ypos , BigExplosion1, 0);
				Nokia5110_DisplayBuffer();
				Delay100ms(5);
			Nokia5110_Clear();
		  sprintf(message, "Ooooops!!   collosion!!  You got %d   Lives left ", --lives);
				GPIO_write_pin(GPIOF,PIN_1,1);
				Nokia5110_Clear();
				Nokia5110_OutString(message);
				Delay100ms(20);
				CLEAR_BIT_PERIPH_BAND(GPIOF->DATA,PIN_1);
				 status=1;
			}
	}
	return SAFE;
	
}

void Timer2A_Handler(void)
{	
	clear_Int(TIMER2,TIMERA);
	TimerCount++;
if(game_started==1)
{
 if(status==0)
 { 
	if(Check_Col()==DEAD){
			game_state=DEAD; //
			   x=0;
				return;
		}
	}
 else{
   status=0; 
 }
		
		 if(x>=60){
			carPos=(carPos+10)%30;
			}
			x=(x+3) % 62;
      Draw_Cars();			
}}

void IntCtrl_EnableIRQ(IRQn_Type interruptIRQn)
{
	if(interruptIRQn >= 0)
	{
			SET_BIT_PERIPH_BAND_VAL(NVIC->ISER[(interruptIRQn)/32],1<<(uint8_t)interruptIRQn % 32);
	}
}

void interrupt_init(){

IntCtrl_EnableIRQ(GPIOB_IRQn);    // (h) enable interrupt 10 in NVIC for Port B

GPIO_PORTB_IM_R |= 0x10;    // (f) arm interrupt on PB4
GPIO_PORTB_IS_R = 0x00;     // (d) PB4 is edge-sensitive
GPIO_PORTB_IBE_R = 0x00;    //     PB4 is not both edges
GPIO_PORTB_IEV_R = 0x00;    //     PB4 falling edge event
GPIO_PORTB_ICR_R = 0x10;    // (e) clear flag4
}

void GPIOPortB_Handler(){
	if(GPIO_PORTB_RIS_R & (1 << 4))
	 {
			GPIO_PORTB_ICR_R = 0x10;	
		 	game_state=SAFE;
		  lives=3;
		 
		  timer_init(&Timer2);
      game_started=1; 
	 }

}	


unsigned long ADC0_InSeq3(void){  
	unsigned long result;
  ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}


int main(void){
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
	EnableInterrupts();  
	Game_init(&PORTF_ARR);
	GPIO_init(&PORTb);
	interrupt_init();
  Random_Init(1);
  ADC0_Init();
	Nokia5110_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer
			
	Nokia5110_Clear();		
	Nokia5110_OutString("Press       The Button If You Wanna Play :)");
 

	while(1)
	{	
	 if(game_started==1)
	 {
    		 
		if(game_state==SAFE){
			    Nokia5110_DisplayBuffer();
			    adcData = ADC0_SSFIFO3_R&0xFFF;
					adcData=ADC0_InSeq3();
					ADC0_ISC_R = 0x0008; 
				  ratio=((48 * ((float)(adcData)/(4095.0)))+12)/(48+12);
			    ypos = ratio * 48;
			    Nokia5110_PrintBMP(68,ypos , rCar, 0);
			    WaitForInterrupt();	
					Nokia5110_DisplayBuffer();
						}
					else{			 
								game_started=0;
						    Nokia5110_Clear();
								Nokia5110_OutString("Press       The Button To Play Again :)");
					}
				}
			}
		}





