
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
// Function Declararions
void WaitForInterrupt(void);  // low power mode
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Timer2_Init(unsigned long period);
void game();
unsigned long ADC0_InSeq3(void);
void Draw_Cars();
int Check_Col();
// ********************* Variables ***************

// External 
extern GPIO_pins_config_t PORTF_ARR; 
extern GPIO_pins_config_t PORTb;
extern Gpt_ConfigType Timer2;


// Internal
unsigned int TimerCount=0; // used to calc the score 
long ypos = 28; // for y movement of the car
float ratio =0; // for adc
volatile int carPos=0;
int x=0;
int rCarWidth=16;
int rCarHeight=10;
int game_state=SAFE;
int yCheck;
int lives=3;
int status=0;
int game_started=0;
volatile int adcData;






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

	// Helper Functions
	void Delay100ms(unsigned long count){unsigned long volatile time;
  while(count>0){
    time = 227240;  // 0.1sec at 80 MHz
   while(time){
	  	time--;
    }
    count--;
  }
}

	
	
	
	// ADC Functions
	
void ADc1_Init(void){ volatile unsigned long delay;
	// work on ADC1 , SS3 ,PE1
 		// we work on ADC 1 (IN VEDIO), sequencer 3 (One simple/ trigger) , PE1(AN2) "IN vedio also "	
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
// timer handler (draws the cars and checks collosion )
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


// draw  Cars 


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
// check if there's a collosion
int Check_Col(){
	char message[100];
	yCheck=carPos+(rCarHeight*3);
	if((ypos<yCheck && ypos>carPos  )&& x>=58){
			if(lives==1){
				Nokia5110_PrintBMP(60,ypos , BigExplosion1, 0);
				Nokia5110_DisplayBuffer();
				Delay100ms(10);
				Nokia5110_Clear();
				GPIO_write_pin(GPIOF,PIN_1,1);
				Nokia5110_SetCursor(1, 1);
				Nokia5110_OutString("GAME OVER");
				Nokia5110_SetCursor(1, 2);
				sprintf(message, "Nice try,your score is: %u",TimerCount);
				Nokia5110_OutString(message);
				lives--;
				Delay100ms(30);
				CLEAR_BIT_PERIPH_BAND(GPIOF->DATA,PIN_1);
				return DEAD;
			} 
			else {
				Nokia5110_PrintBMP(60,ypos , BigExplosion1, 0);
				Nokia5110_DisplayBuffer();
				Delay100ms(10);
			Nokia5110_Clear();
		  sprintf(message, "Ooooops!!   collosion!!  You got %d   Lives left ", --lives);
				// turn on the led
				GPIO_write_pin(GPIOF,PIN_1,1);
				Nokia5110_Clear();
				Nokia5110_OutString(message);
				Delay100ms(30);
				// turn it off
				CLEAR_BIT_PERIPH_BAND(GPIOF->DATA,PIN_1);
				 status=1;
			}
	}
	return SAFE;
	
}


// Interrupt Functions 
// to enable the interrupt for certain  IRQN
void IntCtrl_EnableIRQ(IRQn_Type interruptIRQn)
{
	if(interruptIRQn >= 0)
	{
			SET_BIT_PERIPH_BAND_VAL(NVIC->ISER[(interruptIRQn)/32],1<<(uint8_t)interruptIRQn % 32);
	}
}
 // interrupt init for port b (used for push button)
void interrupt_init(){

IntCtrl_EnableIRQ(GPIOB_IRQn);    // (h) enable interrupt 10 in NVIC for Port B

GPIO_PORTB_IM_R |= 0x10;    // (f) arm interrupt on PB4
GPIO_PORTB_IS_R = 0x00;     // (d) PB4 is edge-sensitive
GPIO_PORTB_IBE_R = 0x00;    //     PB4 is not both edges
GPIO_PORTB_IEV_R = 0x00;    //     PB4 falling edge event
GPIO_PORTB_ICR_R = 0x10;    // (e) clear flag4
}
void GPIOPortB_Handler(){
//	NVIC_ST_CTRL_R = 0; 
	if(GPIO_PORTB_RIS_R & (1 << 4))
	 {
			GPIO_PORTB_ICR_R = 0x10;	
		 	game_state=SAFE;
		  lives=3;
		 
		  timer_init(&Timer2);
      game_started=1; 
	 }

}	

int main(void){
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
	EnableInterrupts();  
	Game_init(&PORTF_ARR);
	GPIO_init(&PORTb);
	interrupt_init();
  Random_Init(1);
  ADc1_Init();
	Nokia5110_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer
	// Draw First Message
	Nokia5110_Clear();		
	Nokia5110_SetCursor(3, 1);
	Nokia5110_OutString("Press");
	Nokia5110_SetCursor(0, 3);
	Nokia5110_OutString(" The Button ");
	Nokia5110_SetCursor(2, 5);
	Nokia5110_OutString("To Start:)");

	while(1)
	{	
	 if(game_started==1)
	 {
    		 
		if(game_state==SAFE){
				//Nokia5110_Clear();
				  ADC1_PSSI_R = 0x0008; 
			    WaitForInterrupt();
			    if((ADC1_RIS_R & (1<<3)) == (1<<3))
          {
	         ADC1_ISC_R |=(1<<3); // clear the interrupt bit
           adcData = ADC1_SSFIFO3_R&0xFFF;
          }
			    ratio=((48 * ((float)(adcData)/(4095.0)))+12)/(48+12);
			    ypos = ratio * 48;
			    Nokia5110_PrintBMP(68,ypos , rCar, 0); 
					Nokia5110_DisplayBuffer();					 		
						}
					else{			 
								game_started=0;
							Nokia5110_Clear();		
							Nokia5110_SetCursor(3, 1);
							Nokia5110_OutString("Press");
							Nokia5110_SetCursor(0, 2);
							Nokia5110_OutString(" The Button ");
							Nokia5110_SetCursor(2, 3);
							Nokia5110_OutString("To Play ");
							Nokia5110_SetCursor(2,5);
							Nokia5110_OutString(" Again:)");
					}
				}
			}
		}






