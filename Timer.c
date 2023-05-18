#include "Timer.h"
#include "GPIO.h"
#include <stdbool.h>
void timer_init(Gpt_ConfigType *totalconfig)
{  
	unsigned long volatile delay;
  int value;  
	if (totalconfig->GptId >= 0 && totalconfig->GptId <= 5) {
           SET_BIT_PERIPH_BAND(SYSCTL->RCGCTIMER , (totalconfig->GptId));
      }else {
   	      value= (totalconfig->GptId)-5;   
			    SET_BIT_PERIPH_BAND(SYSCTL->RCGCWTIMER , (value));
			}
			delay =0;
      delay =0;			
	   totalconfig->TIMERx->CTL=0x00000000;
   switch(totalconfig->mode16OR32)
   {
		 case GPTm_MODE_16bit:
			  totalconfig->TIMERx->CFG=0x00000004;
		    break;
		 case GPTm_MODE_32bit:
        totalconfig->TIMERx->CFG=0x00000000;
		    break;			 
		 default:
			 break;
	 
	 }		 
	 switch(totalconfig->Gpt_Mode)
   {
		 case GPTm_MODE_CONTINUOUS:
			 if(totalconfig->BlockID==TIMERA){ 
		   totalconfig->TIMERx->TAMR=0x00000002;
			 }
			 else{
			 totalconfig->TIMERx->TBMR=0x00000002;
			  }
		    break;
		 case GPTm_MODE_ONESHOT:
       	 if(totalconfig->BlockID==TIMERA){ 
		   totalconfig->TIMERx->TAMR=0x00000001;
			 }
			 else{
			 totalconfig->TIMERx->TBMR=0x00000001;
			  }
		    break;			 
		 default:
			 break;
	 }	
	
	 switch(totalconfig->GptChannelCount)
   {
		 case COUNT_UP:
			if(totalconfig->BlockID==TIMERA){ 
		   totalconfig->TIMERx->TAMR |= (1<<4);
			 }
			 else{
			 totalconfig->TIMERx->TBMR|= (1<<4);
			  }
		    break;
		 case COUNT_DOWN:
      if(totalconfig->BlockID==TIMERA){ 
		   totalconfig->TIMERx->TAMR &= ~(1<<4);
			 }
			 else{
			 totalconfig->TIMERx->TBMR &= ~(1<<4);
			  }
		    break;			 
		 default:
			 break;
	 }
  	 if(totalconfig->BlockID==TIMERA)
		 {totalconfig->TIMERx->TAILR=totalconfig->loadValue-1; 
		    // 6) clear timer2A timeout flag
         totalconfig->TIMERx->ICR |= 0x00000001;			 
       // 7) arm timeout interrupt
			   totalconfig->TIMERx->IMR |= 0x00000001;
         NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000; // 8) priority 4
    // interrupts enabled in the main program after all devices initialized
   // vector number 39, interrupt number 23
         NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
		     totalconfig->TIMERx->CTL |= 0x00000001;
		 }
		 else{
	     totalconfig->TIMERx->TBILR=totalconfig->loadValue-1;
			  totalconfig->TIMERx->ICR |= (1<<8);			 
       // 7) arm timeout interrupt
			   totalconfig->TIMERx->IMR |= (1<<8);
         NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x00000000; // 8) priority 4
    // interrupts enabled in the main program after all devices initialized
   // vector number 39, interrupt number 23
         NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
			  totalconfig->TIMERx->CTL |= (1<<8);
		 }
		 
}
void clear_Int(TIMER0_Type *TIMERx,block_type BlockID){
 if(BlockID==TIMERA){ 
	TIMERx->ICR |=  0x00000001;
 }
 else
 {
  TIMERx->ICR |=  (1<<8);
 }

}
	

