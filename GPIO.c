#include "GPIO.h"
#include "GPIO_cnfg.h"



volatile int delay;
int portNum, pinNum ;
void Game_init( GPIO_pins_config_t * totalConfig)
 { int i;
	 	SET_BIT_PERIPH_BAND(SYSCTL->RCGCGPIO , (totalConfig->portNum));
	  SET_BIT_PERIPH_BAND(SYSCTL->RCGC2 , (totalConfig->portNum));
	 delay = 0;
	 totalConfig->GPIOx->LOCK = 0x4C4F434B;
	 totalConfig->GPIOx->CR=0xff;
	 for(i=0;i<MAX_PINS_NUM;i++)
	 {
	   GPIO_init(&totalConfig[i]);
	 }
 }


return_type GPIO_init( GPIO_pins_config_t * totalConfig)
{
	switch(totalConfig->pinDirection)
	{
		case INPUT:
			CLEAR_BIT_PERIPH_BAND(totalConfig->GPIOx->DIR,totalConfig->pinNumber);
			break;
		case OUTPUT:
			SET_BIT_PERIPH_BAND(totalConfig->GPIOx->DIR,totalConfig->pinNumber);
			break;
		default:
			break;
	}
  if(totalConfig->altFunctionSelect == GPIO){
	  CLEAR_BIT_PERIPH_BAND(totalConfig->GPIOx->AFSEL,totalConfig->pinNumber);
	  CLEAR_BIT_PERIPH_BAND(totalConfig->GPIOx->PCTL,totalConfig->pinNumber);
	}
else{
  SET_BIT_PERIPH_BAND(totalConfig->GPIOx->AFSEL,totalConfig->pinNumber);
}
	switch(totalConfig->PullUPOrDown)
			{
				case PULL_UP:
						SET_BIT_PERIPH_BAND(totalConfig->GPIOx->PUR,totalConfig->pinNumber);
						break;
				case PULL_DOWN:
						SET_BIT_PERIPH_BAND(totalConfig->GPIOx->PDR,totalConfig->pinNumber);
						break;
				default:
						break;
			}	
			
	if(totalConfig->digitalAnalogSelect == DIGITAL)
	{
		CLEAR_BIT_PERIPH_BAND(totalConfig->GPIOx->AMSEL,totalConfig->pinNumber);
		SET_BIT_PERIPH_BAND(totalConfig->GPIOx->DEN,totalConfig->pinNumber);
	}
	else
	{
	  SET_BIT_PERIPH_BAND(totalConfig->GPIOx->AMSEL,totalConfig->pinNumber);
		CLEAR_BIT_PERIPH_BAND(totalConfig->GPIOx->DEN,totalConfig->pinNumber);
	}
	
	
	
	return SUCC;
}



int GPIO_read_pin( GPIOA_Type *GPIOx , GPIO_PINS pin)
{
			char data;    
			data =GPIOx->DATA & (1<<pin);
	return data;
}

return_type GPIO_write_pin( GPIOA_Type *GPIOx, GPIO_PINS pin, int data)
{
			if( data == 0)
			{
		   	GPIOx->DATA &= ~(1<<pin);
			}
			else
			{
				GPIOx->DATA = (1<<pin); 
			}
	return SUCC;
}
