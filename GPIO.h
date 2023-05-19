#include "GPIO_cnfg.h"
#include "SS.h"
#include "TM4C123GH6PM11.h"
#define GPIOPORTS   ((GPIOA_Type *) GPIOA_BASE)

#define PERIPH_BIT_BAND_ALIAS				0x42000000UL
#define PERIPH_BIT_BAND_REGION			0x40000000UL

# define STD_HIGH     1u /* Physical state 5V or 3.3V */
# define STD_LOW      0u /* Physical state 0V */
#define SET_BIT_PERIPH_BAND(REG,PIN)										((*(volatile uint32_t *)(PERIPH_BIT_BAND_ALIAS + ((uint32_t)(&REG) - PERIPH_BIT_BAND_REGION ) * 32 + PIN *4  )) = STD_HIGH )
#define CLEAR_BIT_PERIPH_BAND(REG,PIN)									((*(volatile uint32_t *)(PERIPH_BIT_BAND_ALIAS + ((uint32_t)(&REG) - PERIPH_BIT_BAND_REGION ) * 32 + PIN *4  )) = STD_LOW )	
#define SET_BIT_PERIPH_BAND_VAL(REG,VALUE)							((*(volatile uint32_t *)(((uint32_t)(&REG)))) = VALUE )

typedef enum  
{
	SUCC, NOTSUCC
}return_type;

typedef enum  
{
	PORT_A,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	PORT_F
}GPIO_PORTS;

typedef enum  
{
	PIN_0,
	PIN_1,
	PIN_2,
	PIN_3,
	PIN_4,
	PIN_5,
	PIN_6,
	PIN_7	
}GPIO_PINS;

typedef enum  
{
	INPUT=0,
	OUTPUT
}pin_dir;

typedef enum  
{
	GPIO, ALTF
}alt_function;

typedef enum  
{
	PULL_UP=0,
	PULL_DOWN
}internaAttach;

typedef enum  
{
	DIGITAL, ANALOG
}dig_ana_select;

typedef struct GPIO_pins_config_s
{
	GPIOA_Type *GPIOx;
	GPIO_PORTS	portNum;
	GPIO_PINS pinNumber;
	pin_dir pinDirection;
	alt_function altFunctionSelect;
	internaAttach PullUPOrDown;
	dig_ana_select digitalAnalogSelect; 
	
}GPIO_pins_config_t;

typedef struct GPIO_port_config_s
{
	GPIO_pins_config_t pinsConfig[MAX_PINS_NUM];
	
}GPIO_port_config_t;


return_type GPIO_init( GPIO_pins_config_t * totalConfig);
int GPIO_read_pin( GPIOA_Type *GPIOx, GPIO_PINS pinNum);
return_type GPIO_write_pin( GPIOA_Type *GPIOx, GPIO_PINS pinNum, int data);
void Game_init( GPIO_pins_config_t * totalConfig);



