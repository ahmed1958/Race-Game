#include "GPIO.h"
#include "GPIO_cnfg.h"
GPIO_pins_config_t PORTb=
{
    GPIOB,
		PORT_B,
		PIN_4,
		INPUT,
		GPIO,
		PULL_UP,
		DIGITAL
};

GPIO_pins_config_t PORTF_ARR[8]=
{
		{GPIOF,
		PORT_F,
		PIN_0,
		INPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_1,
		OUTPUT,
		GPIO,
		PULL_DOWN,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_2,
		OUTPUT,
		GPIO,
		PULL_DOWN,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_3,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_4,
		INPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_5,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_6,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOF,
		PORT_F,
		PIN_7,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
};



GPIO_pins_config_t PORTE_ARR[8]=
{
		{GPIOE,
		PORT_E,
		PIN_0,
		OUTPUT,
		ALTF,
		PULL_UP,
		ANALOG},
		
		{GPIOE,
		PORT_E,
		PIN_1,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_2,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_3,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_4,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_5,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_6,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
		
		{GPIOE,
		PORT_E,
		PIN_7,
		OUTPUT,
		GPIO,
		PULL_UP,
		DIGITAL},
};
