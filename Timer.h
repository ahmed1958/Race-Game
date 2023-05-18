#include "SS.h"
#include "TM4C123GH6PM11.h"
typedef enum
{
	GPtm_TIMER0,
	GPtm_TIMER1,
	GPtm_TIMER2,
	GPtm_TIMER3,
	GPtm_TIMER4,
	GPtm_TIMER5,
	GPtmW_TIMER0,
	GPtmW_TIMER1,
	GPtmW_TIMER2,
	GPtmW_TIMER3,
	GPtmW_TIMER4,
	GPtmW_TIMER5	
}Gptm_Type;
typedef enum
{
	TIMERA,
	TIMERB
}block_type;
typedef enum
{
	COUNT_UP,
	COUNT_DOWN
}Gptm_CountType;
typedef enum
{
	GPTm_MODE_CONTINUOUS,
	GPTm_MODE_ONESHOT
}Gptm_ModeType;
typedef enum
{
	GPTm_MODE_16bit,
	GPTm_MODE_32bit
}Gptm_16or32;
typedef struct
{
	TIMER0_Type *TIMERx;
	Gptm_Type GptId;
	Gptm_16or32 mode16OR32;
	block_type BlockID;
	uint32_t loadValue;
	Gptm_ModeType Gpt_Mode;
	Gptm_CountType	GptChannelCount;
}Gpt_ConfigType;

void timer_init(Gpt_ConfigType *totalconfig);
void clear_Int(TIMER0_Type *TIMERx,block_type BlockID);