#include "ComDef.h"
#include <string.h>
#include "plib035.h"

extern uint16_t system_status;
extern uint32_t systick_counter;

extern uint16_t mb_input_regs[];

extern float p1_adc_f, t1_adc_f;
extern uint16_t p1_adc_n, t1_adc_n;

extern uint16_t p1_adc_buffer[], t1_adc_buffer[];
extern uint8_t p1_buf_index, t1_buf_index;
extern uint16_t p1_adc_max, p1_adc_min;
extern uint16_t t1_adc_max, t1_adc_min;

extern uint8_t p1_sample_volume, t1_sample_volume;
extern uint8_t p1_buffer_full, t1_buffer_full;
extern uint32_t p1_sliding_sum, t1_sliding_sum;

void SysTick_Handler(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
	
	if (((system_status & STATE_MB_POLL)==0)&&(systick_counter&1)) system_status |= STATE_MB_POLL;
	systick_counter++;
}

//void TMR1_IRQHandler(void)
//{
//	TMR_ITStatusClear(TMR1);
//}

void ADC_SEQ0_IRQHandler()
{
	uint32_t current_data;
    ADC_SEQ_ITStatusClear(ADC_SEQ_Num_0);
	current_data = ADC_SEQ_GetFIFOData(ADC_SEQ_Num_0);

	// skip zero data
	if (current_data != 0)
	{
		// update sliding sum
		if (p1_buffer_full) p1_sliding_sum += current_data - p1_adc_buffer[p1_buf_index];
		else p1_sliding_sum += current_data;
		
		p1_adc_buffer[p1_buf_index++] = current_data;
	}
	
	if (p1_buf_index == p1_sample_volume)
	{
		p1_buf_index = 0;
		p1_buffer_full = 1;
	}
}

void ADC_SEQ1_IRQHandler()
{
	uint32_t current_data;
    ADC_SEQ_ITStatusClear(ADC_SEQ_Num_1);
	current_data = ADC_SEQ_GetFIFOData(ADC_SEQ_Num_1);
	
	// skip zero data
	if (current_data != 0)
	{
		// update sliding sum
		if (t1_buffer_full) t1_sliding_sum += current_data - t1_adc_buffer[t1_buf_index];
		else t1_sliding_sum += current_data;
		
		t1_adc_buffer[t1_buf_index++] = current_data;
	}
	
	if (t1_buf_index == t1_sample_volume)
	{
		t1_buf_index = 0;
		t1_buffer_full = 1;
	}
}
