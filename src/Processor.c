#include <stdio.h>
#include <string.h>
#include "Processor.h"
#include "..\modbus\mbdefs.h"
#include "..\modbus\mbcrc.h"
#include "plib035_mflash.h"
#include "ComDef.h"

// -- Constant values ----------------------------------------------------------
const float A_VDD = 3.3f;
const uint8_t ITERATION_CAP = 10;

const uint8_t P1_GRADE = 3;
const uint8_t T1_GRADE = 9;
const uint8_t FACTORS_COUNT = (P1_GRADE + 1)*(T1_GRADE + 1);

// -- Variables ----------------------------------------------------------------
float tmp;
float p_out1 = 0, p_out1_0 = 0.5F, p_out1_bak = 0.5F;
float p_out1_max = 1.0F, p_out1_min = 0.01F; // kgs/cm2
float err_out1 = 0.0001F;	// kgs/cm2
uint8_t iteration_cap = 10;
float t1_sub_value = 0;
float dt1_corr_min = 0, dt1_corr_max = 0;

// main P1 factors
float polynom_p1_factors[P1_GRADE + 1];
// whole set of correction factors
float polynom_factors[FACTORS_COUNT];

// adc sample volumes
uint16_t t1_sample_volume, p1_sample_volume;
// adc sampling ranges
float p1_adc_max, p1_adc_min;
float t1_adc_max, t1_adc_min;

uint8_t p1_buffer_full = 0, t1_buffer_full = 0;

// adc buffer sliding sums
uint32_t p1_sliding_sum = 0, t1_sliding_sum = 0;

// adc sampling buffers
uint16_t p1_adc_buffer[MAX_P1_BUF_VOLUME];
uint16_t t1_adc_buffer[MAX_T1_BUF_VOLUME];
uint8_t p1_buf_index = 0, t1_buf_index = 0;

// process_data temperature overheat
int16_t elec_t_max;

// input adc signals
float p1_adc_f, t1_adc_f;
uint16_t p1_adc_n, t1_adc_n;

// pwm correction table
struct TempPoint out1_corr_table[20];

float out1_Umin, out1_Umax;
uint16_t out1_Nmin, out1_Nmax;
uint16_t out1_N;

uint16_t evaluation_period = 0;

//-----------------------------------------------------------------------------------------------------------------

extern uint32_t systick_counter;

//-----------------------------------------------------------------------------------------------------------------

//
// User data are stored in 2 kB of NVR memory sector.
// It consists of two copies of data pages 1 kB each (511 2-byte Modbus registers and last 2 bytes of page CRC-16).
//

// user data bytes
uint8_t user_data[1024];
uint8_t copy[2] = {0,0};

//-----------------------------------------------------------------------------------------------------------------

// general system state
extern uint16_t system_status, command_status;

extern unsigned char mb_discretes[];
extern uint32_t mb_baudrate;
extern uint8_t mb_address;

// convert float to int16 adc output
inline float adc_to_float(uint16_t value)
{
	return A_VDD * value / 0x1000;
}

//-----------------------------------------------------------------------------------------------------------------

// read user data page (located in NVR area)
int8_t read_user_data_page(int8_t page)
{
	uint16_t i;
	uint32_t data[2];
	uint32_t address = 0x0 + (page << 10);
	
	// total of 4 pages
	if (page > 3) return -1;
	
	// read sector of info memory 
	__disable_irq ();
	for (i = 0; i < 128; i++)
	{
		MFLASH_ReadData(address, data, MFLASH_Region_NVR);
		memcpy((uint8_t*)&user_data[8*i], (uint8_t*)&data, 8);	
		address += 0x08;
	}
	__enable_irq ();
	
	if (usMBCRC16(&user_data[0], 1024) != 0)	
		return -2;
	
	return page;
}

//-----------------------------------------------------------------------------------------------------------------

// check info page consistency
int32_t check_user_data_page(uint8_t page)
{
	uint16_t i;
	uint32_t data[2];
	uint32_t address = 0x0 + (page << 10);
	
	if (page > 3) return -1;
	
	// read Sector info memory 
	__disable_irq ();  ///////////////////////////
	for (i = 0; i < 128; i++)
	{
		MFLASH_ReadData(address, data, MFLASH_Region_NVR);
		if (memcmp((uint8_t*)&user_data[8*i], (uint8_t*)&data, 8) != 0) break;
		address += 0x08;
	}
	__enable_irq (); //////////////////////////////
	return (int32_t)(address & 0xFFFF);
}

//-----------------------------------------------------------------------------------------------------------------

// write info page
int16_t write_user_data_page(uint8_t page)
{
	uint16_t i;
	uint32_t data[2];
	uint32_t address = 0x0 + (page << 10);
		
	if (page > 3) return -1;	
	__disable_irq (); 
	MFLASH_ErasePage(address, MFLASH_Region_NVR);

	// CRC-16
	i = usMBCRC16(&user_data[0], 1022);
	user_data[1022] = i;
	user_data[1023] = i >> 8;

	for (i = 0; i < 128; i++)
	{
		memcpy((uint8_t*)&data[0], (uint8_t*)&user_data[8*i], 8);
		MFLASH_WriteData(address, data, MFLASH_Region_NVR);	
		address +=0x08;		
	}		
	__enable_irq (); 
	return page;
}

//-----------------------------------------------------------------------------------------------------------------

// init user data
uint8_t init_processor(void)
{
	uint8_t page;
	p1_buf_index = 0; p1_buffer_full = 0;
	t1_buf_index = 0; t1_buffer_full = 0;
	
	for (page = 0; page < 2; page++)
	{
		if (read_user_data_page(page) == page)
			break;
	}
 	
	if (page == 0)
	{
		copy[0] = 1;
		if (check_user_data_page(1) == 0x800)	// comparison failed
			copy[1] = 1;
		else
		{
			write_user_data_page(1);
			if (check_user_data_page(1) == 0x800)	// comparison failed
				copy[1] =1;
		}
	}
	
	if (page == 1)
	{
		copy[1] = 1;
		if (check_user_data_page(0) == 0x400)	// comparison failed
			copy[0] = 1;
		else
		{
			write_user_data_page(0);
			if (check_user_data_page(0) == 0x400)	// comparison failed
				copy[0] = 1;
		}
	}	
	
	// correct sample of user data obtained
	if (page < 2)
	{
		// initialize polynom factors and OUT1 correction factors
		memcpy((uint8_t*)polynom_factors, (uint8_t*)&user_data[A1_0_0_L*2], (A1_3_9_H - A1_0_0_L + 1)*2);
		memcpy((uint8_t*)&out1_corr_table, (uint8_t*)&user_data[2*OUT1_REF1], 120);

		// baudrate and address
		mb_baudrate = 100UL * (((uint16_t)user_data[2*BAUDRATE + 1] << 8) + user_data[2*BAUDRATE]);
		mb_address = ((uint16_t)user_data[2*MB_ADDR + 1] << 8) + user_data[2*MB_ADDR];

		// initialize other parameters
		iteration_cap = ((uint16_t)user_data[2*MAX_ITERATIONS + 1] << 8) + user_data[2*MAX_ITERATIONS];
		if (iteration_cap < 5) iteration_cap = ITERATION_CAP;
		
		t1_sample_volume = ((uint16_t)user_data[2*T1_SAMPLE_VOLUME + 1] << 8) + user_data[2*T1_SAMPLE_VOLUME];
		if (t1_sample_volume < 1 || t1_sample_volume > MAX_T1_BUF_VOLUME) t1_sample_volume = DEFAULT_T1_BUF_VOLUME;
		p1_sample_volume = ((uint16_t)user_data[2*P1_SAMPLE_VOLUME + 1] << 8) + user_data[2*P1_SAMPLE_VOLUME];
		if (p1_sample_volume < 1 || p1_sample_volume > MAX_P1_BUF_VOLUME) p1_sample_volume = DEFAULT_P1_BUF_VOLUME;
		
		memcpy((uint8_t*)&p_out1_0, (uint8_t*)&user_data[2*POUT1_0_L], 4);
		p_out1_bak = p_out1_0;	

		memcpy((uint8_t*)&err_out1, (uint8_t*)&user_data[2*ERR_POUT1_L], 4);

		memcpy((uint8_t*)&p_out1_max, (uint8_t*)&user_data[2*POUT1_MAX_L], 4);
		memcpy((uint8_t*)&p_out1_min, (uint8_t*)&user_data[2*POUT1_MIN_L], 4);				

		memcpy((uint8_t*)&t1_sub_value, (uint8_t*)&user_data[2*SUB_T1_L], 4);
		memcpy((uint8_t*)&dt1_corr_max, (uint8_t*)&user_data[2*DSUB_T1_MAX_L], 4);
		memcpy((uint8_t*)&dt1_corr_min, (uint8_t*)&user_data[2*DSUB_T1_MIN_L], 4);

		memcpy((uint8_t*)&p1_adc_max, (uint8_t*)&user_data[2*P1_MAX_L], 4);
		memcpy((uint8_t*)&p1_adc_min, (uint8_t*)&user_data[2*P1_MIN_L], 4);
		memcpy((uint8_t*)&t1_adc_max, (uint8_t*)&user_data[2*T1_MAX_L], 4);
		memcpy((uint8_t*)&t1_adc_min, (uint8_t*)&user_data[2*T1_MIN_L], 4);
      
		memcpy((uint8_t*)&elec_t_max, (uint8_t*)&user_data[2*ELEC_T_MAX], 2);
      
		memcpy((uint8_t*)&out1_Umin, (uint8_t*)&user_data[2*UOUT1_MIN_L], 4); 
		memcpy((uint8_t*)&out1_Umax, (uint8_t*)&user_data[2*UOUT1_MAX_L], 4); 

		memcpy((uint8_t*)&dt1_corr_min, (uint8_t*)&user_data[2*DSUB_T1_MIN_L], 4); 
		memcpy((uint8_t*)&dt1_corr_max, (uint8_t*)&user_data[2*DSUB_T1_MAX_L], 4);
            
		memcpy((uint8_t*)&mb_holding_regs[0], (uint8_t*)&user_data[0], 2*END_ADDR);
	}
	return page;	
}

//-----------------------------------------------------------------------------------------------------------------

int16_t update_user_data (uint8_t page)
{
	memcpy((uint8_t*)user_data, (uint8_t*)mb_holding_regs, (END_ADDR)*2);
	return  write_user_data_page(page);	
}

//-----------------------------------------------------------------------------------------------------------------

float polynom_value(uint8_t pf_index, float arg)
{
	uint8_t idx, idx_start = pf_index*(T1_GRADE + 1), idx_end = idx_start + T1_GRADE + 1;
	float val = 0;
	for (idx = idx_start; idx < idx_end; idx++)
	{
		val += val * arg + polynom_factors[idx];
	}
	return val;
}

//-----------------------------------------------------------------------------------------------------------------

void process_data (void)
{
	float t1_arg;
	uint8_t idx;
	uint8_t factors_correct = 0;
	
	evaluation_period = systick_counter;
	
	// process input p1 data
	if (p1_buffer_full)
	{
		p1_adc_n = p1_sliding_sum / p1_sample_volume;
		p1_adc_f = (A_VDD * p1_sliding_sum) / (0xFFF * p1_sample_volume);
	}
	else if (p1_buf_index > 0)
	{
		p1_adc_n = p1_sliding_sum / p1_buf_index;
		p1_adc_f = (A_VDD * p1_sliding_sum) / (0xFFF * p1_buf_index);
	}
	
	mb_input_regs[ADC_CH1] = p1_adc_n;
	memcpy((uint8_t*)&mb_input_regs[P1_L], (uint8_t*)&p1_adc_f, 4); 
	
	// check ranges of p1
	if (p1_adc_f > p1_adc_max || p1_adc_f < p1_adc_min)
	{
		system_status &= ~P1_OK;
		mb_discretes[0] |= P1_OK;
	}
	else
	{
		system_status |= P1_OK;
		mb_discretes[0] &= ~P1_OK;
	}

	
	// process input t1 data
	if (t1_buffer_full)
	{
		t1_adc_n = t1_sliding_sum / t1_sample_volume;
		t1_adc_f = (A_VDD * t1_sliding_sum) / (0xFFF * t1_sample_volume);
	}
	else if (t1_buf_index > 0)
	{
		t1_adc_n = t1_sliding_sum / t1_buf_index;
		t1_adc_f = (A_VDD * t1_sliding_sum) / (0xFFF * t1_buf_index);
	}
	
	mb_input_regs[ADC_CH2] = t1_adc_n;
	mb_input_regs[ELEC_T] = t1_adc_n;
	memcpy((uint8_t*)&mb_input_regs[T1_L], (uint8_t*)&t1_adc_f, 4); 

	// check ranges of t1
	if (t1_adc_f > t1_adc_max || t1_adc_f < t1_adc_min)
	{
		system_status &= ~T1_OK;
		mb_discretes[0] |= T1_OK;
	}
	else
	{
		system_status |= T1_OK;
		mb_discretes[0] &= ~T1_OK;
	}

	// evaluate factors
	if ( system_status & T1_OK && !(command_status & CALC_DISABLED) )
	{
		t1_arg = t1_adc_f - t1_sub_value;    
		t1_arg += dt1_corr_min + (p_out1 / p_out1_max * (dt1_corr_max - dt1_corr_min));

		factors_correct = 0;
		// evaluate current polynom factors
		for (idx = 0; idx < P1_GRADE + 1; idx++)
		{
			polynom_p1_factors[idx] = polynom_value(idx, t1_arg);
			if (polynom_p1_factors[idx] != 0 && idx > 0)
				factors_correct = 1;
		}
	
		// check factors
		if (factors_correct == 0)
			mb_discretes[1] |= P1_SETTINGS_FAULT;
		else mb_discretes[1] &= ~P1_SETTINGS_FAULT;
	}

	// check sub temperature value
	if (t1_sub_value == 0) 
		mb_discretes[1] |= T1_SETTINGS_FAULT;		
	else mb_discretes[1] &= ~T1_SETTINGS_FAULT;

	if ( (system_status & P1_OK) && factors_correct && !(command_status & CALC_DISABLED) )
	{ 
		if (++mb_input_regs[ATT_POUT1_L] == 0) mb_input_regs[ATT_POUT1_H]++;
		p_out1 = solve_polynom(p1_adc_f, p_out1_bak);
				
		__disable_irq ();			
		if (p_out1 > -4.0F)
		{
			memcpy((uint8_t*)&mb_input_regs[P1_L], (uint8_t*)&p_out1, 4);
			p_out1_bak = p_out1;
		}
		else
		{
			if (++mb_input_regs[FAIL_POUT1_L]==0)  mb_input_regs[FAIL_POUT1_H]++;
			p_out1_bak = p_out1_0;
		}
		__enable_irq ();				
	}
	
	// algorithm time (ms)
	mb_input_regs[OUTPUT_INTERVAL_MS] = systick_counter - evaluation_period;
}

//-----------------------------------------------------------------------------------------------------------------

// polynom evaluation
float evaluate_p_polynom(float arg)
{
	int i;
	float f_value = 0;
	for (i = 0; i < P1_GRADE + 1; i++)
		f_value += f_value * arg + polynom_p1_factors[i];
	return f_value;
}

//-----------------------------------------------------------------------------------------------------------------

// polynom derivation
float evaluate_derivative(float arg)
{
	int i;
	float f_value = 0;
	for (i = 1; i < P1_GRADE + 1; i++)
		f_value += i * f_value * arg + polynom_p1_factors[i];
	return f_value;
}

//-----------------------------------------------------------------------------------------------------------------

// Newton's method
float solve_polynom(float y0, float x0) 
{
	unsigned char i;
	float p1_i;
	float p1_i_1 = x0;	
	float a1_0_temp = polynom_p1_factors[0];
	polynom_p1_factors[0] -= y0;
		
	for (i = 0; i < iteration_cap; i++)
	{
		p1_i = p1_i_1 - (evaluate_p_polynom(p1_i_1) / evaluate_derivative(p1_i_1));
		if ((p1_i - p1_i_1) > 0)
		{
			if ((p1_i - p1_i_1) < err_out1) break;
		}
		else
		{
			if ((p1_i_1 - p1_i) < err_out1) break;
		}			
		p1_i_1 = p1_i;
	}	

	polynom_p1_factors[0] = a1_0_temp;
	if (i < iteration_cap) return p1_i;
	else return -5.0F;
}

//-----------------------------------------------------------------------------------------------------------------

// setup output channel
uint16_t set_output_range(void)
{
	float n_relation = 0;
	uint8_t i;
	
	// check data
	if((out1_corr_table[0].t_ref == 0) || (out1_corr_table[0].n_min == 0) || (out1_corr_table[0].n_max == 0) || (out1_corr_table[0].n_max < out1_corr_table[0].n_min)) return 0;
		
	// select range
	if (t1_adc_n <= out1_corr_table[0].t_ref)
	{
		out1_Nmin = out1_corr_table[0].n_min;
		out1_Nmax = out1_corr_table[0].n_max;
		return 1;
	}

	// intermediate
	for(i = 0; i < 19; i++)
	{
		// reached the end of table
		if (out1_corr_table[i+1].t_ref == 0)
		{
			out1_Nmin = out1_corr_table[i].n_min;
			out1_Nmax = out1_corr_table[i].n_max;
			return 1;
		}
		else if (t1_adc_n <= out1_corr_table[i+1].t_ref)		// intermediate point
		{
			n_relation = ((float)t1_adc_n - out1_corr_table[i].t_ref) / (out1_corr_table[i+1].t_ref - out1_corr_table[i].t_ref);
			out1_Nmin = out1_corr_table[i].n_min + (out1_corr_table[i+1].n_min - out1_corr_table[i].n_min) * n_relation;
			out1_Nmin = out1_corr_table[i].n_max + (out1_corr_table[i+1].n_max - out1_corr_table[i].n_max) * n_relation;
			return 1;
		}
	}

	return 0;
}
