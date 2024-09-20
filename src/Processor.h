#ifndef _PROCESSOR_H
#define _PROCESSOR_H

#include "stdint.h"

float evaluate_p_polynom (float);
float evaluate_derivative (float);
float solve_polynom (float, float);

uint16_t float_to_uint16adc(float);

int8_t read_user_data_page(int8_t);
int32_t check_user_data_page(uint8_t);
int16_t write_user_data_page(uint8_t);
int16_t update_user_data(uint8_t);

uint8_t init_processor(void);
void process_data(void);
uint16_t set_output_range(void);

struct TempPoint
{
	int16_t  t_ref;
	uint16_t n_min;
	uint16_t n_max;	
};

#endif // _PROCESSOR_H
