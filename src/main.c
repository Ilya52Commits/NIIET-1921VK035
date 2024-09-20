//-- Includes ------------------------------------------------------------------
#include "..\perph\plib035.h"
#include "..\modbus\mb.h"
#include "..\modbus\mbport.h"
#include "fw_updater.h"
#include "ComDef.h"
#include "Processor.h"
#include <string.h>
#include <stdlib.h>

// -- Variables ----------------------------------------------------------------

/* Определение переменных для хранения скорости передачи данных по Modbus и адреса устройства. 
Устанавливаются значения по умолчанию: скорость 57600 бод и адрес 0x01 */
uint32_t mb_baudrate = 57600; 	// default value
uint8_t  mb_address = 0x01;		// default	value
// -----------------------------------------------------------------------------

/* Переменные для системного таймера (systick_counter), 
счётчиков, использующихся при загрузке данных во флэш-память, и времени расчёта. */
uint32_t systick_counter, flash_load_counter1, flash_load_counter2;
uint32_t calculation_time = 0;
// -----------------------------------------------------------------------------

// system control states | состояния управления системой

/* Статусные переменные для системы и команд, а также переменная для хранения статуса Modbus (тип eMBErrorCode). */
uint16_t system_status = 0;
uint16_t command_status = 0;
eMBErrorCode mb_status;
// -----------------------------------------------------------------------------

// -- Extern variables ---------------------------------------------------------
/* Объявление внешних массивов, используемых для хранения регистров Modbus: 
удерживающих, входных регистров, дискретных входов и выходов. */
extern uint16_t mb_holding_regs[];
extern uint16_t	mb_input_regs[];
extern unsigned char mb_discretes[];
extern unsigned char mb_coils[];
// -----------------------------------------------------------------------------

/* Объявление внешних переменных, хранящих копии данных и пользовательские данные. */
extern uint8_t copy[2];
extern uint8_t user_data[];
// -----------------------------------------------------------------------------

/* Внешние переменные для параметров аналоговых входов и выходов, а также промежуточных вычислений. */
extern uint8_t p1_sample_volume;
extern uint8_t t1_sample_volume;
extern uint16_t out1_N, out1_Nmin, out1_Nmax;
extern float out1_Umin, out1_Umax, tmp;
extern float p_out1_max, p_out1_min;
// -----------------------------------------------------------------------------

//-- Peripheral init function prototypes ---------------------------------------
/* Прототипы функций, используемых для инициализации и выполнения основных операций */ 
void systick_init(void);
void adc_init(void);
void pwm_init(void);
//void timer_init(void);
void perform_auto_addressing(void);
void process_analog_output(void);
// -----------------------------------------------------------------------------

//-- Main ----------------------------------------------------------------------
__attribute__ ((noreturn)) int main(void)
{	
	int i;
	
	/* Настройка системной тактовой частоты микроконтроллера с 
	помощью команд переключения системного тактового сигнала на тактирование от PLL (Phase-Locked Loop). */
	RCU_SysClkChangeCmd(RCU_SysClk_PLLClk);
	RCU_PLL_AutoConfig(DEFAULT_SYSCLK_FREQ, RCU_PLL_Ref_OSEClk);
	SystemCoreClockUpdate();
	// ---------------------------------------------------------------------------
	
	/* Включение тактирования для порта ввода-вывода GPIOA и его сброс. */
	RCU_AHBClkCmd(RCU_AHBClk_GPIOA, ENABLE);
	RCU_AHBRstCmd(RCU_AHBRst_GPIOA, ENABLE);
	// ---------------------------------------------------------------------------

	// init from user data
	/* Инициализация процессора дважды и выход из цикла,
	если условие по массиву copy выполняется (например, проверка состояния загрузки данных) */
	for (i = 0; i < 2; i++)
	{
		init_processor();
		if (copy[0] & copy[1]) break;
	}
	// ---------------------------------------------------------------------------
	
	//****** FIRMWARE VER. *******
	mb_input_regs[FW_ID_L] = 0x0001;
	mb_input_regs[FW_ID_H] = 0x0100;
	
	// baudrate and address
	if (mb_baudrate != 9600 && mb_baudrate != 57600 && mb_baudrate != 115200)
		mb_baudrate = DEFAULT_MB_BAUDRATE;
	
	if (mb_address < 1 || mb_address > 247) 
		mb_address = DEFAULT_MB_ADDRESS;
	
	mb_holding_regs[MB_ADDR] = mb_address;
	mb_holding_regs[BAUDRATE] = mb_baudrate / 100;
	
	// init modbus stack
	eMBInit(MB_RTU, mb_address, 0, mb_baudrate, MB_PAR_NONE); 
	eMBEnable();	

	// init peripherals
	systick_init();
	adc_init();
	pwm_init();
	
	// CPU and chip IDs
	uint32_t chip_id = SIU->CHIPID;
	memcpy((uint8_t*)&mb_input_regs[CHIP_ID_L], (uint8_t*)&chip_id, 4);
	
	chip_id = SCB->CPUID;
	memcpy((uint8_t*)&mb_input_regs[CPU_ID_L], (uint8_t*)&chip_id, 4);
		
  while (1)
	{
		// processing modbus on tick | обработка modbus по тику
		if (system_status & STATE_MB_POLL)
		{
			// сброс флага
			system_status &= ~STATE_MB_POLL;
			// опрос modbus
			eMBPoll();
		}
		
		
		// FW update requested | запрошено обновление FW
		if (command_status & FW_UPDATE_MODE)
		{
			// check FW update password failed | проверка пароля обновнения FW не удалась
			if (mb_holding_regs[FW_UPDATE_PASSWORD_IN] != FW_UPDATE_PASSWORD)
			{
				mb_holding_regs[FW_UPDATE_PASSWORD_IN] = 0;
				// drop FW update coil | сброс FW обновление катушки
				mb_coils[FW_UPDATE_IDX] &= ~(FW_UPDATE_MODE >> FW_UPDATE_POS);
				// drop FW update command | сброс команды обновления FW 
				command_status &= ~FW_UPDATE_MODE;
				// drop FW password accepted state | сброс состояния принятия пароля FW
				mb_discretes[FW_UPDATE_IDX] &= ~(FW_UPDATE_PASSWORD_OK >> FW_UPDATE_POS);
			}
			else // password is OK | пароль впорядке 
			{
				mb_discretes[FW_UPDATE_IDX] |= FW_UPDATE_PASSWORD_OK >> FW_UPDATE_POS;
				
				// run updater mode | запустить режим обновления
				perform_fw_update();
			}
		}
		
		// update user data from modbus | обновление пльзовательских данных из modbus
		if (command_status & EXECUTE_EEPROM_WRITE)
		{
			// флаги для контроля временем выполнения процесса
			flash_load_counter2 = 0; 
			flash_load_counter1 = systick_counter; // присвоение системного времени
			
			while (flash_load_counter2 < 1000)
			{		
				// обновление пользовательских данных 
				update_user_data(0);			
				
				if (check_user_data_page(0) == 0x400)
				{
					while (flash_load_counter2 < 1000)
					{					
						update_user_data(1);	
						
						if (check_user_data_page(1) == 0x800)
						{
							NVIC_SystemReset();
							__disable_irq();
							
							while(1);
						}
						else	
						{
							while(systick_counter < (flash_load_counter1 + 100));	
							
							flash_load_counter2 += 100;
							flash_load_counter1 = systick_counter;
						}
					}			
				}
				else
				{
					while(systick_counter < (flash_load_counter1+100));	
					
					flash_load_counter2 += 100; 
					flash_load_counter1 = systick_counter;
				}				
			}	
			NVIC_SystemReset(); // функция сброса
			__disable_irq(); // глобальный запрет прерываний
			
			while(1);			
		}
		
		// auto adressing | авто-адресация
		if (command_status & ADDRESS_SELECT_PENDING)
			perform_auto_addressing();
		
		// main processor of data | главный процессор данных 
		process_data();

		// create output | создание вывода
		process_analog_output();
  }
}

//
//-- Init functions | Инициализация функций -----------------------------------------------------------------------
//

void systick_init(void)
{
	NVIC_SetPriority(SysTick_IRQn, 2);

	SysTick->VAL = 0;
	SysTick->LOAD = (SystemCoreClock / 1000) - 1; // 1 ms for tick
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

//-----------------------------------------------------------------------------------------------------------------

void timer_init()
{
	RCU_APBClkCmd(RCU_APBClk_TMR1, ENABLE);
	RCU_APBRstCmd(RCU_APBRst_TMR1, ENABLE);
	TMR_PeriodConfig(TMR1, SystemCoreClock, 1000);
	TMR_ITCmd(TMR1, ENABLE);
	TMR_ExtInputConfig(TMR1, TMR_ExtInput_Disable);
	TMR_Cmd(TMR1, ENABLE);
	NVIC_EnableIRQ(TMR1_IRQn);
}

//-----------------------------------------------------------------------------------------------------------------

void adc_init(void)
{
  ADC_SEQ_Init_TypeDef ADC_SEQ_InitStruct;

	// DIVN = 2*(N+1)
	if (DEFAULT_SYSCLK_FREQ / DEFAULT_ADC_CLOCK < 1)
		RCU_ADCClkConfig(RCU_PeriphClk_PLLDivClk, (uint32_t)0, DISABLE); // as is 
	else
		RCU_ADCClkConfig(RCU_PeriphClk_PLLDivClk, DEFAULT_SYSCLK_FREQ / (DEFAULT_ADC_CLOCK << 1) - 1, ENABLE); // ~1 MHz


  RCU_ADCClkCmd(ENABLE);
  RCU_ADCRstCmd(ENABLE);
	
	// obtain delay timer intervals | получение интервалом таймаера задержки
	uint32_t adc_clock = RCU_GetADCClkFreq();
	uint32_t p1_delay_count = adc_clock / 1000 * DEFAULT_ADC_P1_INTERVAL - 1;
	uint32_t t1_delay_count = p1_delay_count * DEFAULT_ADC_T1_INTERVAL / DEFAULT_ADC_P1_INTERVAL;

	// ADC on | Включение АЦП
  ADC_AM_Cmd(ENABLE);
	
	// setup sequencer 0 (p1 channesl) | настройка секвенсора 0 (p1 каналы)
  ADC_SEQ_StructInit(&ADC_SEQ_InitStruct);
  ADC_SEQ_InitStruct.StartEvent = ADC_SEQ_StartEvent_Cycle;
  ADC_SEQ_InitStruct.SWStartEn = ENABLE;
	ADC_SEQ_InitStruct.ReqMax = ADC_SEQ_ReqNum_0;				// single channel | один канал
	ADC_SEQ_InitStruct.Req[ADC_SEQ_ReqNum_0] = ADC_CH_Num_0;	// ADC channel 0 | канал АЦП 0 
	ADC_SEQ_InitStruct.ReqAverage = (ADC_SEQ_Average_TypeDef)DEFAULT_P1_SAMPLE_VOLUME;	// avg
  ADC_SEQ_Init(ADC_SEQ_Num_0, &ADC_SEQ_InitStruct);

	// setup sequencer 1 (t1 channel) | настройка секвенсора 1 (t1 каналы)
	ADC_SEQ_InitStruct.Req[ADC_SEQ_ReqNum_0] = ADC_CH_Num_1;
	ADC_SEQ_InitStruct.ReqAverage = (ADC_SEQ_Average_TypeDef)DEFAULT_T1_SAMPLE_VOLUME;	// avg
  ADC_SEQ_Init(ADC_SEQ_Num_1, &ADC_SEQ_InitStruct);
	
	// setup timers for sequencers update | настройка таймеров для обновления секвенсора
	ADC_SEQ_SetRestartTimer(ADC_SEQ_Num_0, p1_delay_count);				
	ADC_SEQ_SetRestartTimer(ADC_SEQ_Num_1, t1_delay_count);			
	
  ADC_SEQ_Cmd(ADC_SEQ_Num_0, ENABLE);
  ADC_SEQ_Cmd(ADC_SEQ_Num_1, ENABLE);

  // refresh data after every sequencer output update | обновлять данные после каждого обновления секвенсора
  ADC_SEQ_ITConfig(ADC_SEQ_Num_0, 0, DISABLE);
  ADC_SEQ_ITConfig(ADC_SEQ_Num_1, 0, DISABLE);
  ADC_SEQ_ITCmd(ADC_SEQ_Num_0, ENABLE);
  ADC_SEQ_ITCmd(ADC_SEQ_Num_1, ENABLE);

	NVIC_EnableIRQ(ADC_SEQ0_IRQn);
  NVIC_EnableIRQ(ADC_SEQ1_IRQn);

  while (!ADC_AM_ReadyStatus()) {}

	ADC_SEQ_SwStartCmd(); // ADC start | запуск АЦП
}

//-----------------------------------------------------------------------------------------------------------------

void pwm_init(void)
{
  PWM_TB_Init_TypeDef PWM_TB_InitStruct;
  PWM_CMP_Init_TypeDef PWM_CMP_InitStruct;
  PWM_AQ_Init_TypeDef PWM_AQ_InitStruct;
  GPIO_Init_TypeDef GPIO_InitStruct;

  // PWM1 clock on
  RCU_APBClkCmd(RCU_APBClk_PWM1, ENABLE);
  RCU_APBRstCmd(RCU_APBRst_PWM1, ENABLE);

  // setup timer of PWM
  PWM_TB_StructInit(&PWM_TB_InitStruct);
  PWM_TB_InitStruct.Mode = PWM_TB_Mode_Up;
  // no prescaling (8 MHz)
  PWM_TB_InitStruct.ClkDiv = PWM_TB_ClkDiv_1;
  PWM_TB_InitStruct.ClkDivExtra = PWM_TB_ClkDivExtra_1;
  PWM_TB_InitStruct.Period = PWM_PERIOD;
  PWM_TB_Init(PWM1, &PWM_TB_InitStruct);

  // setup comparator
  PWM_CMP_StructInit(&PWM_CMP_InitStruct);
  PWM_CMP_InitStruct.CmpB = 0; // off for now
  PWM_CMP_Init(PWM1, &PWM_CMP_InitStruct);

  // setup output compare logic
  PWM_AQ_StructInit(&PWM_AQ_InitStruct);
  PWM_AQ_InitStruct.ActionB_CTREqCMPBUp = PWM_AQ_Action_ToZero; // 0 on CMP
  PWM_AQ_InitStruct.ActionB_CTREqPeriod = PWM_AQ_Action_ToOne;  // 1 on PERIOD
  PWM_AQ_Init(PWM1, &PWM_AQ_InitStruct);

  // setup pins
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.AltFunc = ENABLE;
  GPIO_InitStruct.Pin = GPIO_Pin_11; //PWM1_B
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_DigitalCmd(GPIOA, GPIO_Pin_11, ENABLE);
	
	PWM_TB_PrescCmd(PWM_TB_Presc_1, ENABLE);
}

//-----------------------------------------------------------------------------------------------------------------

void perform_auto_addressing(void)
{
	command_status &= ~ADDRESS_SELECT_PENDING;

	eMBDisable();
	
	mb_address = 0x01;
	mb_status = eMBInit( MB_RTU, mb_address, 0, mb_baudrate, MB_PAR_NONE ); 
	mb_status = eMBEnable(  );	
						
	srand(SIU->CHIPID);
						
	while (mb_address == 0x01)
	{
//				#ifdef IWDG			
//				if (IWDG_GetFlagStatus(IWDG_FLAG_RVU) == 0)
//				{
//					IWDG_ReloadCounter();
//				}
//				#endif  
		uint16_t random_delay = rand() % 19; 
		uint32_t delay_to_moment = systick_counter + random_delay * 10;
		
		while(systick_counter < delay_to_moment); // random delay in ms
		eMBPoll();								  // process modbus
								
		// check after receiving id to ADDRESS_DISTRIB_IN
		if((mb_holding_regs[ID_ADDRESS_IN_L] == mb_input_regs[CHIP_ID_L]) && (mb_holding_regs[ID_ADDRESS_IN_H] == mb_input_regs[CHIP_ID_H])) // our id is obtained
		{
			eMBDisable();
			mb_address = mb_holding_regs[MB_ADDR] = mb_holding_regs[ID_MB_ADDRESS_IN];
			mb_status = eMBInit( MB_RTU, mb_address, 0, mb_baudrate, MB_PAR_NONE ); 
			mb_status = eMBEnable(  );
		}
	}
	mb_holding_regs[ID_MB_ADDRESS_IN] = 0;
	// update memory for address
	command_status |= EXECUTE_EEPROM_WRITE;
}

//-----------------------------------------------------------------------------------------------------------------

void process_analog_output(void)
{
	if ((command_status & ENABLE_ANALOG_OUTPUT) == 0)
	{
		if (set_output_range())
		{	
			if ((system_status & P1_OK) && (system_status & T1_OK))	
			{
				if (mb_discretes[1] & (P1_SETTINGS_FAULT | T1_SETTINGS_FAULT))	// settings missing
					out1_N = (int32_t)((out1_Nmax - out1_Nmin) / (out1_Umax - out1_Umin) * (ERR_SETTINGS_VOLT - out1_Umin) + out1_Nmin);
				else
				{
					// evaluated output
					memcpy((uint8_t*)&tmp, (uint8_t*)&mb_input_regs[P1_L], 4);	
					if (p_out1_max > p_out1_min)
					{
						if(tmp < (p_out1_min - MIN_OUT1_RANGE) || tmp > MAX_OUT1_FACTOR*p_out1_max) 
							out1_N = PWM_PERIOD;
						else
							out1_N = (int32_t)((out1_Nmax - out1_Nmin) / (out1_Umax - out1_Umin) * (tmp - out1_Umin) + out1_Nmin);
					}
					else // settings error
						out1_N = (int32_t)((out1_Nmax - out1_Nmin) / (out1_Umax - out1_Umin) * (ERR_SETTINGS_VOLT - out1_Umin) + out1_Nmin);
				}
										 
				calculation_time = systick_counter - calculation_time;
				mb_input_regs[OUTPUT_INTERVAL_MS] = calculation_time & PWM_PERIOD;
				calculation_time = systick_counter;
			}
			else // input signals error
				out1_N = (int32_t)((out1_Nmax - out1_Nmin) / (out1_Umax - out1_Umin) * (ERR_SIGNALS_VOLT - out1_Umin) + out1_Nmin);

		}
		else out1_N = 0;	// no output setup factors
	}
	else out1_N = mb_holding_regs[DATA_OUT1];	// режим управления ЦАП
				
	if (out1_N > PWM_PERIOD) 
		PWM_CMP_SetCmpB(PWM1, PWM_PERIOD);
	else 
		PWM_CMP_SetCmpB(PWM1, out1_N & PWM_PERIOD);	
	
	mb_input_regs[OUT1_N] =	out1_N & PWM_PERIOD;
}
	
//-- Assert --------------------------------------------------------------------
#if defined USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    printf("Assert failed: file %s on line %d\n", file, (int)line);
    while (1) {
    };
}
#endif /* USE_FULL_ASSERT */
