#ifndef _COMDEF_H
#define _COMDEF_H

#include <stdint.h>
#define IWDG									/*   Watch Dog  */	

#define DEFAULT_SYSCLK_FREQ				(8000000UL)	// 8MHz
#define DEFAULT_ADC_CLOCK				(1000000UL) // 1MHz
#define DEFAULT_MB_ADDRESS				(1)
#define DEFAULT_MB_BAUDRATE				(57600)

#define DEFAULT_ADC_P1_INTERVAL			(1) 		// ms
#define DEFAULT_ADC_T1_INTERVAL			(100) 		// ms

#define DEFAULT_T1_SAMPLE_VOLUME 		(4)
#define DEFAULT_P1_SAMPLE_VOLUME 		(4)

#define DEFAULT_P1_BUF_VOLUME			(16)
#define DEFAULT_T1_BUF_VOLUME			(16)
#define MAX_P1_BUF_VOLUME				(64)
#define MAX_T1_BUF_VOLUME				(64)

#define PROC_T_SAMPLE_VOLUME			(10)

#define PWM_PERIOD						(0xFFF)

// out1 evaluation ranges
#define MIN_OUT1_RANGE					(1.0F)
#define MAX_OUT1_FACTOR					(2.0F)	// !!! it is a multiplier

// output error levels
#define ERR_SETTINGS_VOLT				(0.05F)					
#define ERR_SIGNALS_VOLT				(0.05F)					

// modbus input registers
enum regInputBufAddr
{
	P1_L,
	P1_H,
	T1_L,
	T1_H,
	//RESERVED 4-18
	ELEC_T = 19,
	// RESERVED 20
	POUT1_L = 21,
	POUT1_H,
	//RESERVED 23-40
	ATT_POUT1_L = 41,
	ATT_POUT1_H,
	//RESERVED 43-60
	FAIL_POUT1_L = 61,
	FAIL_POUT1_H,		
	//RESERVED 63-80
	ADC_CH1 = 81,
	ADC_CH2,
	//RESERVED 83-100	
	OUT1_N = 101,
	//RESERVED 102-105
	FW_ID_L = 106,
	FW_ID_H,
	ID_ADDRESS_OUT_L = 119,
	ID_ADDRESS_OUT_H,
	OUTPUT_INTERVAL_MS = 121,
	// RESERVED 122
	CPU_ID_L = 123,
	CPU_ID_H,
	CHIP_ID_L = 125,
	CHIP_ID_H,
	RESET_COUNT = 127
};

// modbus holding registers
enum regHoldingBufAddr
{
	TYPEID,  //0
	SN = 10,
	SN_EXT = 12,
	BAUDRATE = 15, //15
	MB_ADDR = 16,
	SENSOR_ID_L = 17,
	SENSOR_ID_H,
	HARDWARE_ID = 19,
	// RESERVED 20-29
	POUT1_MAX_L = 30,
	POUT1_MAX_H,
	POUT1_MIN_L,
	POUT1_MIN_H,
	// RESERVED 34-49
	P1_SAMPLE_VOLUME = 50,
	T1_SAMPLE_VOLUME,
	// RESERVED 52-59
	POUT1_0_L = 60,
	POUT1_0_H,
	// RESERVED 62-79
	SUB_T1_L = 80,
	SUB_T1_H,
	DSUB_T1_MAX_L = 82,
	DSUB_T1_MAX_H,
	DSUB_T1_MIN_L = 84,
	DSUB_T1_MIN_H,
	// RESERVED 86-89
	MAX_ITERATIONS = 90,
	ERR_POUT1_L = 91,
	ERR_POUT1_H,
	// RESERVED 93-111
	P1_MIN_L = 111,
	P1_MIN_H,	
	P1_MAX_L,
	P1_MAX_H,	
	T1_MIN_L,
	T1_MIN_H,	
	T1_MAX_L,
	T1_MAX_H, // 118
	// RESERVED 119-130
	ELEC_T_MAX = 131,
	// RESERVED 131-140
	DATA_OUT1 = 141,
	// RESERVED 141-150
	A1_0_0_L = 151,
	A1_0_0_H,
	A1_3_9_L = 229,
	A1_3_9_H,
	OUT1_REF1 = 231,
	OUT1_N1_MIN,
	OUT1_N1_MAX,
	//...
	OUT1_N20_MAX = 290,
	//
	UOUT1_MIN_L = 291,
	UOUT1_MIN_H,
	UOUT1_MAX_L,
	UOUT1_MAX_H,
	// RESERVED 295-310
	ID_ADDRESS_IN_L = 311,
	ID_ADDRESS_IN_H,
	ID_MB_ADDRESS_IN,
	// RESERVED 314-320
	
	//
	// LOADER_SERVICE_BLOCK
	//
	FW_UPDATE_PASSWORD_IN = 321,	// here user loads password for FW update
	FW_UPDATE_PAGE_IN,
	FW_UPDATE_PAGEBUF_IN,
	FW_UPDATE_BUFFER_START = 341,
	FW_UPDATE_BUFFER_END = 404,
	// RESERVED 405-511
	END_ADDR = 511
};

// system_status
#define P1_OK					0x1
#define T1_OK					0x2
#define P1_SETTINGS_FAULT		0x4
#define T1_SETTINGS_FAULT		0x8
#define STATE_MB_POLL			0x20	// modbus polls

#define FW_UPDATE_PASSWORD_OK	0x8000	// fw update mode ON
#define FW_UPDATE_PAGE_ERASED	0x0100	// updater flash page erased
#define FW_UPDATE_BUF_WRITTEN	0x0200	// updater buffer written to flash page
#define FW_UPDATE_BUF_READ		0x0400	// updater buffer read from flash page

// command_status
#define REQUEST_EEPROM_WRITE  	0x1
#define CALC_DISABLED     		0x2
#define ENABLE_ANALOG_OUTPUT   	0x4
#define ADDRESS_SELECT_PENDING 	0x20
#define EXECUTE_EEPROM_WRITE  	0x40

// FW update command flags
#define FW_UPDATE_ERASE_PAGE	0x0100	// coil 8
#define FW_UPDATE_WRITE_BUF		0x0200	// coil 9
#define FW_UPDATE_READ_BUF		0x0400	// coil 10
#define FW_UPDATE_WRITE_NEW		0x0800	// coil 11
#define FW_UPDATE_MODE			0x8000  // coil 15

#define FW_UPDATE_POS			(8)
#define FW_UPDATE_IDX			(1)

#endif // _COMDEF_H
