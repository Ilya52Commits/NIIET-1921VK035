#ifndef _MBDEFS_H
#define _MBDEFS_H

#include <stdio.h>
#include <ctype.h>
#include "mb.h"
#include "port.h"
#include "stdint.h"

/* ----------------------- Defines ------------------------------------------*/

#define MODBUS_WRONG_PERIOD 4

/* Main register area */
#define REG_INPUT_START 1 //1200
#define REG_INPUT_NREGS	128//50//44// 40//32 //250

extern USHORT   mb_input_regs_start;
extern USHORT   mb_input_regs[REG_INPUT_NREGS];

/* Calibration data */
#define REG_INPUT_START_CBL 1 //1500
#define REG_INPUT_NREGS_CBL 1 //100

extern USHORT   usRegInputStart_CBL;
extern USHORT   usRegInputBuf_CBL[REG_INPUT_NREGS_CBL];


#define REG_HOLDING_START 1 //1000
#define REG_HOLDING_NREGS 512

extern uint16_t   mb_holding_regs_start;
extern uint16_t   mb_holding_regs[REG_HOLDING_NREGS];

#define REG_COILS_START     1 //1100
#define REG_COILS_SIZE      16 //32

extern uint16_t   mb_coils_start;
extern unsigned char mb_coils[REG_COILS_SIZE / 8];

#define REG_DISCRETES_START     1 //1300
#define REG_DISCRETES_SIZE      24 //32

extern uint16_t   mb_discretes_start;
extern unsigned char mb_discretes[REG_DISCRETES_SIZE / 8];

eMBException
mbf100( UCHAR * pucFrame, USHORT * pusLength ); // Change slave ID

eMBException
mbf101( UCHAR * pucFrame, USHORT * pusLength ); 

eMBException
mbf102( UCHAR * pucFrame, USHORT * pusLength );

eMBException
mbf103( UCHAR * pucFrame, USHORT * pusLength );

eMBException
mbf104( UCHAR * pucFrame, USHORT * pusLength ); // Change BaudRate and Parity control
// 0 for 9600, 1 for 14400, 2 for 19200, 3 for 38400, 4 for 57600, 5 for 115200

eMBException
mbf105( UCHAR * pucFrame, USHORT * pusLength ); // Enqueue command with parameter

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );

eMBErrorCode
 eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                           eMBRegisterMode eMode );

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode );

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete );

void mbResetToF0(void);

#endif
