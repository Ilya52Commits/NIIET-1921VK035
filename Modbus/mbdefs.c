#include <string.h>
#include "mbdefs.h"
#include "mbframe.h"
#include "ComDef.h"

#include "plib035_mflash.h"

#define MB_PDU_FUNC_WRITE_VALUE_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE                  ( 4 )

extern	uint16_t command_status;
//extern uint16_t PGA[6];
USHORT   			mb_input_regs_start = REG_INPUT_START;
USHORT			mb_input_regs[REG_INPUT_NREGS];
USHORT   			usRegInputStart_CBL = REG_INPUT_START_CBL;
USHORT   			usRegInputBuf_CBL[REG_INPUT_NREGS_CBL];
uint16_t   		mb_holding_regs_start = REG_HOLDING_START;
uint16_t   mb_holding_regs[REG_HOLDING_NREGS];
uint16_t   		mb_coils_start = REG_COILS_START;
unsigned char mb_coils[REG_COILS_SIZE / 8];
uint16_t   		mb_discretes_start = REG_DISCRETES_START;
unsigned char mb_discretes[REG_DISCRETES_SIZE / 8];

 
void xMBUtilSetBits( UCHAR *, USHORT, UCHAR, UCHAR );
UCHAR xMBUtilGetBits( UCHAR *, USHORT , UCHAR );



eMBErrorCode
eMBRegInputCB_( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
	
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
      iRegIndex = ( int )( usAddress - mb_input_regs_start );
      while( usNRegs > 0 )
      {
        *pucRegBuffer++ = ( unsigned char )( mb_input_regs[iRegIndex] >> 8 );
        *pucRegBuffer++ = ( unsigned char )( mb_input_regs[iRegIndex] & 0xFF );
        iRegIndex++;
        usNRegs--;
      }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
	USHORT   usStart;
	USHORT*  usBuf;
    int iRegIndex;
	
    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
	{
		usStart = mb_input_regs_start;
		usBuf = (USHORT*)&mb_input_regs;
	}
	else if( ( usAddress >= REG_INPUT_START_CBL ) && ( usAddress + usNRegs <= REG_INPUT_START_CBL + REG_INPUT_NREGS_CBL ) )
    {
		usStart = usRegInputStart_CBL;
		usBuf = (USHORT*)&usRegInputBuf_CBL;
	}
	else
    {
		eStatus = MB_ENOREG;
		return eStatus;
    }
		
	iRegIndex = ( int )( usAddress - usStart );
	while( usNRegs > 0 )
	{
		*pucRegBuffer++ = ( unsigned char )( usBuf[iRegIndex] >> 8 );
		*pucRegBuffer++ = ( unsigned char )( usBuf[iRegIndex] & 0xFF );
		iRegIndex++;
		usNRegs--;
	}
    return eStatus;
}


eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int          		iRegIndex;

	if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
	{
		iRegIndex = ( int )( usAddress - mb_holding_regs_start );
		switch ( eMode )
		{
			/* Pass current register values to the protocol stack. */
			case MB_REG_READ:
				 while( usNRegs > 0 )
				 {
					 *pucRegBuffer++ = ( UCHAR ) ( mb_holding_regs[iRegIndex] >> 8 );
					 *pucRegBuffer++ = ( UCHAR ) ( mb_holding_regs[iRegIndex] & 0xFF );
					 iRegIndex++;
					 usNRegs--;
				 }
				 break;

			/* Update current register values with new values from the protocol stack. */
			case MB_REG_WRITE:
				 while( usNRegs > 0 )
				 {
					 mb_holding_regs[iRegIndex] = *pucRegBuffer++ << 8;
					 mb_holding_regs[iRegIndex] |= *pucRegBuffer++;
					 iRegIndex++;
					 usNRegs--;
				 }
				 break;
	    }
	}
	else
	{
	    eStatus = MB_ENOREG;
	}
	return eStatus;
}



eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                        xMBUtilGetBits( mb_coils, usBitOffset,
                                        ( unsigned char )( iNCoils >
                                                           8 ? 8 :
                                                           iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( mb_coils, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                }
				command_status = (uint16_t)( mb_coils[1] << 8 | mb_coils[0] );// & 0x1100;  // mask available coils ////////////////								
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	  eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscretes = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    if( ( usAddress >= REG_DISCRETES_START )
        && ( usAddress + usNDiscrete <= REG_DISCRETES_START + REG_DISCRETES_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_DISCRETES_START );

		while( iNDiscretes > 0 )
        {
            *pucRegBuffer++ =
                        xMBUtilGetBits( mb_discretes, usBitOffset,
                                        ( unsigned char )( iNDiscretes >
                                                           8 ? 8 :
                                                           iNDiscretes ) );
            iNDiscretes -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
    //return MB_ENOREG;
}
