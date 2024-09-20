

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "plib035.h"

/* ----------------------- Modbus includes ----------------------------------*/
//#include <intrinsics.h>

/* ----------------------- Variables ----------------------------------------*/
//static ULONG    ulNesting;
//static __istate_t xOldState;

extern UCHAR xPort;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortEnterCritical( )
{
	// disable interrupts
	NVIC_DisableIRQ((xPort == 0) ? UART0_TX_IRQn : UART1_TX_IRQn);
	NVIC_DisableIRQ((xPort == 0) ? UART0_RX_IRQn : UART1_RX_IRQn);
}

void
vMBPortExitCritical( )
{

	// enable interrupts
	NVIC_EnableIRQ((xPort == 0) ? UART0_TX_IRQn : UART1_TX_IRQn);
	NVIC_EnableIRQ((xPort == 0) ? UART0_RX_IRQn : UART1_RX_IRQn);
}

void
vMBPortClose( )
{
    extern void vMBPortSerialClose( );
    extern void vMBPortTimerClose( );
    vMBPortSerialClose( );
    vMBPortTimerClose( );
}
