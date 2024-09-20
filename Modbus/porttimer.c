
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
//#include "MDR32F9Qx_config.h"
#include "plib035_tmr.h" //#include "MDR32F9Qx_timer.h"
//#include "MDR32F9Qx_rst_clk.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- static functions ---------------------------------*/
//static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
extern void InitFrqCheck (void);
  
uint32_t CPUclk; 

BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	RCU_APBClkCmd(RCU_APBClk_TMR0, ENABLE);
	RCU_APBRstCmd(RCU_APBRst_TMR0, ENABLE);
	TMR_PeriodConfig(TMR0, SystemCoreClock, SystemCoreClock / 20000);
	TMR_ITCmd(TMR0, ENABLE);
	TMR_ExtInputConfig(TMR0, TMR_ExtInput_Disable);
	TMR_Cmd(TMR0, ENABLE);
	NVIC_EnableIRQ(TMR0_IRQn);
	return TRUE;
}

void vMBPortTimerClose( void )
{
	NVIC_DisableIRQ(TMR0_IRQn);		
	TMR_ITCmd(TMR0, DISABLE);	
	TMR_Cmd(TMR0, DISABLE);
	TMR_SetCounter(TMR0, 0);
}

void vMBPortTimersEnable()
{
	/* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	TMR_Cmd(TMR0, DISABLE);
	TMR_SetCounter(TMR0, 0);
	TMR_ITCmd(TMR0, ENABLE);
	TMR_Cmd(TMR0, ENABLE);	 
	NVIC_EnableIRQ(TMR0_IRQn);
}

void vMBPortTimersDisable()
{
  /* Disable any pending timers. */
	NVIC_DisableIRQ(TMR0_IRQn);
	TMR_Cmd(TMR0, DISABLE);	 
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */

void TMR0_IRQHandler(void)
{
	TMR_ITStatusClear(TMR0);
	pxMBPortCBTimerExpired(  );
}




