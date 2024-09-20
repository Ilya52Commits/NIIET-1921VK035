// MB Serial interface to MDR32F
#include <string.h>
#include "port.h"
//#include "MDR32F9Qx_config.h"
//#include "MDR32Fx.h"
//#include "MDR1986BE4.h"
//#include "MDR32F9Qx_rst_clk.h"
#include "plib035_gpio.h" //#include "MDR32F9Qx_port.h"
#include "plib035_uart.h" //#include "MDR32F9Qx_uart.h" 
//#include "ComDef.h"
#include "plib035.h"
#include "K1921VK035.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Start implementation -----------------------------*/
extern uint32_t mbBaudrate;
extern uint8_t mbAddress;

GPIO_Init_TypeDef GPIO_InitStruct;
UART_Init_TypeDef UART_InitStruct;
uint32_t data[10], index5 = 0;
uint16_t arraySENT[10], INDXX=0;

extern volatile UCHAR ucRTUBuf[];

UCHAR xPort;

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if (xRxEnable)
	{
		GPIO_InitStruct.AltFunc = ENABLE;
		GPIO_InitStruct.Pin = (xPort == 0) ? GPIO_Pin_11 : GPIO_Pin_9;
		GPIO_Init(GPIOB, &GPIO_InitStruct);      //?????
		NVIC_EnableIRQ((xPort == 0) ? UART0_RX_IRQn : UART1_RX_IRQn); 
    }
    else
	{
		GPIO_InitStruct.AltFunc = DISABLE;
		GPIO_InitStruct.Pin = (xPort == 0) ? GPIO_Pin_11 : GPIO_Pin_9;
		GPIO_Init(GPIOB, &GPIO_InitStruct);			
		NVIC_DisableIRQ((xPort == 0) ? UART0_RX_IRQn : UART1_RX_IRQn); 
    }
	
    if(xTxEnable)
	{
	   	GPIO_InitStruct.AltFunc = ENABLE;
		GPIO_InitStruct.Pin = (xPort == 0) ? GPIO_Pin_10 : GPIO_Pin_8;
		GPIO_Init(GPIOB, &GPIO_InitStruct);			
		NVIC_EnableIRQ((xPort == 0) ? UART0_TX_IRQn : UART1_TX_IRQn); 
    }
    else
	{
		GPIO_InitStruct.AltFunc = DISABLE;
		GPIO_InitStruct.Pin = (xPort == 0) ? GPIO_Pin_10 : GPIO_Pin_8;
		GPIO_Init(GPIOB, &GPIO_InitStruct);			
		NVIC_DisableIRQ((xPort == 0) ? UART0_TX_IRQn : UART1_TX_IRQn); 
    }  
}



BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	// Enables the clock on PORTB
	RCU_AHBClkCmd(RCU_AHBClk_GPIOB, ENABLE);
	RCU_AHBRstCmd(RCU_AHBRst_GPIOB, ENABLE);
	
	xPort = ucPORT; 
  
	// Fill PortInit structure
	GPIO_InitStruct.AltFunc = ENABLE;
	GPIO_InitStruct.OutMode = GPIO_OutMode_PP;
	GPIO_InitStruct.PullMode = GPIO_PullMode_PU;
	GPIO_InitStruct.Pin = (ucPORT == 0) ? GPIO_Pin_10 : GPIO_Pin_8;
	GPIO_InitStruct.Digital = ENABLE;
	GPIO_InitStruct.DriveMode = GPIO_DriveMode_HighFast;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
  
	// Configure PORTB pins 1 (UART1_RX) as input
	GPIO_InitStruct.Pin = (ucPORT == 0) ? GPIO_Pin_11 : GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	  

	// Initialize USART_InitStructure
	RCU_UARTClkConfig((ucPORT == 0) ? UART0_Num : UART1_Num, RCU_PeriphClk_PLLClk, 1, ENABLE);
	
	RCU_UARTClkCmd((ucPORT == 0) ? UART0_Num : UART1_Num, ENABLE);
	RCU_UARTRstCmd((ucPORT == 0) ? UART0_Num : UART1_Num, ENABLE);
	
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.BaudRate = (uint32_t)ulBaudRate;
	UART_InitStruct.Tx = ENABLE;
	UART_InitStruct.Rx = ENABLE;
	UART_InitStruct.DataWidth = UART_DataWidth_8;
	UART_InitStruct.ParityBit = UART_ParityBit_Disable;
	UART_InitStruct.StopBit = UART_StopBit_1;
	UART_InitStruct.FIFO = DISABLE;

	// Configure UART parameters
	UART_Init((ucPORT == 0) ? UART0 : UART1, &UART_InitStruct);
	UART_Cmd((ucPORT == 0) ? UART0 : UART1, ENABLE);

	UART_ITCmd((ucPORT == 0) ? UART0 : UART1, UART_ITSource_All, ENABLE);
	NVIC_EnableIRQ((ucPORT == 0) ? UART0_TX_IRQn : UART1_TX_IRQn); 
	NVIC_EnableIRQ((ucPORT == 0) ? UART0_RX_IRQn : UART1_RX_IRQn); 
	__NVIC_SetPriority((ucPORT == 0) ? UART0_TX_IRQn : UART1_TX_IRQn, 1);
	__NVIC_SetPriority((ucPORT == 0) ? UART0_RX_IRQn : UART1_RX_IRQn, 1);
	__NVIC_SetPriority(TMR0_IRQn, 1);

	return TRUE;
}

void
vMBPortSerialClose( void )
{
    UART_Cmd((xPort == 0) ? UART0 : UART1, DISABLE);		// << CHNG >>
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */

	UART_SendData((xPort == 0) ? UART0 : UART1, (uint16_t) ucByte);
	//while(UART_FlagStatus(UART1, UART_Flag_Busy) == SET) {};
    return TRUE; // No checking!
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */

	*pucByte = (uint8_t)UART_RecieveData((xPort == 0) ? UART0 : UART1);

    return TRUE; // No checking!
}


void UART1_RX_IRQHandler()
{
	//UART1->ICR |= UART_FR_RXFE_Msk;
	//UART_ITStatusClear(UART1, UART_IMSC_RTIM_Msk);
	//index5 = 0;
	//data[index5++] = UART1->DR;
	pxMBFrameCBByteReceived(  ); 
}

void UART0_RX_IRQHandler()
{
	//UART1->ICR |= UART_FR_RXFE_Msk;
	//UART_ITStatusClear(UART1, UART_IMSC_RTIM_Msk);
	//index5 = 0;
	//data[index5++] = UART1->DR;
	pxMBFrameCBByteReceived(  ); 
}

void UART1_TX_IRQHandler()
{
	UART_ITStatusClear(UART1, UART_ITSource_TxFIFOLevel);
	pxMBFrameCBTransmitterEmpty(  );
}

void UART0_TX_IRQHandler()
{
	UART_ITStatusClear(UART0, UART_ITSource_TxFIFOLevel);
	pxMBFrameCBTransmitterEmpty(  );
}

void UART1_TD_IRQHandler()
{
	UART_ITStatusClear(UART1, UART_ITSource_TxFIFOLevel);
	//inter1++;
}

void UART0_TD_IRQHandler()
{
	UART_ITStatusClear(UART0, UART_ITSource_TxFIFOLevel);
	//inter1++;
}

void UART1_E_RT_IRQHandler()
{
	//inter4++;
}


/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
