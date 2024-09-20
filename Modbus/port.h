#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/

#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

/* ----------------------- Defines ------------------------------------------*/
#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )   vMBPortEnterCritical()
#define EXIT_CRITICAL_SECTION( )    vMBPortExitCritical()

#define assert( x )

typedef char    BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE                                    1
#endif

#ifndef FALSE
#define FALSE                                   0
#endif

#define MB_PORT_HAS_CLOSE	                    1
#define MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS    2

/* ----------------------- Prototypes ---------------------------------------*/
void            vMBPortEnterCritical( void );
void            vMBPortExitCritical( void );

#endif
