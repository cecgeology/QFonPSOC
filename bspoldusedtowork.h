/* ========================================
 *

 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/


#ifndef bsp_h
#define bsp_h
    
#ifdef __cplusplus   //macro do deal with C++ mangling prevention
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_C_DECLS /* empty */
# define __END_C_DECLS /* empty */
#endif    

#define BSP_TICKS_PER_SEC    100U //currently needs to be fixed, as it is doubly defined with below

	/* clock and interrupt rates, in Hz */  //these are for PSOC 5, modify as needed
#define CLOCK_FREQ     66857000u  //CAN THIS BE CHANGED TO AUTOMATIC?
//#define INTERRUPT_FREQ 2u  //THIS IS IN HZ, replaced with BSP ticks per sec
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u /* Cortex-M3 hard vector */
	
//#define DebugISR_VECTOR_NUMBER 	
// MACRO EXPANSION FOR USING QF ON TOP OF PSOC
//#include <device.h>
//#include <project.h>
	
#define BSP_LedOff(); 		LED15p4_Write(0);
#define BSP_LedOn(); 		LED15p4_Write(1);	
#define BSP_Shutdown(); 	LED15p4_Write(0);
#define BSP_LedToggle(); 	LED15p4_Write(~LED15p4_Read());
    
#define BSP_Uart_PutString 	UARTA_PutString
#define BSP_Uart_PutChar	UARTA_PutChar

	
#define UARTQS_Start		UARTB_Start
#define UARTQS_PutChar	    UARTB_PutChar

/* macro to set and start custom interrupt vectors */
#ifndef 	ISRENABLER
#define ISRENABLER(hardwarename , functionname); {		\
				hardwarename ## _SetVector( functionname ); \
				hardwarename ## _SetPriority(  hardwarename ## _INTC_PRIOR_NUMBER); \
				hardwarename ## _Enable();\
				}
#endif

/* macro to set and start custom interrupt vectors with an enumerated priority*/
#ifndef 	ISRENABLERQF
#define ISRENABLERQF(hardwarename , functionname, priority); {		\
				hardwarename ## _SetVector( functionname ); \
				hardwarename ## _SetPriority( priority ); \
				hardwarename ## _Enable();\
				}
#endif







#define BSP_LCD_Start			LCD_Debug_Start	
#define BSP_LCD_ClearDisplay	LCD_Debug_ClearDisplay
#define BSP_LCD_Position		LCD_Debug_Position
#define BSP_LCD_PrintString		LCD_Debug_PrintString

__BEGIN_C_DECLS //macro do deal with C++ mangling prevention


//Function calls to implement on this particular board
	
	
/*	
#define BSP_Start_GravSystem(); {\
	CtlCalGravWrite(1u); \//fix fix
}
	*/
#define SysTickOutDividedBy2(); SysTickOut_Write(~SysTickOut_Read()); //for debugging, spit out from the Systick handler
	

//LED_Write(~LED_Read());
	//Prototypes
void BSP_init(void);
void BSP_randomSeed(uint32_t seed);
void BSP_terminate(int16_t result);	
void BSP_Command_Issued();

void BSP_Display_Init(void);

//print this string at 0,0 on the debug LCD
#ifndef LCDDEBUG
#define LCDDEBUG(str3prnt); {	\
	BSP_LCD_ClearDisplay(); \
	BSP_LCD_Position(0, 0); /* row, column */\
	BSP_LCD_PrintString( str3prnt ); \
}
#endif




__END_C_DECLS //macro do deal with C++ mangling prevention


#ifndef Q_SPY
	#define Q_SPY
	#include "qs_port.h"
#endif

	
	#endif                                                             /* bsp_h */