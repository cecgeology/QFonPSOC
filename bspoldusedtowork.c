/*****************************************************************************
* Date of the Last Update:  Oct 19, 2016
*
* Copyright (C) CEC Geology LLC 2016-
* All Rights Reserved
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
*****************************************************************************/
/* ================REVISION===HISTORY===========
*
* 2016 October
* Copied old code previoulsy used in creator 3 with psoc5
* merged in with Quantum Leaps Version: 5.6.5 DPP example 
* old was DPP example for Version: 5.2.0
*
*
*
*/
#include <stdbool.h> //boolean support
#include <stdint.h> 
#include <stdio.h>

#include "qp_port.h"
//#include "QFproject.h"
#include "bsp.h"
#include "qassert.h"

#include <project.h>  //PSOC Part library
#include <cytypes.h>  //PSOC type library

/* add other drivers if necessary... */

Q_DEFINE_THIS_FILE

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
* DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
*/
enum KernelUnawareISRs { /* see NOTE00 */
    USART0_RX_PRIO,
    /* ... */
    MAX_KERNEL_UNAWARE_CMSIS_PRI  /* keep always last */
};
/* "kernel-unaware" interrupts can't overlap "kernel-aware" interrupts */
Q_ASSERT_COMPILE(MAX_KERNEL_UNAWARE_CMSIS_PRI <= QF_AWARE_ISR_CMSIS_PRI);

enum KernelAwareISRs {
    GPIO_EVEN_PRIO = QF_AWARE_ISR_CMSIS_PRI, /* see NOTE00 */
    SYSTICK_PRIO,
    /* ... */
    MAX_KERNEL_AWARE_CMSIS_PRI /* keep always last */
};
/* "kernel-aware" interrupts should not overlap the PendSV priority */
Q_ASSERT_COMPILE(MAX_KERNEL_AWARE_CMSIS_PRI <= (0xFF >>(8-__NVIC_PRIO_BITS)));

/* ISRs defined in this BSP ------------------------------------------------*/
void SysTick_Handler(void); //this one is standard to M3, but not in the style Cypress PSOC

/* ISRs 
CY_ISR_PROTO(isrDebug_Handler);
CY_ISR_PROTO(Switch2_Handler);
CY_ISR_PROTO(Switch3_Handler);
CY_ISR_PROTO(DmaFreqReadDone_Handler);
CY_ISR_PROTO(DmaFreqCalDone_Handler);
*/

/* Local-scope objects -----------------------------------------------------*/
#ifdef Q_SPY

    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;
    
        /* QS source IDs These need to be added manually*/
    static uint8_t l_SysTick_Handler; // define thesevariables for auto more readable and QSPY dump
    static uint8_t l_Switch2_Handler;     
	static uint8_t l_Switch3_Handler; 	
	static uint8_t l_DmaFreqCalDone_Handler;

 static uint8_t const l_UartA = 0u;
//    #define UART_BAUD_RATE      115200U
//    #define UART_TXFIFO_DEPTH   4U
//    #define UART_FR_TXFE        0x00000080U

    enum AppRecords {                 /* application-specific trace records */
        PHILO_STAT = QS_USER,
		TERMINAL_STAT
    };

#endif

/*..........................................................................*/

CY_ISR(SysTick_Handler){
/* no need to clear interrupt source with SysTick */
    static QEvt const tickEvt = { TIME_TICK_SIG, 0U, 0U };

    #ifdef Q_SPY

{
    uint32_t tmp = SysTick->CTRL;     /* clear SysTick_CTRL_COUNTFLAG */
    QS_tickTime_ += QS_tickPeriod_;   /* account for the clock rollover */
}
#endif
#ifdef SYSTICKDIV2OUT
//	SysTickOut_Write(~SysTickOut_Read()); //toggle a pin for debbuging reasons
#endif

	QF_TICK_X(0U, &l_SysTick_Handler); /* process all time events at rate 0 */ //This call will service all time events assigned to the tick rate 0.
    //assiging a 0 to the rate here arms a one-shot event
	
//QF_PUBLISH(&tickEvt, &l_SysTick_Handler); /* publish to all subscribers */

//Do anything else here that needs to be done every systick

}

/* put in all other interrupt handlers here */
CY_ISR(Switch2_Handler)
{
	
static QEvt const gEvent = {DEBUG_SIG, 0u, 0u};  //post a static and base type QEvt event. Note, the whole struct (QEvt) has to be filled and invariant
QF_PUBLISH((QEvt *)&gEvent, &l_Switch2_Handler);    

}


/*..........................................................................*/
/*..........................................................................*/
void BSP_init(void) {
//INITIALIZATION CODE
    
    //	CyGlobalIntEnable;	
	UARTA_Start();

//	UARTA_PutString("\n\rUART READY\n\r");
//	UARTA_PutCRLF(' ');

    if (QS_INIT((void *)0) == 0) {    /* initialize the QS software tracing */
        Q_ERROR();
    }
    //MAKE SURE THE  QS software tracing OBJECTS ARE ADDED HERE
    QS_OBJ_DICTIONARY(&l_SysTick_Handler); 
	QS_OBJ_DICTIONARY(&l_UartA);
	//QS_OBJ_DICTIONARY(&l_UartA);
    //QS_OBJ_DICTIONARY(&l_GPIOPortA_IRQHandler);
}
/*..........................................................................*/
void BSP_terminate(int16_t result) {
    (void)result;
}

/*..........................................................................*/
void QF_onStartup(void) {
   /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate */

    /* point the Systick vector to the ISR in this file */
    CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_Handler);//PSOC SPECIFIC
   /* Set the number of ticks between interrupts.
       Ignore the function success/fail return value.
    */
    (void)SysTick_Config(CLOCK_FREQ / BSP_TICKS_PER_SEC); /* defined in auto-generated core_cm3.h */

    	/* DISABLE INTERRUPTS BEFORE CHANGING */
    
    //	isrUartArx_Disable();
	//isrSwitch2_SetVector(Switch2_Handler);	
	
	/* INTERRUPT VECTORS SET HERE */
//	isrUartArx_SetVector(isrUartArx_Handler);

	//isrDebug_StartEx(isrDebug_Handler) 	;// note calling this changes its priority //PSOC SPECIFIC 
 
	
	/* assigning all priority bits for preemption-prio. and none to sub-prio. */
	NVIC_SetPriorityGrouping(0U);

    /* set priorities of ALL ISRs used in the system, see NOTE00
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    
    NVIC_SetPriority(SysTick_IRQn,   SYSTICK_PRIO); //CMSIS function

    //xxxx_SetPriority(XXXX_PRIO);
    
    //isrSwitch2_SetPriority(SWITCH2_PRIO);

     /* enable IRQs... by using the xxxx_Enable(); function */
	//DON'T CALL THE _Start() function provided by cypress or the priority will be lost, instead enable    
    
	//isrSwitch2_Enable();

	
    
    ISRENABLER(isrSwitch2, Switch2_Handler);
    
	CyGlobalIntEnable;
    
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
void QF_onIdle(void) {       /* called with interrupts disabled, see NOTE01 */

    /* toggle the User LED on and then off, see NOTE02 */
//	  LED15p4_Write(~LED15p4_Read());//WARNING BOARD SPECIFIC
#ifdef Q_SPY
    QF_INT_ENABLE();

	//if ((UART0->FR & UART_FR_TXFE) != 0) {                      /* TX done? */
        uint16_t b;       /* max bytes we can accept */
//        uint8_t const *block;

        QF_INT_DISABLE();
  		b = QS_getByte();      //block = QS_getBlock(&fifo);    /* try to get next block to transmit */
		QF_INT_ENABLE();
		
    if (b != QS_EOD) {
       // QF_INT_ENABLE();
		
		UARTQS_PutChar((uint8_t)b);
	}
	//	UartQS_PutArray(block, UART_TXFIFO_DEPTH);
  //      while (fifo-- != 0) {                    /* any bytes in the block? */
  //          UART0->DR = *block++;                      /* put into the FIFO */
        //}
    //}
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M MCU.
    */
    QF_CPU_SLEEP();         /* atomically go to sleep and enable interrupts */
#else
    QF_INT_ENABLE();                              /* just enable interrupts */
#endif
}

/*..........................................................................*/
void Q_onAssert(char const Q_ROM * const file, int_t line) {
    assert_failed(file, line);
}
/*..........................................................................*/
/* error routine that is called if the CMSIS library encounters an error    */
void assert_failed(char const *file, int line) {
    (void)file;                                   /* avoid compiler warning */
    (void)line;                                   /* avoid compiler warning */
    QF_INT_DISABLE();         /* make sure that all interrupts are disabled */
//	sprintf(file);
char tempPrint[16];
UARTA_PutCRLF(' ');	
UARTA_PutString("Assertion FAILED in file: ");
UARTA_PutString(file);
UARTA_PutString(" at line: ");	
sprintf(tempPrint, "%10u", line);
UARTA_PutString(tempPrint);
QS_onFlush(); //?
CyDelay(1000);
UARTA_PutCRLF(' ');	
NVIC_SystemReset();                             /* perform system reset */
}

/* QS callbacks ============================================================*/

#ifdef Q_SPY
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[2*1024];                 /* buffer for Quantum Spy */
    uint32_t tmp;
    QS_initBuf(qsBuf, sizeof(qsBuf));
	
	UARTQS_Start();
//	UARTQS_PutString("QS UART ONLINE"); just to debug
    
    QS_tickPeriod_ = CLOCK_FREQ / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_;        /* to start the timestamp at zero */

                                                 /* setup the QS filters... */
  QS_FILTER_ON(QS_ALL_RECORDS);

	
/*    QS_FILTER_OFF(QS_ALL_RECORDS);
    QS_FILTER_OFF(QS_QEP_STATE_ENTRY);
    QS_FILTER_OFF(QS_QEP_STATE_EXIT);
    QS_FILTER_OFF(QS_QEP_STATE_INIT);
    QS_FILTER_OFF(QS_QEP_INIT_TRAN);
    QS_FILTER_OFF(QS_QEP_INTERN_TRAN);
    QS_FILTER_OFF(QS_QEP_TRAN);
    QS_FILTER_OFF(QS_QEP_IGNORED);

    QS_FILTER_OFF(QS_QF_ACTIVE_ADD);
    QS_FILTER_OFF(QS_QF_ACTIVE_REMOVE);
    QS_FILTER_OFF(QS_QF_ACTIVE_SUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_UNSUBSCRIBE);
*/    QS_FILTER_OFF(QS_QF_ACTIVE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET_LAST);
//    QS_FILTER_OFF(QS_QF_EQUEUE_INIT);
//    QS_FILTER_OFF(QS_QF_EQUEUE_POST_FIFO);
//    QS_FILTER_OFF(QS_QF_EQUEUE_POST_LIFO);
//    QS_FILTER_OFF(QS_QF_EQUEUE_GET);
//    QS_FILTER_OFF(QS_QF_EQUEUE_GET_LAST);
//    QS_FILTER_OFF(QS_QF_MPOOL_INIT);
//    QS_FILTER_OFF(QS_QF_MPOOL_GET);
//    QS_FILTER_OFF(QS_QF_MPOOL_PUT);
//    QS_FILTER_OFF(QS_QF_PUBLISH);
//    QS_FILTER_OFF(QS_QF_NEW);
//    QS_FILTER_OFF(QS_QF_GC_ATTEMPT);
//    QS_FILTER_OFF(QS_QF_GC);
    QS_FILTER_OFF(QS_QF_TICK);
//    QS_FILTER_OFF(QS_QF_TIMEEVT_ARM);
//    QS_FILTER_OFF(QS_QF_TIMEEVT_AUTO_DISARM);
//    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM_ATTEMPT);
//    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM);
//    QS_FILTER_OFF(QS_QF_TIMEEVT_REARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_POST);
    QS_FILTER_OFF(QS_QF_CRIT_ENTRY);
    QS_FILTER_OFF(QS_QF_CRIT_EXIT);
    QS_FILTER_OFF(QS_QF_ISR_ENTRY);
    QS_FILTER_OFF(QS_QF_ISR_EXIT);
	
	

    return (uint8_t)1;                                    /* return success */
}

/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {            /* invoked with interrupts locked */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {    /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else {     /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
   // uint16_t fifo = UART_TXFIFO_DEPTH;                     /* Tx FIFO depth */
   // uint8_t const *block;
	uint16_t b;
    QF_INT_DISABLE();
//    while ((block = QS_getBlock(&fifo)) != (uint8_t *)0) {
    while ((b = QS_getByte()) != QS_EOD) {
       // QF_INT_ENABLE();
		
		UARTQS_PutChar((uint8_t)b);
		
		}
		/* busy-wait until TX FIFO empty */
//        while ((UART0->FR & UART_FR_TXFE) == 0) {
//        }

//        while (fifo-- != 0) {                    /* any bytes in the block? */
//            UART0->DR = *block++;                   /* put into the TX FIFO */
        //}
  //      fifo = UART_TXFIFO_DEPTH;              /* re-load the Tx FIFO depth */
	//	UartQS_PutArray(block, UART_TXFIFO_DEPTH);
       // QF_INT_DISABLE();
    QF_INT_ENABLE();
}
#endif                                                             /* Q_SPY */
/*--------------------------------------------------------------------------*/


/*****************************************************************************
* NOTE00:
* The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
* ISR priority that is disabled by the QF framework. The value is suitable
* for the NVIC_SetPriority() CMSIS function.
*
* Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
* with the numerical values of priorities equal or higher than
* QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QK_ISR_ENTRY/QK_ISR_ENTRY
* macros or any other QF/QK  services. These ISRs are "QF-aware".
*
* Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
* level (i.e., with the numerical values of priorities less than
* QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
* Such "QF-unaware" ISRs cannot call any QF/QK services. In particular they
* can NOT call the macros QK_ISR_ENTRY/QK_ISR_ENTRY. The only mechanism
* by which a "QF-unaware" ISR can communicate with the QF framework is by
* triggering a "QF-aware" ISR, which can post/publish events.
*
* NOTE01:
* The QV_onIdle() callback is called with interrupts disabled, because the
* determination of the idle condition might change by any interrupt posting
* an event. QV_onIdle() must internally enable interrupts, ideally
* atomically with putting the CPU to the power-saving mode.
*
* NOTE02:
* The User LED is used to visualize the idle loop activity. The brightness
* of the LED is proportional to the frequency of invcations of the idle loop.
* Please note that the LED is toggled with interrupts locked, so no interrupt
* execution time contributes to the brightness of the User LED.
*/





/* [] END OF FILE */
