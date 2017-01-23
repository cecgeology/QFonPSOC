/*****************************************************************************
* Product: DPP example, EFM32-SLSTK3401A board, cooperative QV kernel
* Last Updated for Version: 5.6.5
* Date of the Last Update:  2016-05-08
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) Quantum Leaps, LLC. All rights reserved.
*
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
* Contact information:
* http://www.state-machine.com
* mailto:info@state-machine.com
*****************************************************************************/
#include <stdbool.h> //boolean support
#include <stdint.h> 
#include <stdio.h>

/* comment this out if Q_SPY is not to be enabled */
//#if !defined Q_SPY 
//    #define Q_SPY //although enabled in main, its here so the IDE load that component
//    Q_ASSERT_COMPILE();
//#endif 

#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include <project.h>  //PSOC Part library
#include <cytypes.h>  //PSOC type library



/* add other drivers if necessary... */

Q_DEFINE_THIS_FILE

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
* DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
*/
enum KernelUnawareISRs { /* see NOTE00 */
    USARTQS_RX_PRIO,
    /* ... */
    MAX_KERNEL_UNAWARE_CMSIS_PRI  /* keep always last */
};
/* "kernel-unaware" interrupts can't overlap "kernel-aware" interrupts */
Q_ASSERT_COMPILE(MAX_KERNEL_UNAWARE_CMSIS_PRI <= QF_AWARE_ISR_CMSIS_PRI);










enum KernelAwareISRs {
    SYSTICK_PRIO = QF_AWARE_ISR_CMSIS_PRI, /* see NOTE00 */
    ISRSWITCH1_PRIO,
    ISRSWITCH2_PRIO,
    DEBUG_PRIO,
    
    /* ... */
    MAX_KERNEL_AWARE_CMSIS_PRI /* keep always last */
};
/* "kernel-aware" interrupts should not overlap the PendSV priority */
Q_ASSERT_COMPILE(MAX_KERNEL_AWARE_CMSIS_PRI <= (0xFF >>(8-__NVIC_PRIO_BITS)));

/* ISRs defined in this BSP ------------------------------------------------*/
void SysTick_Handler(void); //this one is standard to M3, but not in the style Cypress PSOC

//Following ISRs are declared using Cypress Prototype Macro
CY_ISR_PROTO(isrDebug_Handler);
CY_ISR_PROTO(isrSwitch1_Handler);
CY_ISR_PROTO(isrSwitch2_Handler);
CY_ISR_PROTO(isrUartQsRx_Handler);


/* Local-scope objects -----------------------------------------------------*/

static uint32_t l_rnd;      /* random seed */

#ifdef Q_SPY

    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    /* QS source IDs */
    static uint8_t const l_SysTick_Handler = (uint8_t)0;
    static uint8_t const l_GPIO_EVEN_IRQHandler = (uint8_t)0;
    static uint8_t const l_isrSwitch1_Handler = (uint8_t)0;;     
	static uint8_t const l_isrSwitch2_Handler = (uint8_t)0;; 	

    
    
    // static USART_TypeDef * const l_USARTQS = ((USART_TypeDef *)(0x40010000UL));

    enum AppRecords { /* application-specific trace records */
        PHILO_STAT = QS_USER,
        COMMAND_STAT
    };

#endif

/*..........................................................................*/
CY_ISR(SysTick_Handler){
/* no need to clear interrupt source with SysTick ? */
    uint32_t tmp = 0;
	
#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif

   // QF_TICK_X(0U, &l_SysTick_Handler); /* process time events for rate 0 */
    //assiging a 0 to the rate here arms a one-shot event
//    QF_PUBLISH(&tickEvt, &l_SysTick_Handler); /* publish to all subscribers */

//the new way (from 5.8.1) to deal with the QF Ticks is to post to an active object  
QACTIVE_POST(the_Ticker0, 0, 0); /* post a don't-care event to Ticker0 */

}

/* put in all other interrupt handlers here */
CY_ISR(isrSwitch1_Handler)
{
	if (PinSwitch1_Read()>0){
            static QEvt const pauseEvt = { PAUSE_SIG, 0U, 0U};
            QF_PUBLISH(&pauseEvt, &l_isrSwitch1_Handler);
    }
        else {            /* the button is released */
            static QEvt const serveEvt = { SERVE_SIG, 0U, 0U};
            QF_PUBLISH(&serveEvt, &l_isrSwitch1_Handler);
        }

}

/*..........................................................................*/
#ifdef Q_SPY
/*
* ISR for receiving bytes from the QSPY Back-End
* NOTE: This ISR is "QF-unaware" meaning that it does not interact with
* the QF/QK and is not disabled. Such ISRs don't need to call QK_ISR_ENTRY/
* QK_ISR_EXIT and they cannot post or publish events.
*/
void isrUartQsRx_Handler(void) {
    /* while RX FIFO NOT empty */
   // while ((l_USARTQS->STATUS & USART_STATUS_RXDATAV) != 0) {
        //uint32_t b = l_USARTQS->RXDATA;
    uint8 b = 0;
while ((b = UARTQS_GetChar())> 0){    
     
        QS_RX_PUT(b);
    }
}
#else
void isrUartQsRx_Handler(void) {}
#endif

/*..........................................................................*/
void BSP_init(void) {
  
    //...
    BSP_randomSeed(1234U);
BSP_LedOn();
    if (QS_INIT((void *)0) == 0) { /* initialize the QS software tracing */
        Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_OBJ_DICTIONARY(&l_GPIO_EVEN_IRQHandler);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);
}
/*..........................................................................*/
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    if (stat[0] == 'e') {
         BSP_LedOn(); 
    }
    else {
         BSP_LedOff(); 
        }
    QS_BEGIN(PHILO_STAT, AO_Philo[n]) /* application-specific record begin */
        QS_U8(1, n);  /* Philosopher number */
        QS_STR(stat); /* Philosopher status */
    QS_END()
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    if (paused != 0U) {
         BSP_LedOn(); 
    }
    else {
         BSP_LedOff(); 
        }
}
/*..........................................................................*/
uint32_t BSP_random(void) { /* a very cheap pseudo-random-number generator */
    uint32_t rnd;

    /* Some flating point code is to exercise the VFP... */
    float volatile x = 3.1415926F;
    x = x + 2.7182818F;

    /* "Super-Duper" Linear Congruential Generator (LCG)
    * LCG(2^32, 3*7*11*13*23, 0, seed)
    */
    rnd = l_rnd * (3U*7U*11U*13U*23U);
    l_rnd = rnd; /* set for the next time */

    return (rnd >> 8);
}
/*..........................................................................*/
void BSP_randomSeed(uint32_t seed) {
    l_rnd = seed;
}
/*..........................................................................*/
void BSP_terminate(int16_t result) {
    (void)result;
}

/*..........................................................................*/
void QF_onStartup(void) {
    /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate */
#if (CY_PSOC3 || CY_PSOC5LP)
    #define  SYSTICK_PERIOD     ((BCLK__BUS_CLK__HZ))
#elif (CY_PSOC4)
    #define  SYSTICK_PERIOD     ((CYDEV_BCLK__HFCLK__HZ))
#endif /* CY_PSOC5A */

    SysTick_Config(SYSTICK_PERIOD / BSP_TICKS_PER_SEC);

    /* point the Systick vector to the ISR in this file */
    CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_Handler);//PSOC SPECIFIC
    
    
    /* assing all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);//CMSIS function

    
    /* INTERRUPT VECTORS SET HERE */
    isrSwitch1_SetVector(isrSwitch1_Handler);
//    isrSwitch2_SetVector(isrSwitch2_Handler);
    /* set priorities of ALL ISRs used in the system, see NOTE00
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    //xxxx_SetPriority(XXXX_PRIO);


    isrSwitch1_SetPriority(ISRSWITCH1_PRIO);
    isrSwitch2_SetPriority(ISRSWITCH2_PRIO);    
    
    /* enable IRQs... by using the xxxx_Enable(); function */
	//DON'T CALL THE xxxx_Start() function provided by cypress or the priority will be lost, instead enable    
 	//isrSwitch2_Enable();   
    
    isrSwitch1_Enable();
    isrSwitch2_Enable();

#ifdef Q_SPY
    /* interrupt used for QS-RX */
        isrUartQsRx_SetPriority(USARTQS_RX_PRIO);
        isrUartQsRx_SetVector(isrUartQsRx_Handler);
        isrUartQsRx_Enable(); 
#endif

    CyGlobalIntEnable; /*macro to enable system wide interrupts*/
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
void QV_onIdle(void) { /* CATION: called with interrupts DISABLED, NOTE01 */
    /* toggle the User LED on and then off, see NOTE02 */

#ifdef Q_SPY
    QF_INT_ENABLE();
    QS_rxParse();  /* parse all the received bytes */
    //uint8 temp = 0;
    //temp = UARTQS_ReadTxStatus();
    if (UARTQS_ReadTxStatus()!=UARTQS_TX_STS_FIFO_FULL ) {  /* is TX buffer not full? */
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  /* not End-Of-Data? */
            UARTQS_PutChar((uint8_t)b); //this is a blocking function but it is not called if it could block.
           // l_USARTQS->TXDATA = (b & 0xFFU);  /* put into the DR register */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M MCU.
    */
    QV_CPU_SLEEP();  /* atomically go to sleep and enable interrupts */
#else
    QF_INT_ENABLE(); /* just enable interrupts */
#endif
}

/*..........................................................................*/
void Q_onAssert(char const *module, int loc) {
    /*
    * NOTE: add here your application-specific error handling
    */
    (void)module;                   /* avoid compiler warning */
    (void)loc;                      /* avoid compiler warning */
    QS_ASSERTION(module, loc, (uint32_t)10000U); /* report assertion to QS */

#ifndef NDEBUG
    /* light up both LEDs */
//    GPIO->P[LED_PORT].DOUT |= ((1U << LED0_PIN) | (1U << LED1_PIN));
    /* for debugging, hang on in an endless loop until PB1 is pressed... */
//    while ((GPIO->P[PB_PORT].DIN & (1U << PB1_PIN)) != 0) {
 //   }
#endif

    NVIC_SystemReset();
}

/* QS callbacks ============================================================*/
#ifdef Q_SPY
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsTxBuf[2*1024]; /* buffer for QS transmit channel */
    static uint8_t qsRxBuf[100];    /* buffer for QS receive channel */
    
    UARTQS_Start(); //carefull this may kill the interrupt priority
    
UARTQS_PutString("QS UART ONLINE"); //just to debug

  
    QS_initBuf  (qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

 
    /* Enable RX interrupts */
    /* NOTE: do not enable the UART0 interrupt in the NVIC yet.
    * Wait till QF_onStartup()
    */

    /* Finally enable the UART */
#if (CY_PSOC3 || CY_PSOC5LP)
    #define  SystemCoreClock     ((BCLK__BUS_CLK__HZ))
#elif (CY_PSOC4)
    #define  SystemCoreClock     ((CYDEV_BCLK__HFCLK__HZ))
#endif /* CY_PSOC5A */

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */

    /* setup the QS filters... */
    QS_FILTER_ON(QS_QEP_STATE_ENTRY);
    QS_FILTER_ON(QS_QEP_STATE_EXIT);
    QS_FILTER_ON(QS_QEP_STATE_INIT);
    QS_FILTER_ON(QS_QEP_INIT_TRAN);
    QS_FILTER_ON(QS_QEP_INTERN_TRAN);
    QS_FILTER_ON(QS_QEP_TRAN);
    QS_FILTER_ON(QS_QEP_IGNORED);
    QS_FILTER_ON(QS_QEP_DISPATCH);
    QS_FILTER_ON(QS_QEP_UNHANDLED);

    QS_FILTER_ON(PHILO_STAT);
    QS_FILTER_ON(COMMAND_STAT);

    /* Alternately turn them all on and selectively off */
    //  QS_FILTER_ON(QS_ALL_RECORDS);

	
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
*/
/*    QS_FILTER_OFF(QS_QF_ACTIVE_POST_FIFO);
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
    */
    
    return (uint8_t)1; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {  /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t b;

    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) { /* while not End-Of-Data... */
        QF_INT_ENABLE();
        /* while TXE not empty */
        UARTQS_PutChar((uint8_t)b);
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
}
/*..........................................................................*/
/*! callback function to reset the target (to be implemented in the BSP) */
void QS_onReset(void) {
    NVIC_SystemReset();
}
/*..........................................................................*/
/*! callback function to execute a user command (to be implemented in BSP) */
void QS_onCommand(uint8_t cmdId, uint32_t param) {
 //   void assert_failed(char const *module, int loc);
    (void)cmdId;
    (void)param;
    QS_BEGIN(COMMAND_STAT, (void *)1) /* application-specific record begin */
        QS_U8(2, cmdId);
        QS_U32(8, param);
    QS_END()

    if (cmdId == 10U) {
        Q_ERROR();
    }
    else if (cmdId == 11U) {
        assert_failed("QS_onCommand", 123);
    }
}

#endif /* Q_SPY */
/*--------------------------------------------------------------------------*/

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
uint8 j = 10;
while (j>0){
CyDelay(100);
BSP_LedToggle();
j--;
}
UARTA_PutCRLF(' ');	
NVIC_SystemReset();                             /* perform system reset */
}




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
