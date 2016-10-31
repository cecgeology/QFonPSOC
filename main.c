/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdbool.h>
#include <stdint.h>

#include "qpc.h"
#include "dpp.h"
#include "bsp.h"


#define Q_SPY //enable this for software tracing
Q_DEFINE_THIS_FILE

int main(void)
{
    //CyGlobalIntEnable; /* Enable global interrupts. */
    
    static QEvt const *tableQueueSto[N_PHILO];
    static QEvt const *philoQueueSto[N_PHILO][N_PHILO];
    static QSubscrList subscrSto[MAX_PUB_SIG];
    static QF_MPOOL_EL(TableEvt) smlPoolSto[2*N_PHILO]; /* small pool */
    uint8_t n;

    Philo_ctor(); /* instantiate all Philosopher active objects */
    Table_ctor(); /* instantiate the Table active object */

    QF_init();    /* initialize the framework and the underlying RT kernel */
    BSP_init();   /* initialize the Board Support Package */

    /* object dictionaries... */
    QS_OBJ_DICTIONARY(smlPoolSto);
    QS_OBJ_DICTIONARY(tableQueueSto);
    QS_OBJ_DICTIONARY(philoQueueSto[0]);
    QS_OBJ_DICTIONARY(philoQueueSto[1]);
    QS_OBJ_DICTIONARY(philoQueueSto[2]);
    QS_OBJ_DICTIONARY(philoQueueSto[3]);
    QS_OBJ_DICTIONARY(philoQueueSto[4]);

    /* initialize publish-subscribe... */
    QF_psInit(subscrSto, Q_DIM(subscrSto));

    /* initialize event pools... */
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

    /* start the active objects... */
    for (n = 0U; n < N_PHILO; ++n) {
        QACTIVE_START(AO_Philo[n],           /* AO to start */
                      (uint_fast8_t)(n + 1), /* QP priority of the AO */
                      philoQueueSto[n],      /* event queue storage */
                      Q_DIM(philoQueueSto[n]), /* queue length [events] */
                      (void *)0,             /* stack storage (not used) */
                      0U,                    /* size of the stack [bytes] */
                     (QEvt *)0);             /* initialization event */
    }
    QACTIVE_START(AO_Table,                  /* AO to start */
                  (uint_fast8_t)(N_PHILO + 1), /* QP priority of the AO */
                  tableQueueSto,             /* event queue storage */
                  Q_DIM(tableQueueSto),      /* queue length [events] */
                  (void *)0,                 /* stack storage (not used) */
                  0U,                        /* size of the stack [bytes] */
                  (QEvt *)0);                /* initialization event */

    return QF_run(); /* run the QF application */
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
 
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
