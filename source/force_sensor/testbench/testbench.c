#include "testbench.h"

void forceInit(force_t *m, q_handle_t *tx_queue)
{
    *m = (force_t) {
        .tx_queue    = tx_queue
    };

    return;
}

void forceSetParam(force_t *mi, ADCReadings_t *adc)
{
    char cmd[56]; // 37 byte + '\0'
    int arg1;

    arg1 = adc->lv_5_v_sense;


    snprintf(cmd, 56, "%04d\r\n\0", arg1);
    qSendToBack(mi->tx_queue, cmd);
}

void tiPeriodic(force_t* m) {
    tiParseMessage(m);   
}