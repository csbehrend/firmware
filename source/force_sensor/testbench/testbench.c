#include "testbench.h"

static int16_t tiParseTerm(char *rx_buf, uint8_t start, char *search_term, uint32_t *val_addr);

void forceInit(force_t *m, q_handle_t *tx_queue){
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

/*
static void tiParseMessage(micro_t *m)
{
    char     tmp_rx_buf[TI_MAX_RX_LENGTH];
    int16_t  curr;
    uint8_t  i;
    uint32_t val_buf;

    curr = m->last_msg_loc;

    // Copy buffer so it doesn't change underneath us
    for (i = 0; i < TI_MAX_RX_LENGTH; ++i) {
        tmp_rx_buf[i] = m->rx_buf[i];
    }

    m->last_serial_time = sched.os_ticks;

    // Parse Front Left Torque
    //if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "FL", &val_buf);
    //if (curr >= 0) m->Tx_in[0] = (uint16_t) val_buf;

    // Parse Front Right Torque
    //if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "FR", &val_buf);
    //if (curr >= 0) m->Tx_in[1] = (uint16_t) val_buf;

    // Parse Rear Left Torque
    //if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "RL", &val_buf);
    //if (curr >= 0) m->Tx_in[2] = val_buf;

    // Parse Rear Right Torque
    //if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "RR", &val_buf);
    //if (curr >= 0) m->Tx_in[3] = (uint8_t) val_buf;

    char Tx_temp[4];
    bool flag_FL = true;
    bool flag_FR = true;
    bool flag_RL = true;
    bool flag_RR = true;
    
    for (uint8_t i = 0; i < TI_MAX_RX_LENGTH; i++) 
    {
        if ((tmp_rx_buf[i] == 'F') && (tmp_rx_buf[i+1] == 'L') && flag_FL)
        {
            if (((tmp_rx_buf[i+2]) >= '0') && ((tmp_rx_buf[i+2]) <= '9') && ((tmp_rx_buf[i+3]) >= '0') && ((tmp_rx_buf[i+3]) <= '9') && ((tmp_rx_buf[i+4]) >= '0') && ((tmp_rx_buf[i+4]) <= '9') && ((tmp_rx_buf[i+5]) >= '0') && ((tmp_rx_buf[i+5]) <= '9'))
            {
                Tx_temp[0] = (tmp_rx_buf[i+2]);
                Tx_temp[1] = (tmp_rx_buf[i+3]);
                Tx_temp[2] = (tmp_rx_buf[i+4]);
                Tx_temp[3] = (tmp_rx_buf[i+5]);

                flag_FL = false;

                sscanf(Tx_temp, "%04d", &(m->Tx_in[0]));
            }
        }
        else if ((tmp_rx_buf[i] == 'F') && (tmp_rx_buf[i+1] == 'R')  && flag_FR)
        {
            if (((tmp_rx_buf[i+2]) >= '0') && ((tmp_rx_buf[i+2]) <= '9') && ((tmp_rx_buf[i+3]) >= '0') && ((tmp_rx_buf[i+3]) <= '9') && ((tmp_rx_buf[i+4]) >= '0') && ((tmp_rx_buf[i+4]) <= '9') && ((tmp_rx_buf[i+5]) >= '0') && ((tmp_rx_buf[i+5]) <= '9'))
            {
                Tx_temp[0] = (tmp_rx_buf[i+2]);
                Tx_temp[1] = (tmp_rx_buf[i+3]);
                Tx_temp[2] = (tmp_rx_buf[i+4]);
                Tx_temp[3] = (tmp_rx_buf[i+5]);

                flag_FR = false;

                sscanf(Tx_temp, "%04d", &(m->Tx_in[1]));
            }
        }
        else if ((tmp_rx_buf[i] == 'R') && (tmp_rx_buf[i+1] == 'L')  && flag_RL)
        {
            if (((tmp_rx_buf[i+2]) >= '0') && ((tmp_rx_buf[i+2]) <= '9') && ((tmp_rx_buf[i+3]) >= '0') && ((tmp_rx_buf[i+3]) <= '9') && ((tmp_rx_buf[i+4]) >= '0') && ((tmp_rx_buf[i+4]) <= '9') && ((tmp_rx_buf[i+5]) >= '0') && ((tmp_rx_buf[i+5]) <= '9'))
            {
                Tx_temp[0] = (tmp_rx_buf[i+2]);
                Tx_temp[1] = (tmp_rx_buf[i+3]);
                Tx_temp[2] = (tmp_rx_buf[i+4]);
                Tx_temp[3] = (tmp_rx_buf[i+5]);

                if ((tmp_rx_buf[i+3] == '0')  && (m->Tx_in[1] == 10))
                {
                    m->Tx_in[1] = 12;
                }

                flag_RL = false;

                sscanf(Tx_temp, "%04d", &(m->Tx_in[2]));
            }
        }
        else if ((tmp_rx_buf[i] == 'R') && (tmp_rx_buf[i+1] == 'R') && flag_RR)
        {   
            if (((tmp_rx_buf[i+2]) >= '0') && ((tmp_rx_buf[i+2]) <= '9') && ((tmp_rx_buf[i+3]) >= '0') && ((tmp_rx_buf[i+3]) <= '9') && ((tmp_rx_buf[i+4]) >= '0') && ((tmp_rx_buf[i+4]) <= '9') && ((tmp_rx_buf[i+5]) >= '0') && ((tmp_rx_buf[i+5]) <= '9'))
            {
                Tx_temp[0] = (tmp_rx_buf[i+2]);
                Tx_temp[1] = (tmp_rx_buf[i+3]);
                Tx_temp[2] = (tmp_rx_buf[i+4]);
                Tx_temp[3] = (tmp_rx_buf[i+5]);

                flag_RR = false;

                sscanf(Tx_temp, "%04d", &(m->Tx_in[3]));
            }
        }
    }
}
*/
/*
static int16_t tiParseTerm(char *rx_buf, uint8_t start, char *search_term, uint32_t *val_addr)
{
    uint8_t search_length = strlen(search_term);
    bool match = false;
    uint8_t curr = 0xFF;

    for (uint8_t i = start; i < TI_MAX_RX_LENGTH + start; ++i) 
    {
        if (rx_buf[i % TI_MAX_RX_LENGTH] == search_term[0])
        {
            match = true;
            // possible match, check entire term matches
            for (uint8_t j = 0; j < search_length; ++j)
            {
                if (rx_buf[(i + j) % TI_MAX_RX_LENGTH] != search_term[j])
                {
                    // not a match, continue searching
                    match = false;
                    break;
                }
            }
            if (match)
            {
                curr = i % TI_MAX_RX_LENGTH;
                break;
            }
        }
    }
    if (!match) return -1;
    // destroy match to prevent re-reading same data
    // rx_buf[curr] = '\0';
    curr = (curr + search_length) % TI_MAX_RX_LENGTH;

    uint32_t val = 0;

    // Extract value
    for (uint8_t i = curr; i < TI_MAX_RX_LENGTH + curr; ++i)
    {
        char c = rx_buf[i % TI_MAX_RX_LENGTH];
        if ((c == ' ' && val == 0) || c == '.') continue;
        else if (c >= '0' && c <= '9')
        {
            val = (val * 10) + (c - '0');
        }
        else
        {
            curr = i % TI_MAX_RX_LENGTH;
            break;
        }
    }

    *val_addr = val;
    return curr;
}
*/