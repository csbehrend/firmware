/**
 * @file can_parse.c
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "can_parse.h"

// prototypes
bool initCANFilter();

can_data_t can_data;
q_handle_t* q_rx_can_a;
volatile uint32_t last_can_rx_time_ms = 0;

void initCANParse(q_handle_t* rx_a)
{
    q_rx_can_a = rx_a;
    initCANFilter();
}

void canRxUpdate()
{
    CanMsgTypeDef_t msg_header;
    CanParsedData_t* msg_data_a;

    if(qReceive(q_rx_can_a, &msg_header) == SUCCESS_G)
    {
        last_can_rx_time_ms = sched.os_ticks;
        msg_data_a = (CanParsedData_t *) &msg_header.Data;
        /* BEGIN AUTO CASES */
        switch(msg_header.ExtId)
        {
            case ID_MAIN_HB:
                can_data.main_hb.car_state = msg_data_a->main_hb.car_state;
                can_data.main_hb.precharge_state = msg_data_a->main_hb.precharge_state;
                can_data.main_hb.stale = 0;
                can_data.main_hb.last_rx = sched.os_ticks;
                break;
            case ID_REAR_WHEEL_DATA:
                can_data.rear_wheel_data.left_speed = msg_data_a->rear_wheel_data.left_speed;
                can_data.rear_wheel_data.right_speed = msg_data_a->rear_wheel_data.right_speed;
                can_data.rear_wheel_data.left_normal = msg_data_a->rear_wheel_data.left_normal;
                can_data.rear_wheel_data.right_normal = msg_data_a->rear_wheel_data.right_normal;
                can_data.rear_wheel_data.stale = 0;
                can_data.rear_wheel_data.last_rx = sched.os_ticks;
                break;
            case ID_REAR_MOTOR_CURRENTS_TEMPS:
                can_data.rear_motor_currents_temps.left_current = msg_data_a->rear_motor_currents_temps.left_current;
                can_data.rear_motor_currents_temps.right_current = msg_data_a->rear_motor_currents_temps.right_current;
                can_data.rear_motor_currents_temps.left_temp = msg_data_a->rear_motor_currents_temps.left_temp;
                can_data.rear_motor_currents_temps.right_temp = msg_data_a->rear_motor_currents_temps.right_temp;
                can_data.rear_motor_currents_temps.stale = 0;
                can_data.rear_motor_currents_temps.last_rx = sched.os_ticks;
                break;
            case ID_DAQ_COMMAND_DASHBOARD:
                can_data.daq_command_DASHBOARD.daq_command = msg_data_a->daq_command_DASHBOARD.daq_command;
                daq_command_DASHBOARD_CALLBACK(&msg_header);
                break;
            default:
                __asm__("nop");
        }
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    CHECK_STALE(can_data.main_hb.stale,
                sched.os_ticks, can_data.main_hb.last_rx,
                UP_MAIN_HB);
    CHECK_STALE(can_data.rear_wheel_data.stale,
                sched.os_ticks, can_data.rear_wheel_data.last_rx,
                UP_REAR_WHEEL_DATA);
    CHECK_STALE(can_data.rear_motor_currents_temps.stale,
                sched.os_ticks, can_data.rear_motor_currents_temps.last_rx,
                UP_REAR_MOTOR_CURRENTS_TEMPS);
    /* END AUTO STALE CHECKS */
}

bool initCANFilter()
{
    CAN1->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    uint32_t timeout = 0;
    while(!(CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout == PHAL_CAN_INIT_TIMEOUT)
         return false;

    CAN1->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN1->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN1->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale

    /* BEGIN AUTO FILTER */
    CAN1->FA1R |= (1 << 0);    // configure bank 0
    CAN1->sFilterRegister[0].FR1 = (ID_MAIN_HB << 3) | 4;
    CAN1->sFilterRegister[0].FR2 = (ID_REAR_WHEEL_DATA << 3) | 4;
    CAN1->FA1R |= (1 << 1);    // configure bank 1
    CAN1->sFilterRegister[1].FR1 = (ID_REAR_MOTOR_CURRENTS_TEMPS << 3) | 4;
    CAN1->sFilterRegister[1].FR2 = (ID_DAQ_COMMAND_DASHBOARD << 3) | 4;
    /* END AUTO FILTER */

    CAN1->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)

    // Enter back into NORMAL mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;

    return timeout != PHAL_CAN_INIT_TIMEOUT;
}


void canProcessRxIRQs(CanMsgTypeDef_t* rx)
{
    CanParsedData_t* msg_data_a;

    msg_data_a = (CanParsedData_t *) rx->Data;
    switch(rx->ExtId)
    {
        /* BEGIN AUTO RX IRQ */
        /* END AUTO RX IRQ */
        default:
            __asm__("nop");
    }
}
