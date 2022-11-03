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
        msg_data_a = (CanParsedData_t *) &msg_header.Data;
        last_can_rx_time_ms = sched.os_ticks;
        if (msg_header.IDE == 0 && msg_header.StdId == ID_LWS_STANDARD)
        {
            can_data.LWS_Standard.LWS_ANGLE = (int16_t) msg_data_a->LWS_Standard.LWS_ANGLE;
            can_data.LWS_Standard.LWS_SPEED = msg_data_a->LWS_Standard.LWS_SPEED;
            can_data.LWS_Standard.Ok = msg_data_a->LWS_Standard.Ok;
            can_data.LWS_Standard.Cal = msg_data_a->LWS_Standard.Cal;
            can_data.LWS_Standard.Trim = msg_data_a->LWS_Standard.Trim;
            can_data.LWS_Standard.Reserved_1 = msg_data_a->LWS_Standard.Reserved_1;
            can_data.LWS_Standard.Reserved_2 = msg_data_a->LWS_Standard.Reserved_2;
            can_data.LWS_Standard.stale = 0;
            can_data.LWS_Standard.last_rx = sched.os_ticks;
        }
        else
        
        {

        /* BEGIN AUTO CASES */
        switch(msg_header.ExtId)
        {
            case ID_RAW_THROTTLE_BRAKE:
                can_data.raw_throttle_brake.throttle = msg_data_a->raw_throttle_brake.throttle;
                can_data.raw_throttle_brake.brake = msg_data_a->raw_throttle_brake.brake;
                can_data.raw_throttle_brake.stale = 0;
                can_data.raw_throttle_brake.last_rx = sched.os_ticks;
                break;
            case ID_START_BUTTON:
                can_data.start_button.start = msg_data_a->start_button.start;
                break;
            case ID_FRONT_MOTOR_CURRENTS_TEMPS:
                can_data.front_motor_currents_temps.left_current = msg_data_a->front_motor_currents_temps.left_current;
                can_data.front_motor_currents_temps.right_current = msg_data_a->front_motor_currents_temps.right_current;
                can_data.front_motor_currents_temps.left_temp = msg_data_a->front_motor_currents_temps.left_temp;
                can_data.front_motor_currents_temps.right_temp = msg_data_a->front_motor_currents_temps.right_temp;
                can_data.front_motor_currents_temps.right_voltage = msg_data_a->front_motor_currents_temps.right_voltage;
                can_data.front_motor_currents_temps.stale = 0;
                can_data.front_motor_currents_temps.last_rx = sched.os_ticks;
                break;
            case ID_REAR_MOTOR_CURRENTS_TEMPS:
                can_data.rear_motor_currents_temps.left_current = msg_data_a->rear_motor_currents_temps.left_current;
                can_data.rear_motor_currents_temps.right_current = msg_data_a->rear_motor_currents_temps.right_current;
                can_data.rear_motor_currents_temps.left_temp = msg_data_a->rear_motor_currents_temps.left_temp;
                can_data.rear_motor_currents_temps.right_temp = msg_data_a->rear_motor_currents_temps.right_temp;
                can_data.rear_motor_currents_temps.right_voltage = msg_data_a->rear_motor_currents_temps.right_voltage;
                can_data.rear_motor_currents_temps.stale = 0;
                can_data.rear_motor_currents_temps.last_rx = sched.os_ticks;
                break;
            case ID_FRONT_DRIVELINE_HB:
                can_data.front_driveline_hb.front_left_motor = msg_data_a->front_driveline_hb.front_left_motor;
                can_data.front_driveline_hb.front_left_motor_link = msg_data_a->front_driveline_hb.front_left_motor_link;
                can_data.front_driveline_hb.front_left_last_link_error = msg_data_a->front_driveline_hb.front_left_last_link_error;
                can_data.front_driveline_hb.front_right_motor = msg_data_a->front_driveline_hb.front_right_motor;
                can_data.front_driveline_hb.front_right_motor_link = msg_data_a->front_driveline_hb.front_right_motor_link;
                can_data.front_driveline_hb.front_right_last_link_error = msg_data_a->front_driveline_hb.front_right_last_link_error;
                can_data.front_driveline_hb.stale = 0;
                can_data.front_driveline_hb.last_rx = sched.os_ticks;
                break;
            case ID_REAR_DRIVELINE_HB:
                can_data.rear_driveline_hb.rear_left_motor = msg_data_a->rear_driveline_hb.rear_left_motor;
                can_data.rear_driveline_hb.rear_left_motor_link = msg_data_a->rear_driveline_hb.rear_left_motor_link;
                can_data.rear_driveline_hb.rear_left_last_link_error = msg_data_a->rear_driveline_hb.rear_left_last_link_error;
                can_data.rear_driveline_hb.rear_right_motor = msg_data_a->rear_driveline_hb.rear_right_motor;
                can_data.rear_driveline_hb.rear_right_motor_link = msg_data_a->rear_driveline_hb.rear_right_motor_link;
                can_data.rear_driveline_hb.rear_right_last_link_error = msg_data_a->rear_driveline_hb.rear_right_last_link_error;
                can_data.rear_driveline_hb.stale = 0;
                can_data.rear_driveline_hb.last_rx = sched.os_ticks;
                break;
            case ID_DASHBOARD_HB:
                can_data.dashboard_hb.apps_faulted = msg_data_a->dashboard_hb.apps_faulted;
                can_data.dashboard_hb.bse_faulted = msg_data_a->dashboard_hb.bse_faulted;
                can_data.dashboard_hb.apps_brake_faulted = msg_data_a->dashboard_hb.apps_brake_faulted;
                can_data.dashboard_hb.stale = 0;
                can_data.dashboard_hb.last_rx = sched.os_ticks;
                break;
            case ID_MAX_CELL_TEMP:
                can_data.max_cell_temp.max_temp = msg_data_a->max_cell_temp.max_temp;
                break;
            case ID_FRONT_WHEEL_DATA:
                can_data.front_wheel_data.left_speed = msg_data_a->front_wheel_data.left_speed;
                can_data.front_wheel_data.right_speed = msg_data_a->front_wheel_data.right_speed;
                can_data.front_wheel_data.left_normal = msg_data_a->front_wheel_data.left_normal;
                can_data.front_wheel_data.right_normal = msg_data_a->front_wheel_data.right_normal;
                can_data.front_wheel_data.stale = 0;
                can_data.front_wheel_data.last_rx = sched.os_ticks;
                break;
            case ID_REAR_WHEEL_DATA:
                can_data.rear_wheel_data.left_speed = msg_data_a->rear_wheel_data.left_speed;
                can_data.rear_wheel_data.right_speed = msg_data_a->rear_wheel_data.right_speed;
                can_data.rear_wheel_data.left_normal = msg_data_a->rear_wheel_data.left_normal;
                can_data.rear_wheel_data.right_normal = msg_data_a->rear_wheel_data.right_normal;
                can_data.rear_wheel_data.stale = 0;
                can_data.rear_wheel_data.last_rx = sched.os_ticks;
                break;
            case ID_LWS_STANDARD:
                can_data.LWS_Standard.LWS_ANGLE = (int16_t) msg_data_a->LWS_Standard.LWS_ANGLE;
                can_data.LWS_Standard.LWS_SPEED = msg_data_a->LWS_Standard.LWS_SPEED;
                can_data.LWS_Standard.Ok = msg_data_a->LWS_Standard.Ok;
                can_data.LWS_Standard.Cal = msg_data_a->LWS_Standard.Cal;
                can_data.LWS_Standard.Trim = msg_data_a->LWS_Standard.Trim;
                can_data.LWS_Standard.Reserved_1 = msg_data_a->LWS_Standard.Reserved_1;
                can_data.LWS_Standard.Reserved_2 = msg_data_a->LWS_Standard.Reserved_2;
                can_data.LWS_Standard.stale = 0;
                can_data.LWS_Standard.last_rx = sched.os_ticks;
                break;
            case ID_ORION_CURRENTS_VOLTS:
                can_data.orion_currents_volts.pack_current = (int16_t) msg_data_a->orion_currents_volts.pack_current;
                can_data.orion_currents_volts.pack_voltage = msg_data_a->orion_currents_volts.pack_voltage;
                can_data.orion_currents_volts.stale = 0;
                can_data.orion_currents_volts.last_rx = sched.os_ticks;
                break;
            case ID_ORION_INFO:
                can_data.orion_info.discharge_enable = msg_data_a->orion_info.discharge_enable;
                can_data.orion_info.charge_enable = msg_data_a->orion_info.charge_enable;
                can_data.orion_info.charger_safety = msg_data_a->orion_info.charger_safety;
                can_data.orion_info.dtc_status = msg_data_a->orion_info.dtc_status;
                can_data.orion_info.multi_input = msg_data_a->orion_info.multi_input;
                can_data.orion_info.always_on = msg_data_a->orion_info.always_on;
                can_data.orion_info.is_ready = msg_data_a->orion_info.is_ready;
                can_data.orion_info.is_charging = msg_data_a->orion_info.is_charging;
                can_data.orion_info.multi_input_2 = msg_data_a->orion_info.multi_input_2;
                can_data.orion_info.multi_input_3 = msg_data_a->orion_info.multi_input_3;
                can_data.orion_info.reserved = msg_data_a->orion_info.reserved;
                can_data.orion_info.multi_output_2 = msg_data_a->orion_info.multi_output_2;
                can_data.orion_info.multi_output_3 = msg_data_a->orion_info.multi_output_3;
                can_data.orion_info.multi_output_4 = msg_data_a->orion_info.multi_output_4;
                can_data.orion_info.multi_enable = msg_data_a->orion_info.multi_enable;
                can_data.orion_info.multi_output_1 = msg_data_a->orion_info.multi_output_1;
                can_data.orion_info.pack_dcl = msg_data_a->orion_info.pack_dcl;
                can_data.orion_info.pack_ccl = msg_data_a->orion_info.pack_ccl;
                can_data.orion_info.pack_soc = msg_data_a->orion_info.pack_soc;
                can_data.orion_info.stale = 0;
                can_data.orion_info.last_rx = sched.os_ticks;
                break;
            case ID_GYRO_DATA:
                can_data.gyro_data.gx = (int16_t) msg_data_a->gyro_data.gx;
                can_data.gyro_data.gy = (int16_t) msg_data_a->gyro_data.gy;
                can_data.gyro_data.gz = (int16_t) msg_data_a->gyro_data.gz;
                can_data.gyro_data.stale = 0;
                can_data.gyro_data.last_rx = sched.os_ticks;
                break;
            case ID_ACCEL_DATA:
                can_data.accel_data.ax = (int16_t) msg_data_a->accel_data.ax;
                can_data.accel_data.ay = (int16_t) msg_data_a->accel_data.ay;
                can_data.accel_data.az = (int16_t) msg_data_a->accel_data.az;
                can_data.accel_data.stale = 0;
                can_data.accel_data.last_rx = sched.os_ticks;
                break;
            case ID_MAIN_MODULE_BL_CMD:
                can_data.main_module_bl_cmd.cmd = msg_data_a->main_module_bl_cmd.cmd;
                can_data.main_module_bl_cmd.data = msg_data_a->main_module_bl_cmd.data;
                main_module_bl_cmd_CALLBACK(msg_data_a);
                break;
            case ID_DAQ_COMMAND_MAIN_MODULE:
                can_data.daq_command_MAIN_MODULE.daq_command = msg_data_a->daq_command_MAIN_MODULE.daq_command;
                daq_command_MAIN_MODULE_CALLBACK(&msg_header);
                break;
            default:
                __asm__("nop");
        }
        /* END AUTO CASES */
        }
    }

    /* BEGIN AUTO STALE CHECKS */
    CHECK_STALE(can_data.raw_throttle_brake.stale,
                sched.os_ticks, can_data.raw_throttle_brake.last_rx,
                UP_RAW_THROTTLE_BRAKE);
    CHECK_STALE(can_data.front_motor_currents_temps.stale,
                sched.os_ticks, can_data.front_motor_currents_temps.last_rx,
                UP_FRONT_MOTOR_CURRENTS_TEMPS);
    CHECK_STALE(can_data.rear_motor_currents_temps.stale,
                sched.os_ticks, can_data.rear_motor_currents_temps.last_rx,
                UP_REAR_MOTOR_CURRENTS_TEMPS);
    CHECK_STALE(can_data.front_driveline_hb.stale,
                sched.os_ticks, can_data.front_driveline_hb.last_rx,
                UP_FRONT_DRIVELINE_HB);
    CHECK_STALE(can_data.rear_driveline_hb.stale,
                sched.os_ticks, can_data.rear_driveline_hb.last_rx,
                UP_REAR_DRIVELINE_HB);
    CHECK_STALE(can_data.dashboard_hb.stale,
                sched.os_ticks, can_data.dashboard_hb.last_rx,
                UP_DASHBOARD_HB);
    CHECK_STALE(can_data.front_wheel_data.stale,
                sched.os_ticks, can_data.front_wheel_data.last_rx,
                UP_FRONT_WHEEL_DATA);
    CHECK_STALE(can_data.rear_wheel_data.stale,
                sched.os_ticks, can_data.rear_wheel_data.last_rx,
                UP_REAR_WHEEL_DATA);
    CHECK_STALE(can_data.LWS_Standard.stale,
                sched.os_ticks, can_data.LWS_Standard.last_rx,
                UP_LWS_STANDARD);
    CHECK_STALE(can_data.orion_currents_volts.stale,
                sched.os_ticks, can_data.orion_currents_volts.last_rx,
                UP_ORION_CURRENTS_VOLTS);
    CHECK_STALE(can_data.orion_info.stale,
                sched.os_ticks, can_data.orion_info.last_rx,
                UP_ORION_INFO);
    CHECK_STALE(can_data.gyro_data.stale,
                sched.os_ticks, can_data.gyro_data.last_rx,
                UP_GYRO_DATA);
    CHECK_STALE(can_data.accel_data.stale,
                sched.os_ticks, can_data.accel_data.last_rx,
                UP_ACCEL_DATA);
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
    CAN1->sFilterRegister[0].FR1 = (ID_RAW_THROTTLE_BRAKE << 3) | 4;
    CAN1->sFilterRegister[0].FR2 = (ID_START_BUTTON << 3) | 4;
    CAN1->FA1R |= (1 << 1);    // configure bank 1
    CAN1->sFilterRegister[1].FR1 = (ID_FRONT_MOTOR_CURRENTS_TEMPS << 3) | 4;
    CAN1->sFilterRegister[1].FR2 = (ID_REAR_MOTOR_CURRENTS_TEMPS << 3) | 4;
    CAN1->FA1R |= (1 << 2);    // configure bank 2
    CAN1->sFilterRegister[2].FR1 = (ID_FRONT_DRIVELINE_HB << 3) | 4;
    CAN1->sFilterRegister[2].FR2 = (ID_REAR_DRIVELINE_HB << 3) | 4;
    CAN1->FA1R |= (1 << 3);    // configure bank 3
    CAN1->sFilterRegister[3].FR1 = (ID_DASHBOARD_HB << 3) | 4;
    CAN1->sFilterRegister[3].FR2 = (ID_MAX_CELL_TEMP << 3) | 4;
    CAN1->FA1R |= (1 << 4);    // configure bank 4
    CAN1->sFilterRegister[4].FR1 = (ID_FRONT_WHEEL_DATA << 3) | 4;
    CAN1->sFilterRegister[4].FR2 = (ID_REAR_WHEEL_DATA << 3) | 4;
    CAN1->FA1R |= (1 << 5);    // configure bank 5
    CAN1->sFilterRegister[5].FR1 = (ID_LWS_STANDARD << 3) | 4;
    CAN1->sFilterRegister[5].FR2 = (ID_ORION_CURRENTS_VOLTS << 3) | 4;
    CAN1->FA1R |= (1 << 6);    // configure bank 6
    CAN1->sFilterRegister[6].FR1 = (ID_ORION_INFO << 3) | 4;
    CAN1->sFilterRegister[6].FR2 = (ID_GYRO_DATA << 3) | 4;
    CAN1->FA1R |= (1 << 7);    // configure bank 7
    CAN1->sFilterRegister[7].FR1 = (ID_ACCEL_DATA << 3) | 4;
    CAN1->sFilterRegister[7].FR2 = (ID_MAIN_MODULE_BL_CMD << 3) | 4;
    CAN1->FA1R |= (1 << 8);    // configure bank 8
    CAN1->sFilterRegister[8].FR1 = (ID_DAQ_COMMAND_MAIN_MODULE << 3) | 4;
    /* END AUTO FILTER */
    CAN1->FA1R |= (1 << 6);    // configure bank 6
    CAN1->sFilterRegister[6].FR1 = (ID_LWS_STANDARD << 21);

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
