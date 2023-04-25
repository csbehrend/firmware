/**
 * @file can_parse.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _CAN_PARSE_H_
#define _CAN_PARSE_H_

#include "common/queue/queue.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "Torque_Vector"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_TORQUE_REQUEST 0x4000042
#define ID_BITSTREAM_FLASH_STATUS 0x1902
#define ID_FAULT_SYNC_TORQUE_VECTOR 0x8ca42
#define ID_FRONT_WHEEL_DATA 0x4000003
#define ID_REAR_WHEEL_DATA 0x4000043
#define ID_BITSTREAM_DATA 0x400193e
#define ID_BITSTREAM_REQUEST 0x1000197e
#define ID_LWS_STANDARD 0x2b0
#define ID_ORION_CURRENTS_VOLTS 0x140006f8
#define ID_ORION_INFO 0x140006b8
#define ID_REAR_CONTROLLER_TEMPS 0xc000301
#define ID_FRONT_MOTOR_CURRENTS_TEMPS 0xc000283
#define ID_REAR_MOTOR_CURRENTS_TEMPS 0xc0002c1
#define ID_FILT_THROTTLE_BRAKE 0x4000245
#define ID_GPS_VELOCITY 0xc0002b7
#define ID_SFS_ANG_VEL 0xc016a37
#define ID_REAR_WHEEL_SPEEDS 0x8000381
#define ID_FAULT_SYNC_MAIN_MODULE 0x8ca01
#define ID_FAULT_SYNC_DRIVELINE 0x8ca83
#define ID_FAULT_SYNC_DASHBOARD 0x8cb05
#define ID_FAULT_SYNC_PRECHARGE 0x8cac4
#define ID_FAULT_SYNC_TEST_NODE 0x8cb7f
#define ID_SET_FAULT 0x809c83e
#define ID_RETURN_FAULT_CONTROL 0x809c87e
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_TORQUE_REQUEST 6
#define DLC_BITSTREAM_FLASH_STATUS 1
#define DLC_FAULT_SYNC_TORQUE_VECTOR 3
#define DLC_FRONT_WHEEL_DATA 8
#define DLC_REAR_WHEEL_DATA 8
#define DLC_BITSTREAM_DATA 8
#define DLC_BITSTREAM_REQUEST 5
#define DLC_LWS_STANDARD 5
#define DLC_ORION_CURRENTS_VOLTS 4
#define DLC_ORION_INFO 7
#define DLC_REAR_CONTROLLER_TEMPS 2
#define DLC_FRONT_MOTOR_CURRENTS_TEMPS 8
#define DLC_REAR_MOTOR_CURRENTS_TEMPS 8
#define DLC_FILT_THROTTLE_BRAKE 3
#define DLC_GPS_VELOCITY 8
#define DLC_SFS_ANG_VEL 6
#define DLC_REAR_WHEEL_SPEEDS 8
#define DLC_FAULT_SYNC_MAIN_MODULE 3
#define DLC_FAULT_SYNC_DRIVELINE 3
#define DLC_FAULT_SYNC_DASHBOARD 3
#define DLC_FAULT_SYNC_PRECHARGE 3
#define DLC_FAULT_SYNC_TEST_NODE 3
#define DLC_SET_FAULT 3
#define DLC_RETURN_FAULT_CONTROL 2
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_TORQUE_REQUEST(queue, front_left_, front_right_, rear_left_, rear_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TORQUE_REQUEST, .DLC=DLC_TORQUE_REQUEST, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->torque_request.front_left = front_left_;\
        data_a->torque_request.front_right = front_right_;\
        data_a->torque_request.rear_left = rear_left_;\
        data_a->torque_request.rear_right = rear_right_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_BITSTREAM_FLASH_STATUS(queue, flash_success_, flash_timeout_rx_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_BITSTREAM_FLASH_STATUS, .DLC=DLC_BITSTREAM_FLASH_STATUS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->bitstream_flash_status.flash_success = flash_success_;\
        data_a->bitstream_flash_status.flash_timeout_rx = flash_timeout_rx_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_FAULT_SYNC_TORQUE_VECTOR(queue, idx_, latched_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_FAULT_SYNC_TORQUE_VECTOR, .DLC=DLC_FAULT_SYNC_TORQUE_VECTOR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->fault_sync_torque_vector.idx = idx_;\
        data_a->fault_sync_torque_vector.latched = latched_;\
        qSendToBack(&queue, &msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 3 / 2 // 3 / 2 would be 150% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_FRONT_WHEEL_DATA 10
#define UP_REAR_WHEEL_DATA 10
#define UP_LWS_STANDARD 15
#define UP_ORION_CURRENTS_VOLTS 32
#define UP_ORION_INFO 32
#define UP_REAR_CONTROLLER_TEMPS 500
#define UP_FRONT_MOTOR_CURRENTS_TEMPS 500
#define UP_REAR_MOTOR_CURRENTS_TEMPS 500
#define UP_FILT_THROTTLE_BRAKE 15
#define UP_GPS_VELOCITY 40
#define UP_SFS_ANG_VEL 40
#define UP_REAR_WHEEL_SPEEDS 15
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t front_left: 12;
        uint64_t front_right: 12;
        uint64_t rear_left: 12;
        uint64_t rear_right: 12;
    } torque_request;
    struct {
        uint64_t flash_success: 1;
        uint64_t flash_timeout_rx: 1;
    } bitstream_flash_status;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_torque_vector;
    struct {
        uint64_t left_speed: 16;
        uint64_t right_speed: 16;
        uint64_t left_normal: 16;
        uint64_t right_normal: 16;
    } front_wheel_data;
    struct {
        uint64_t left_speed: 16;
        uint64_t right_speed: 16;
        uint64_t left_normal: 16;
        uint64_t right_normal: 16;
    } rear_wheel_data;
    struct {
        uint64_t d0: 8;
        uint64_t d1: 8;
        uint64_t d2: 8;
        uint64_t d3: 8;
        uint64_t d4: 8;
        uint64_t d5: 8;
        uint64_t d6: 8;
        uint64_t d7: 8;
    } bitstream_data;
    struct {
        uint64_t download_request: 1;
        uint64_t download_size: 32;
    } bitstream_request;
    struct {
        uint64_t LWS_ANGLE: 16;
        uint64_t LWS_SPEED: 8;
        uint64_t Ok: 1;
        uint64_t Cal: 1;
        uint64_t Trim: 1;
        uint64_t Reserved_1: 5;
        uint64_t Reserved_2: 8;
    } LWS_Standard;
    struct {
        uint64_t pack_current: 16;
        uint64_t pack_voltage: 16;
    } orion_currents_volts;
    struct {
        uint64_t discharge_enable: 1;
        uint64_t charge_enable: 1;
        uint64_t charger_safety: 1;
        uint64_t dtc_status: 1;
        uint64_t multi_input: 1;
        uint64_t always_on: 1;
        uint64_t is_ready: 1;
        uint64_t is_charging: 1;
        uint64_t multi_input_2: 1;
        uint64_t multi_input_3: 1;
        uint64_t reserved: 1;
        uint64_t multi_output_2: 1;
        uint64_t multi_output_3: 1;
        uint64_t multi_output_4: 1;
        uint64_t multi_enable: 1;
        uint64_t multi_output_1: 1;
        uint64_t pack_dcl: 16;
        uint64_t pack_ccl: 16;
        uint64_t pack_soc: 8;
    } orion_info;
    struct {
        uint64_t left_temp: 8;
        uint64_t right_temp: 8;
    } rear_controller_temps;
    struct {
        uint64_t left_current: 16;
        uint64_t right_current: 16;
        uint64_t left_temp: 8;
        uint64_t right_temp: 8;
        uint64_t right_voltage: 16;
    } front_motor_currents_temps;
    struct {
        uint64_t left_current: 16;
        uint64_t right_current: 16;
        uint64_t left_temp: 8;
        uint64_t right_temp: 8;
        uint64_t right_voltage: 16;
    } rear_motor_currents_temps;
    struct {
        uint64_t throttle: 12;
        uint64_t brake: 12;
    } filt_throttle_brake;
    struct {
        uint64_t gps_vel_n: 16;
        uint64_t gps_vel_e: 16;
        uint64_t gps_vel_d: 16;
        uint64_t gps_vel_total: 16;
    } gps_velocity;
    struct {
        uint64_t sfs_ang_vel_x: 16;
        uint64_t sfs_ang_vel_y: 16;
        uint64_t sfs_ang_vel_z: 16;
    } sfs_ang_vel;
    struct {
        uint64_t left_speed_mc: 16;
        uint64_t right_speed_mc: 16;
        uint64_t left_speed_sensor: 16;
        uint64_t right_speed_sensor: 16;
    } rear_wheel_speeds;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_main_module;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_driveline;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_dashboard;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_precharge;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_test_node;
    struct {
        uint64_t id: 16;
        uint64_t value: 1;
    } set_fault;
    struct {
        uint64_t id: 16;
    } return_fault_control;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint16_t left_speed;
        uint16_t right_speed;
        uint16_t left_normal;
        uint16_t right_normal;
        uint8_t stale;
        uint32_t last_rx;
    } front_wheel_data;
    struct {
        uint16_t left_speed;
        uint16_t right_speed;
        uint16_t left_normal;
        uint16_t right_normal;
        uint8_t stale;
        uint32_t last_rx;
    } rear_wheel_data;
    struct {
        uint8_t d0;
        uint8_t d1;
        uint8_t d2;
        uint8_t d3;
        uint8_t d4;
        uint8_t d5;
        uint8_t d6;
        uint8_t d7;
    } bitstream_data;
    struct {
        uint8_t download_request;
        uint32_t download_size;
    } bitstream_request;
    struct {
        int16_t LWS_ANGLE;
        uint8_t LWS_SPEED;
        uint8_t Ok;
        uint8_t Cal;
        uint8_t Trim;
        uint8_t Reserved_1;
        uint8_t Reserved_2;
        uint8_t stale;
        uint32_t last_rx;
    } LWS_Standard;
    struct {
        int16_t pack_current;
        uint16_t pack_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } orion_currents_volts;
    struct {
        uint8_t discharge_enable;
        uint8_t charge_enable;
        uint8_t charger_safety;
        uint8_t dtc_status;
        uint8_t multi_input;
        uint8_t always_on;
        uint8_t is_ready;
        uint8_t is_charging;
        uint8_t multi_input_2;
        uint8_t multi_input_3;
        uint8_t reserved;
        uint8_t multi_output_2;
        uint8_t multi_output_3;
        uint8_t multi_output_4;
        uint8_t multi_enable;
        uint8_t multi_output_1;
        uint16_t pack_dcl;
        uint16_t pack_ccl;
        uint8_t pack_soc;
        uint8_t stale;
        uint32_t last_rx;
    } orion_info;
    struct {
        uint8_t left_temp;
        uint8_t right_temp;
        uint8_t stale;
        uint32_t last_rx;
    } rear_controller_temps;
    struct {
        uint16_t left_current;
        uint16_t right_current;
        uint8_t left_temp;
        uint8_t right_temp;
        uint16_t right_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } front_motor_currents_temps;
    struct {
        uint16_t left_current;
        uint16_t right_current;
        uint8_t left_temp;
        uint8_t right_temp;
        uint16_t right_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } rear_motor_currents_temps;
    struct {
        uint16_t throttle;
        uint16_t brake;
        uint8_t stale;
        uint32_t last_rx;
    } filt_throttle_brake;
    struct {
        int16_t gps_vel_n;
        int16_t gps_vel_e;
        int16_t gps_vel_d;
        int16_t gps_vel_total;
        uint8_t stale;
        uint32_t last_rx;
    } gps_velocity;
    struct {
        int16_t sfs_ang_vel_x;
        int16_t sfs_ang_vel_y;
        int16_t sfs_ang_vel_z;
        uint8_t stale;
        uint32_t last_rx;
    } sfs_ang_vel;
    struct {
        uint16_t left_speed_mc;
        uint16_t right_speed_mc;
        uint16_t left_speed_sensor;
        uint16_t right_speed_sensor;
        uint8_t stale;
        uint32_t last_rx;
    } rear_wheel_speeds;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_main_module;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_driveline;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_dashboard;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_precharge;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_test_node;
    struct {
        uint16_t id;
        uint8_t value;
    } set_fault;
    struct {
        uint16_t id;
    } return_fault_control;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void bitstream_request_CALLBACK(CanParsedData_t* msg_data_a);
extern void handleCallbacks(uint16_t id, bool latched);
extern void set_fault_daq(uint16_t id, bool value);
extern void return_fault_control(uint16_t id);
extern void send_fault(uint16_t id, bool latched);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
extern void bitstream_data_IRQ(CanParsedData_t* msg_data_a);
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 * 
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(q_handle_t* q_rx_can_a);

/**
 * @brief Pull message off of rx buffer,
 *        update can_data struct,
 *        check for stale messages
 */
void canRxUpdate();

/**
 * @brief Process any rx message callbacks from the CAN Rx IRQ
 * 
 * @param rx rx data from message just recieved
 */
void canProcessRxIRQs(CanMsgTypeDef_t* rx);

#endif