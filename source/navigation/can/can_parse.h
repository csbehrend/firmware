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
#define NODE_NAME "Navigation"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_LATENCY_NAV 0xc0e3437
#define ID_GPS_FIX 0xc016b37
#define ID_GPS_VELOCITY 0xc0002b7
#define ID_GPS_POSITION 0xc002337
#define ID_GPS_COORDINATES 0xc002377
#define ID_IMU_GYRO 0xc0002f7
#define ID_IMU_ACCEL 0xc0023b7
#define ID_BMM_MAG 0xc0023f7
#define ID_SFS_POS 0xc016937
#define ID_SFS_VEL 0xc016977
#define ID_SFS_ACC 0xc0169b7
#define ID_SFS_ANG 0xc0169f7
#define ID_SFS_ANG_VEL 0xc016a37
#define ID_GPS_OOR 0xc016a77
#define ID_IMU_OOR 0xc016ab7
#define ID_BMM_OOR 0xc016af7
#define ID_LATENCY_MAIN 0xc823541
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_LATENCY_NAV 4
#define DLC_GPS_FIX 1
#define DLC_GPS_VELOCITY 8
#define DLC_GPS_POSITION 8
#define DLC_GPS_COORDINATES 8
#define DLC_IMU_GYRO 6
#define DLC_IMU_ACCEL 6
#define DLC_BMM_MAG 6
#define DLC_SFS_POS 6
#define DLC_SFS_VEL 6
#define DLC_SFS_ACC 6
#define DLC_SFS_ANG 8
#define DLC_SFS_ANG_VEL 6
#define DLC_GPS_OOR 1
#define DLC_IMU_OOR 1
#define DLC_BMM_OOR 1
#define DLC_LATENCY_MAIN 4
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_LATENCY_NAV(queue, latency_nav_out_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_LATENCY_NAV, .DLC=DLC_LATENCY_NAV, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->latency_nav.latency_nav_out = latency_nav_out_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_GPS_FIX(queue, gps_vel_n_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_FIX, .DLC=DLC_GPS_FIX, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_fix.gps_vel_n = gps_vel_n_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_GPS_VELOCITY(queue, gps_vel_n_, gps_vel_e_, gps_vel_d_, gps_vel_total_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_VELOCITY, .DLC=DLC_GPS_VELOCITY, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_velocity.gps_vel_n = gps_vel_n_;\
        data_a->gps_velocity.gps_vel_e = gps_vel_e_;\
        data_a->gps_velocity.gps_vel_d = gps_vel_d_;\
        data_a->gps_velocity.gps_vel_total = gps_vel_total_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_GPS_POSITION(queue, gps_pos_x_, gps_pos_y_, gps_pos_z_, height_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_POSITION, .DLC=DLC_GPS_POSITION, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_position.gps_pos_x = gps_pos_x_;\
        data_a->gps_position.gps_pos_y = gps_pos_y_;\
        data_a->gps_position.gps_pos_z = gps_pos_z_;\
        data_a->gps_position.height = height_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_GPS_COORDINATES(queue, latitude_, longitude_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_COORDINATES, .DLC=DLC_GPS_COORDINATES, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_coordinates.latitude = latitude_;\
        data_a->gps_coordinates.longitude = longitude_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_IMU_GYRO(queue, imu_gyro_x_, imu_gyro_y_, imu_gyro_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_IMU_GYRO, .DLC=DLC_IMU_GYRO, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->imu_gyro.imu_gyro_x = imu_gyro_x_;\
        data_a->imu_gyro.imu_gyro_y = imu_gyro_y_;\
        data_a->imu_gyro.imu_gyro_z = imu_gyro_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_IMU_ACCEL(queue, imu_accel_x_, imu_accel_y_, imu_accel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_IMU_ACCEL, .DLC=DLC_IMU_ACCEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->imu_accel.imu_accel_x = imu_accel_x_;\
        data_a->imu_accel.imu_accel_y = imu_accel_y_;\
        data_a->imu_accel.imu_accel_z = imu_accel_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_BMM_MAG(queue, bmm_mag_x_, bmm_mag_y_, bmm_mag_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_BMM_MAG, .DLC=DLC_BMM_MAG, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->bmm_mag.bmm_mag_x = bmm_mag_x_;\
        data_a->bmm_mag.bmm_mag_y = bmm_mag_y_;\
        data_a->bmm_mag.bmm_mag_z = bmm_mag_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_SFS_POS(queue, sfs_pos_x_, sfs_pos_y_, sfs_pos_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_POS, .DLC=DLC_SFS_POS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_pos.sfs_pos_x = sfs_pos_x_;\
        data_a->sfs_pos.sfs_pos_y = sfs_pos_y_;\
        data_a->sfs_pos.sfs_pos_z = sfs_pos_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_SFS_VEL(queue, sfs_vel_x_, sfs_vel_y_, sfs_vel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_VEL, .DLC=DLC_SFS_VEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_vel.sfs_vel_x = sfs_vel_x_;\
        data_a->sfs_vel.sfs_vel_y = sfs_vel_y_;\
        data_a->sfs_vel.sfs_vel_z = sfs_vel_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_SFS_ACC(queue, sfs_acc_x_, sfs_acc_y_, sfs_acc_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_ACC, .DLC=DLC_SFS_ACC, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_acc.sfs_acc_x = sfs_acc_x_;\
        data_a->sfs_acc.sfs_acc_y = sfs_acc_y_;\
        data_a->sfs_acc.sfs_acc_z = sfs_acc_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_SFS_ANG(queue, sfs_ang_a_, sfs_ang_b_, sfs_ang_c_, sfs_ang_d_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_ANG, .DLC=DLC_SFS_ANG, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_ang.sfs_ang_a = sfs_ang_a_;\
        data_a->sfs_ang.sfs_ang_b = sfs_ang_b_;\
        data_a->sfs_ang.sfs_ang_c = sfs_ang_c_;\
        data_a->sfs_ang.sfs_ang_d = sfs_ang_d_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_SFS_ANG_VEL(queue, sfs_ang_vel_x_, sfs_ang_vel_y_, sfs_ang_vel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_ANG_VEL, .DLC=DLC_SFS_ANG_VEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_ang_vel.sfs_ang_vel_x = sfs_ang_vel_x_;\
        data_a->sfs_ang_vel.sfs_ang_vel_y = sfs_ang_vel_y_;\
        data_a->sfs_ang_vel.sfs_ang_vel_z = sfs_ang_vel_z_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_GPS_OOR(queue, vel_oor_, pos_oor_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_OOR, .DLC=DLC_GPS_OOR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_oor.vel_oor = vel_oor_;\
        data_a->gps_oor.pos_oor = pos_oor_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_IMU_OOR(queue, acc_oor_, gyro_oor_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_IMU_OOR, .DLC=DLC_IMU_OOR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->imu_oor.acc_oor = acc_oor_;\
        data_a->imu_oor.gyro_oor = gyro_oor_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_BMM_OOR(queue, bmm_oor_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_BMM_OOR, .DLC=DLC_BMM_OOR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->bmm_oor.bmm_oor = bmm_oor_;\
        qSendToBack(&queue, &msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 3 / 2 // 3 / 2 would be 150% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_LATENCY_MAIN 15
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) \
    if (!stale &&                              \
        (curr - last) > period * STALE_THRESH) \
    stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t latency_nav_out: 32;
    } latency_nav;
    struct {
        uint64_t gps_vel_n: 8;
    } gps_fix;
    struct {
        uint64_t gps_vel_n: 16;
        uint64_t gps_vel_e: 16;
        uint64_t gps_vel_d: 16;
        uint64_t gps_vel_total: 16;
    } gps_velocity;
    struct {
        uint64_t gps_pos_x: 16;
        uint64_t gps_pos_y: 16;
        uint64_t gps_pos_z: 16;
        uint64_t height: 16;
    } gps_position;
    struct {
        uint64_t latitude: 32;
        uint64_t longitude: 32;
    } gps_coordinates;
    struct {
        uint64_t imu_gyro_x: 16;
        uint64_t imu_gyro_y: 16;
        uint64_t imu_gyro_z: 16;
    } imu_gyro;
    struct {
        uint64_t imu_accel_x: 16;
        uint64_t imu_accel_y: 16;
        uint64_t imu_accel_z: 16;
    } imu_accel;
    struct {
        uint64_t bmm_mag_x: 16;
        uint64_t bmm_mag_y: 16;
        uint64_t bmm_mag_z: 16;
    } bmm_mag;
    struct {
        uint64_t sfs_pos_x: 16;
        uint64_t sfs_pos_y: 16;
        uint64_t sfs_pos_z: 16;
    } sfs_pos;
    struct {
        uint64_t sfs_vel_x: 16;
        uint64_t sfs_vel_y: 16;
        uint64_t sfs_vel_z: 16;
    } sfs_vel;
    struct {
        uint64_t sfs_acc_x: 16;
        uint64_t sfs_acc_y: 16;
        uint64_t sfs_acc_z: 16;
    } sfs_acc;
    struct {
        uint64_t sfs_ang_a: 16;
        uint64_t sfs_ang_b: 16;
        uint64_t sfs_ang_c: 16;
        uint64_t sfs_ang_d: 16;
    } sfs_ang;
    struct {
        uint64_t sfs_ang_vel_x: 16;
        uint64_t sfs_ang_vel_y: 16;
        uint64_t sfs_ang_vel_z: 16;
    } sfs_ang_vel;
    struct {
        uint64_t vel_oor: 1;
        uint64_t pos_oor: 1;
    } gps_oor;
    struct {
        uint64_t acc_oor: 1;
        uint64_t gyro_oor: 1;
    } imu_oor;
    struct {
        uint64_t bmm_oor: 1;
    } bmm_oor;
    struct {
        uint64_t latency_main_out: 32;
    } latency_main;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint32_t latency_main_out;
        uint8_t stale;
        uint32_t last_rx;
    } latency_main;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void handleCallbacks(uint16_t id, bool latched);
extern void set_fault_daq(uint16_t id, bool value);
extern void return_fault_control(uint16_t id);
extern void send_fault(uint16_t id, bool latched);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 *
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(q_handle_t *q_rx_can_a);

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
void canProcessRxIRQs(CanMsgTypeDef_t *rx);

extern volatile uint32_t last_can_rx_time_ms;

#endif