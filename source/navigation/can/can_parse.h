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
#define ID_GPS_VELOCITY 0xc0002b7
#define ID_GPS_POSITION 0xc002337
#define ID_GPS_COORDINATES 0xc002377
#define ID_IMU_GYRO 0xc0002f7
#define ID_IMU_ACCEL 0xc0023b7
#define ID_BMM_MAG 0xc0023f7
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_GPS_VELOCITY 8
#define DLC_GPS_POSITION 8
#define DLC_GPS_COORDINATES 8
#define DLC_IMU_GYRO 6
#define DLC_IMU_ACCEL 6
#define DLC_BMM_MAG 6
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
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
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 3 / 2 // 3 / 2 would be 150% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
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
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
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