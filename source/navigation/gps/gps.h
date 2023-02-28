// /**
//  * @file gps.h
//  * @author Chris McGalliard (cmcgalli@purdue.edu)
//  * @brief
//  * @version 0.1
//  * @date 2022-12-28
//  *
//  *
//  */

#include <stdint.h>

#ifndef _GPS_H
#define _GPS_H

union i_Long
{
    uint8_t bytes[4];
    signed long iLong;
};

union i_Short
{
    uint8_t bytes[2];
    signed short iShort;
};

typedef struct
{
    uint8_t raw_message[100];
    uint8_t g_speed_bytes[4];
    signed long g_speed;
    uint8_t longitude_bytes[4];
    signed long longitude;
    uint8_t latitude_bytes[4];
    signed long latitude;
    uint8_t height_bytes[4];
    signed long height;
    uint8_t n_vel_bytes[4];
    signed long n_vel;
    uint8_t e_vel_bytes[4];
    signed long e_vel;
    uint8_t d_vel_bytes[4];
    signed long d_vel;
    uint8_t mag_dec_bytes[2];
    signed short mag_dec;
} GPS_Handle_t; // GPS handle

bool parseVelocity(GPS_Handle_t *GPS);

#endif //_GPS_H