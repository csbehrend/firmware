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

typedef struct
{
    uint8_t raw_message[100];
    uint8_t g_speed_bytes[4];
    signed long g_speed;
} GPS_Handle_t; // GPS handle

#endif //_GPS_H