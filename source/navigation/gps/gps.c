#include <stdio.h>
#include <stdbool.h>
#include "gps.h"
union i_Long iLong;
union i_Short iShort;

// Test Nav Message
GPS_Handle_t gps_handle = {.raw_message = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0x80, 0x10, 0xC1, 0x08, 0xE7, 0x07, 0x01, 0x02, 0x10, 0x2F, 0x20, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xB0, 0xB1, 0xD8, 0x17, 0x03, 0x01, 0xEA, 0x05, 0x32, 0x3B, 0xEC, 0xCB, 0x92, 0x1D, 0xA7, 0x16, 0xBF, 0xB6, 0x01, 0x00, 0xD2, 0x34, 0x02, 0x00, 0x85, 0x0D, 0x00, 0x00, 0x8A, 0x29, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x01, 0x00, 0x00, 0x72, 0x1A, 0xFC, 0x00, 0x6B, 0x01, 0x00, 0x00, 0xEE, 0x13, 0x4F, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x97},
                           .g_speed = 0,
                           .g_speed_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .longitude_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .longitude = 0,
                           .latitude_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .latitude = 0,
                           .height_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .height = 0,
                           .n_vel_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .n_vel = 0,
                           .e_vel_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .e_vel = 0,
                           .d_vel_bytes = {0xFF, 0xFF, 0xFF, 0xFF},
                           .d_vel = 0,
                           .mag_dec_bytes = {0xFF, 0xFF},
                           .mag_dec = 0};

bool parseVelocity(uint8_t *raw_message, GPS_Handle_t *GPS);

// Parse velocity from raw message
bool parseVelocity(uint8_t *raw_message, GPS_Handle_t *GPS)
{
    // Validate the message header, class, and id
    if ((raw_message[0] != 0xB5) && (raw_message[1] != 0x62) && (raw_message[2] != 0x01) && (raw_message[3] != 0x07))
    {
        printf("message failure\n");
        return false;
    }
    else
    {
        // Collect Ground Speed
        GPS->g_speed_bytes[0] = raw_message[66];
        GPS->g_speed_bytes[1] = raw_message[67];
        GPS->g_speed_bytes[2] = raw_message[68];
        GPS->g_speed_bytes[3] = raw_message[69];
        iLong.bytes[0] = raw_message[66];
        iLong.bytes[1] = raw_message[67];
        iLong.bytes[2] = raw_message[68];
        iLong.bytes[3] = raw_message[69];
        GPS->g_speed = iLong.iLong;

        // Collect Longitude
        GPS->longitude_bytes[0] = raw_message[30];
        GPS->longitude_bytes[0] = raw_message[31];
        GPS->longitude_bytes[0] = raw_message[32];
        GPS->longitude_bytes[0] = raw_message[33];
        iLong.bytes[0] = raw_message[30];
        iLong.bytes[1] = raw_message[31];
        iLong.bytes[2] = raw_message[32];
        iLong.bytes[3] = raw_message[33];
        GPS->longitude = iLong.iLong;

        // Colect Latitude
        GPS->latitude_bytes[0] = raw_message[34];
        GPS->latitude_bytes[0] = raw_message[35];
        GPS->latitude_bytes[0] = raw_message[36];
        GPS->latitude_bytes[0] = raw_message[37];
        iLong.bytes[0] = raw_message[34];
        iLong.bytes[1] = raw_message[35];
        iLong.bytes[2] = raw_message[36];
        iLong.bytes[3] = raw_message[37];
        GPS->latitude = iLong.iLong;

        // Collect Height
        GPS->height_bytes[0] = raw_message[42];
        GPS->height_bytes[0] = raw_message[43];
        GPS->height_bytes[0] = raw_message[44];
        GPS->height_bytes[0] = raw_message[45];
        iLong.bytes[0] = raw_message[42];
        iLong.bytes[1] = raw_message[43];
        iLong.bytes[2] = raw_message[44];
        iLong.bytes[3] = raw_message[45];
        GPS->height = iLong.iLong;

        // Collect North Velocity
        GPS->n_vel_bytes[0] = raw_message[54];
        GPS->n_vel_bytes[0] = raw_message[55];
        GPS->n_vel_bytes[0] = raw_message[56];
        GPS->n_vel_bytes[0] = raw_message[57];
        iLong.bytes[0] = raw_message[54];
        iLong.bytes[1] = raw_message[55];
        iLong.bytes[2] = raw_message[56];
        iLong.bytes[3] = raw_message[57];
        GPS->n_vel = iLong.iLong;

        // Collect East Velocity
        GPS->e_vel_bytes[0] = raw_message[58];
        GPS->e_vel_bytes[0] = raw_message[59];
        GPS->e_vel_bytes[0] = raw_message[60];
        GPS->e_vel_bytes[0] = raw_message[61];
        iLong.bytes[0] = raw_message[58];
        iLong.bytes[1] = raw_message[59];
        iLong.bytes[2] = raw_message[60];
        iLong.bytes[3] = raw_message[61];
        GPS->e_vel = iLong.iLong;

        // Collect Down Velocity
        GPS->d_vel_bytes[0] = raw_message[62];
        GPS->d_vel_bytes[0] = raw_message[63];
        GPS->d_vel_bytes[0] = raw_message[64];
        GPS->d_vel_bytes[0] = raw_message[65];
        iLong.bytes[0] = raw_message[62];
        iLong.bytes[1] = raw_message[63];
        iLong.bytes[2] = raw_message[64];
        iLong.bytes[3] = raw_message[65];
        GPS->d_vel = iLong.iLong;

        // Collect Magnetic Declination
        GPS->mag_dec_bytes[0] = raw_message[94];
        GPS->mag_dec_bytes[1] = raw_message[95];
        iShort.bytes[0] = raw_message[94];
        iShort.bytes[1] = raw_message[95];
        GPS->mag_dec = iShort.iShort;
    }
    return true;
}