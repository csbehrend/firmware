#include <stdio.h>
#include <stdbool.h>
#include "gps.h"

union i_Long iLong;
union i_Short iShort;
// Nav Message
GPS_Handle_t gps_handle = {.raw_message = {0},
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
                           .mag_dec = 0,
                           .fix_type = -1,
                           .messages_received = -1};

// Parse velocity from raw message
bool parseVelocity(GPS_Handle_t *GPS)
{
    // Validate the message header, class, and id
    if (((GPS->raw_message)[0] == 181) && (GPS->raw_message[1] == 98) && ((GPS->raw_message)[2] == 1) && ((GPS->raw_message)[3] == 7))
    {
        // Collect fix type
        GPS->fix_type = GPS->raw_message[26];
        if (GPS->fix_type > 2)
        {
            // Collect Ground Speed
            GPS->g_speed_bytes[0] = GPS->raw_message[66];
            GPS->g_speed_bytes[1] = GPS->raw_message[67];
            GPS->g_speed_bytes[2] = GPS->raw_message[68];
            GPS->g_speed_bytes[3] = GPS->raw_message[69];
            iLong.bytes[0] = GPS->raw_message[66];
            iLong.bytes[1] = GPS->raw_message[67];
            iLong.bytes[2] = GPS->raw_message[68];
            iLong.bytes[3] = GPS->raw_message[69];
            GPS->g_speed = iLong.iLong;

            // Collect Longitude
            GPS->longitude_bytes[0] = GPS->raw_message[30];
            GPS->longitude_bytes[1] = GPS->raw_message[31];
            GPS->longitude_bytes[2] = GPS->raw_message[32];
            GPS->longitude_bytes[3] = GPS->raw_message[33];
            iLong.bytes[0] = GPS->raw_message[30];
            iLong.bytes[1] = GPS->raw_message[31];
            iLong.bytes[2] = GPS->raw_message[32];
            iLong.bytes[3] = GPS->raw_message[33];
            GPS->longitude = iLong.iLong;

            // Colect Latitude
            GPS->latitude_bytes[0] = GPS->raw_message[34];
            GPS->latitude_bytes[1] = GPS->raw_message[35];
            GPS->latitude_bytes[2] = GPS->raw_message[36];
            GPS->latitude_bytes[3] = GPS->raw_message[37];
            iLong.bytes[0] = GPS->raw_message[34];
            iLong.bytes[1] = GPS->raw_message[35];
            iLong.bytes[2] = GPS->raw_message[36];
            iLong.bytes[3] = GPS->raw_message[37];
            GPS->latitude = iLong.iLong;

            // Collect Height Above Elip.
            GPS->height_bytes[0] = GPS->raw_message[42];
            GPS->height_bytes[1] = GPS->raw_message[43];
            GPS->height_bytes[2] = GPS->raw_message[44];
            GPS->height_bytes[3] = GPS->raw_message[45];
            iLong.bytes[0] = GPS->raw_message[42];
            iLong.bytes[1] = GPS->raw_message[43];
            iLong.bytes[2] = GPS->raw_message[44];
            iLong.bytes[3] = GPS->raw_message[45];
            GPS->height = iLong.iLong;

            // Collect North Velocity
            GPS->n_vel_bytes[0] = GPS->raw_message[60];
            GPS->n_vel_bytes[1] = GPS->raw_message[61];
            GPS->n_vel_bytes[2] = GPS->raw_message[62];
            GPS->n_vel_bytes[3] = GPS->raw_message[63];
            iLong.bytes[0] = GPS->raw_message[60];
            iLong.bytes[1] = GPS->raw_message[61];
            iLong.bytes[2] = GPS->raw_message[62];
            iLong.bytes[3] = GPS->raw_message[63];
            GPS->n_vel = iLong.iLong;

            // Collect East Velocity
            GPS->e_vel_bytes[0] = GPS->raw_message[64];
            GPS->e_vel_bytes[1] = GPS->raw_message[65];
            GPS->e_vel_bytes[2] = GPS->raw_message[66];
            GPS->e_vel_bytes[3] = GPS->raw_message[67];
            iLong.bytes[0] = GPS->raw_message[64];
            iLong.bytes[1] = GPS->raw_message[65];
            iLong.bytes[2] = GPS->raw_message[66];
            iLong.bytes[3] = GPS->raw_message[67];
            GPS->e_vel = iLong.iLong;

            // Collect Down Velocity
            GPS->d_vel_bytes[0] = GPS->raw_message[68];
            GPS->d_vel_bytes[1] = GPS->raw_message[69];
            GPS->d_vel_bytes[2] = GPS->raw_message[70];
            GPS->d_vel_bytes[3] = GPS->raw_message[71];
            iLong.bytes[0] = GPS->raw_message[68];
            iLong.bytes[1] = GPS->raw_message[69];
            iLong.bytes[2] = GPS->raw_message[70];
            iLong.bytes[3] = GPS->raw_message[71];
            GPS->d_vel = iLong.iLong;
        }

        // // Collect Magnetic Declination
        // GPS->mag_dec_bytes[0] = GPS->raw_message[94];
        // GPS->mag_dec_bytes[1] = GPS->raw_message[95];
        // iShort.bytes[0] = GPS->raw_message[94];
        // iShort.bytes[1] = GPS->raw_message[95];
        // GPS->mag_dec = iShort.iShort;
    }
    return true;
}