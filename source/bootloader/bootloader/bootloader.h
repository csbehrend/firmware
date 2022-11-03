/**
 * @file process.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _PROCESS_H
#define _PROCESS_H

#include "inttypes.h"
#include "stdbool.h"
#include "can_parse.h"
#include "node_defs.h"
#include "common/bootloader/bootloader_common.h"


typedef enum
{
    BLSTAT_INVALID = 0,     /* Invalid operation */
    BLSTAT_BOOT = 1,        /* Bootloader boot alert */
    BLSTAT_WAIT = 2,        /* Waiting to get BLCMD */
    BLSTAT_METDATA_RX = 3,  /* Progress update for bootloader download */
    BLSTAT_PROGRESS = 4,    /* Progress update for bootloader download */
    BLSTAT_DONE = 5,        /* Completed the application download with CRC pass */
    BLSTAT_JUMP_TO_APP = 6, /* About to jump to application */
    BLSTAT_INVAID_APP = 7,  /* Did not attempt to boot because the starting address was invalid */
    BLSTAT_UNKNOWN_CMD = 8,  /* Incorrect CAN command message format */
} BLStatus_t;

typedef enum
{
    BLERROR_CRC_FAIL = 0,
    BLERROR_LOCKED = 1,
    BLERROR_LOW_ADDR = 2,
    BLERROR_ADDR_BOUND = 3
} BLError_t;

void BL_init(uint32_t* app_flash_start, volatile uint32_t* bootloader_ms_ptr);

/**
 * @brief Process an incoming bootlaoder command
 * 
 * @param cmd 
 * @param data 
 */
void BL_processCommand(BLCmd_t cmd, uint32_t data);

/**
 * @brief The entire application has been written to flash.
 * 
 * @return true 
 * @return false 
 */
bool BL_flashComplete(void);

/**
 * @brief Send a Bootloader status message with the correct app ID
 * 
 * @param cmd  Command/status enum, see BLStatus_t
 * @param data Status data, context specific
 */
void BL_sendStatusMessage(uint8_t cmd, uint32_t data);

bool BL_flashStarted(void);
void BL_timeout(void);
volatile uint32_t* BL_getCurrentFlashAddress(void);


#endif