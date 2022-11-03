/**
 * @file process.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "bootloader.h"
#include "common/phal_L4/flash/flash.h"
#include "common/phal_L4/gpio/gpio.h"

static volatile uint32_t* app_flash_start_address;   /* Store the start address of the Application, never changes */
static volatile uint32_t* app_flash_current_address; /* Current address we are writing to */
static volatile uint32_t* app_flash_end_address;     /* Expected end address to stop writing */
static volatile uint32_t* bootloader_ms;

extern q_handle_t q_tx_can;

void BL_init(uint32_t* app_flash_start, volatile uint32_t* bootloader_ms_ptr)
{
    // CRC initializaion
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;              // Clock the CRC peripheral
    CRC->INIT = 0xFFFFFFFF;                         // Reset initial value
    CRC->CR &= ~CRC_CR_POLYSIZE_Msk;                // Set 32 bit (00)
    CRC->POL = 0x04C11DB7;                          // CRC-32b (Ethernet Polynomial)
    CRC->CR |= CRC_CR_RESET;                        // Reset CRC

    app_flash_start_address     = app_flash_start;
    app_flash_end_address       = app_flash_start;
    app_flash_current_address   = app_flash_start;
    bootloader_ms = bootloader_ms_ptr;
}

//#define DEBUG
static bool bl_unlock = false;
static bool flash_complete = false;
static uint32_t num_msg = 0;
static uint32_t first_word = 0;
static uint32_t second_word = 0;

void BL_processCommand(BLCmd_t cmd, uint32_t data)
{
    switch (cmd)
    {
        case BLCMD_START:
        {
            app_flash_current_address = app_flash_start_address;
            app_flash_end_address += data; // Number of words
            *bootloader_ms = 0;
            uint8_t total_pages = ((data + 0x1FF) / 0x200); // page size is 0x800 bytes -> 0x200 words
            
            #ifndef DEBUG
            for(int i = 0; i < total_pages; i++)
                 PHAL_flashErasePage(4+i); // offset by 0x2000 / 0x800
            #endif

            BL_sendStatusMessage(BLSTAT_METDATA_RX, (uint32_t) data);
            num_msg = 0;
            bl_unlock = true;
            flash_complete = false;
            first_word = second_word = 0;
            CRC->CR |= CRC_CR_RESET; // Reset CRC
            break;
        }
        case BLCMD_CRC:
        {
            if (bl_unlock && app_flash_current_address == app_flash_end_address)
            {
                if (CRC->DR != data)
                    BL_sendStatusMessage(BLSTAT_INVALID, BLERROR_CRC_FAIL);
                else
                {
                    // Firmware download successful
                    // Program first double word
                    PHAL_flashWriteU64((uint32_t) app_flash_start_address, (((uint64_t) second_word) << 32) | first_word);
                    BL_sendStatusMessage(BLSTAT_DONE, 0);
                    bl_unlock = false;
                    flash_complete = true;
                }
            }
            else if (!bl_unlock)
                BL_sendStatusMessage(BLSTAT_INVALID, BLERROR_LOCKED);
            else 
                BL_sendStatusMessage(BLSTAT_INVALID, BLERROR_LOW_ADDR);
            break;
        }
        case BLCMD_RST:
            // Bootloader_ResetForFirmwareDownload();
            break;
        default:
        {
            BL_sendStatusMessage(BLSTAT_UNKNOWN_CMD, cmd);
            break;
        }
    }
}

bool BL_flashStarted(void)
{
    return bl_unlock;
}

bool BL_flashComplete(void)
{
    return (num_msg > 10) && flash_complete;
}

volatile uint32_t* BL_getCurrentFlashAddress(void)
{
    return app_flash_current_address;
}

void BL_timeout(void)
{
    bl_unlock = false;
    num_msg = 0;
    flash_complete = false;
}

void bitstream_data_CALLBACK(CanParsedData_t* msg_data_a)
{

    #if (APP_ID == APP_L4_TESTING)
    PHAL_writeGPIO(GPIOB, 7, 1);
    #endif
    uint64_t data;    
    if (bl_unlock)
    {
        if(app_flash_current_address >= app_flash_start_address &&
           app_flash_current_address < app_flash_end_address)
        { 
            num_msg++;
            data = *((uint64_t *) msg_data_a->raw_data);

            // Skip the first two words
            if (app_flash_current_address == app_flash_start_address) 
            {
                CRC->DR = first_word = *((uint32_t *) msg_data_a->raw_data);
                CRC->DR = second_word = *((uint32_t *) msg_data_a->raw_data + 1);
            }
            else
            {
                // Address is 2-word aligned, do the actual programming now
                #ifndef DEBUG
                PHAL_flashWriteU64((uint32_t) app_flash_current_address, data);
                #endif
                CRC->DR = *(app_flash_current_address);
                CRC->DR = *(app_flash_current_address + 1);
            }
            *bootloader_ms = 0; // Reset timeout counter, message received!
            app_flash_current_address += 2; // wrote 64 bits
        }
        else
            BL_sendStatusMessage(BLSTAT_INVALID, BLERROR_ADDR_BOUND);
    }

    #if (APP_ID == APP_L4_TESTING)
    PHAL_writeGPIO(GPIOB, 7, 0);
    #endif
}

/*
    Component specific callbacks 
*/

// Quickly setup case statments for send function based on Node ID
#define NODE_CASE_BL_RESPONSE(app_id, resp_func) \
case app_id:\
            resp_func(q_tx_can, cmd, data);\
            break;\

void BL_sendStatusMessage(uint8_t cmd, uint32_t data)
{
    switch(APP_ID)
    {
        NODE_CASE_BL_RESPONSE(APP_MAIN_MODULE,      SEND_MAIN_MODULE_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_DASHBOARD,        SEND_DASHBOARD_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_TORQUEVECTOR,     SEND_TORQUEVECTOR_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_DRIVELINE_FRONT,  SEND_DRIVELINE_FRONT_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_DRIVELINE_REAR,   SEND_DRIVELINE_REAR_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_PRECHARGE,        SEND_PRECHARGE_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_A,            SEND_BMS_A_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_B,            SEND_BMS_B_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_C,            SEND_BMS_C_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_D,            SEND_BMS_D_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_E,            SEND_BMS_E_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_F,            SEND_BMS_F_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_G,            SEND_BMS_G_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_H,            SEND_BMS_H_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_I,            SEND_BMS_I_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_BMS_J,            SEND_BMS_J_BL_RESP)
        NODE_CASE_BL_RESPONSE(APP_L4_TESTING,       SEND_L4_TESTING_BL_RESP)
        default:
            asm("bkpt");
    }
}

// Quickly setup the CAN callbacks based on Node ID
#define NODE_BL_CMD_CALLBACK(callback_name, can_msg_name, node_id) \
void callback_name(CanParsedData_t* msg_data_a) {\
    if(APP_ID != node_id) return; \
    BL_processCommand((BLCmd_t) msg_data_a->can_msg_name.cmd, msg_data_a->can_msg_name.data); \
} \

NODE_BL_CMD_CALLBACK(main_module_bl_cmd_CALLBACK,     main_module_bl_cmd,     APP_MAIN_MODULE)
NODE_BL_CMD_CALLBACK(dashboard_bl_cmd_CALLBACK,       dashboard_bl_cmd,       APP_DASHBOARD)
NODE_BL_CMD_CALLBACK(torquevector_bl_cmd_CALLBACK,    torquevector_bl_cmd,    APP_TORQUEVECTOR)
NODE_BL_CMD_CALLBACK(driveline_front_bl_cmd_CALLBACK, driveline_front_bl_cmd, APP_DRIVELINE_FRONT)
NODE_BL_CMD_CALLBACK(driveline_rear_bl_cmd_CALLBACK,  driveline_rear_bl_cmd,  APP_DRIVELINE_REAR)
NODE_BL_CMD_CALLBACK(precharge_bl_cmd_CALLBACK,       precharge_bl_cmd,       APP_PRECHARGE)
NODE_BL_CMD_CALLBACK(bms_a_bl_cmd_CALLBACK,           bms_a_bl_cmd,           APP_BMS_A)
NODE_BL_CMD_CALLBACK(bms_b_bl_cmd_CALLBACK,           bms_b_bl_cmd,           APP_BMS_B)
NODE_BL_CMD_CALLBACK(bms_c_bl_cmd_CALLBACK,           bms_c_bl_cmd,           APP_BMS_C)
NODE_BL_CMD_CALLBACK(bms_d_bl_cmd_CALLBACK,           bms_d_bl_cmd,           APP_BMS_D)
NODE_BL_CMD_CALLBACK(bms_e_bl_cmd_CALLBACK,           bms_e_bl_cmd,           APP_BMS_E)
NODE_BL_CMD_CALLBACK(bms_f_bl_cmd_CALLBACK,           bms_f_bl_cmd,           APP_BMS_F)
NODE_BL_CMD_CALLBACK(bms_g_bl_cmd_CALLBACK,           bms_g_bl_cmd,           APP_BMS_G)
NODE_BL_CMD_CALLBACK(bms_h_bl_cmd_CALLBACK,           bms_h_bl_cmd,           APP_BMS_H)
NODE_BL_CMD_CALLBACK(bms_i_bl_cmd_CALLBACK,           bms_i_bl_cmd,           APP_BMS_I)
NODE_BL_CMD_CALLBACK(bms_j_bl_cmd_CALLBACK,           bms_j_bl_cmd,           APP_BMS_J)
NODE_BL_CMD_CALLBACK(l4_testing_bl_cmd_CALLBACK,      l4_testing_bl_cmd,      APP_L4_TESTING)
