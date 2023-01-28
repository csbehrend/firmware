/**
 * @file bmm150.c
 * @author your name (cmcgalli@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2023-01-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bmm150.h"
#include "common/phal_L4/spi/spi.h"
#include "common_defs.h"
#include "stdbool.h"

static inline void BMM150_selectMag(BMM150_Handle_t *bmm);
bool BMM150_readID(BMM150_Handle_t *bmm);
void BMM150_powerOnMag(BMM150_Handle_t *bmm);
bool BMM150_init(BMM150_Handle_t *bmm);
bool BMM150_readMag(BMM150_Handle_t *bmm);
void BMM150_setActive(BMM150_Handle_t *bmm);
int16_t raw_x, raw_y, raw_z;
bool BMM150_readMag(BMM150_Handle_t *bmm)
{
    static uint8_t spi_rx_buff[16] = {0};
    static uint8_t spi_tx_buff[16] = {0};

    spi_tx_buff[0] = (1 << 7) | BMM150_MAG_X_LSB_ADDR;
    BMM150_selectMag(bmm);
    while (PHAL_SPI_busy(bmm->spi))
        ;
    PHAL_SPI_transfer(bmm->spi, spi_tx_buff, 7, spi_rx_buff);
    while (PHAL_SPI_busy(bmm->spi))
        ;
    parsed_data_t new_data = {.raw_data[0] = spi_rx_buff[2], .raw_data[1] = spi_rx_buff[1]};

    // raw_x = ((int16_t)(((spi_rx_buff[2]) << 8) | (spi_rx_buff[1] << 3))) >> 3;
    // raw_y = ((int16_t)(((spi_rx_buff[4]) << 8) | (spi_rx_buff[3] << 3))) >> 3;
    // raw_z = ((int16_t)(((spi_rx_buff[6]) << 8) | (spi_rx_buff[5] << 1))) >> 1;
    raw_x = (int16_t)(new_data.value >> 3);
    raw_x = (raw_x & 0x1000) ? (raw_x | 0xe000) : raw_x;
    new_data.raw_data[0] = spi_rx_buff[4];
    new_data.raw_data[1] = spi_rx_buff[3];
    raw_y = (int16_t)(new_data.value >> 3);
    parsed_data_z_t new_z_data = {.raw_data_1 = spi_rx_buff[6], .raw_data_2 = spi_rx_buff[5]};
    raw_z = (int16_t)new_z_data.value;

    // parsed_data_t new_data = {.raw_data_1 = spi_rx_buff[6], .raw_data_2 = spi_rx_buff[5]};
    // raw_z = (int16_t)new_data.value;

    return true;
}

void BMM150_powerOnMag(BMM150_Handle_t *bmm)
{
    BMM150_selectMag(bmm);
    PHAL_SPI_writeByte(bmm->spi, 0x4b, 0b00000001);
    BMM150_setActive(bmm);
    return;
}

void BMM150_setActive(BMM150_Handle_t *bmm)
{
    BMM150_selectMag(bmm);
    PHAL_SPI_writeByte(bmm->spi, 0x4c, 0b00000000);
    return;
}

uint8_t testresult = 0;
bool BMM150_readID(BMM150_Handle_t *bmm)
{
    BMM150_selectMag(bmm);
    BMM150_powerOnMag(bmm);
    testresult = PHAL_SPI_readByte(bmm->spi, BMM150_CHIP_ID_ADDR, true);
    if (PHAL_SPI_readByte(bmm->spi, BMM150_CHIP_ID_ADDR, true) != BMM150_CHIP_ID)
        return false;
    return true;
}

static inline void BMM150_selectMag(BMM150_Handle_t *bmm)
{
    bmm->spi->nss_gpio_port = bmm->mag_csb_gpio_port;
    bmm->spi->nss_gpio_pin = bmm->mag_csb_pin;
}
