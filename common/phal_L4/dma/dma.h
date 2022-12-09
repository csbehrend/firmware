#ifndef _DMA_H_
#define _DMA_H_

#ifdef STM32L496xx
#include "stm32l496xx.h"
#elif STM32L471xx
#include "stm32l471xx.h"
#elif STM32L432xx
#include "stm32l432xx.h"
#else
#error "Please define a STM32 arch"
#endif

#include <stdbool.h>

typedef struct {
    uint32_t    periph_addr;
    uint32_t    mem_addr;
    uint16_t    tx_size;

    bool        increment;
    bool        circular;
    uint8_t     dir;
    bool        mem_inc;
    bool        periph_inc;
    bool        mem_to_mem;
    uint8_t     priority;
    uint8_t     mem_size;
    uint8_t     periph_size;
    bool        tx_isr_en;
    uint8_t     dma_chan_request; /* Table 44 of Family Reference  */
    uint8_t     channel_idx;

    DMA_TypeDef* periph;
    DMA_Channel_TypeDef* channel;
    DMA_Request_TypeDef* request;
} dma_init_t;

/*
 * @brief Initialize DMA peripheral to set m2m, p2p, or p2m with set size
 *        and length of txfer
 * 
 * @param init -> Address of initialization structure
 * @return true -> Successful init (no clashing params)
 * @return false -> Init not complete (parameters clash)
 */
bool PHAL_initDMA(dma_init_t* init);

/*
 * @brief Start txfer after sucessful DMA peripheral initialization
 * 
 * @param init -> Address of initialization structure
 */
void PHAL_startTxfer(dma_init_t* init);

/*
 * @brief Stop txfer
 * 
 * @param init -> Address of initialization structure
 */
void PHAL_stopTxfer(dma_init_t* init);

/*
 * @brief Re-enable DMA txfer after error ISR fires
 * 
 * @param init -> Address of initialization structure
 */
void PHAL_reEnable(dma_init_t* init);

/*
 * @brief Set memory address for DMA transfer. In Mem to Mem this acts as the source address
 * 
 * @param init -> Address of initialization structure
 */
void PHAL_DMA_setMemAddress(dma_init_t* init, const uint32_t address);

/*
 * @brief Set transfer length for DMA transaction
 * 
 * @param init -> Address of initialization structure
 */
void PHAL_DMA_setTxferLength(dma_init_t* init, const uint32_t length);

#endif