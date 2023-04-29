/* System Includes */
#include "stm32l432xx.h"
#include "common/bootloader/bootloader_common.h"
#include "common/faults/faults.h"
#include "common/phal_L4/adc/adc.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/eeprom_spi/eeprom_spi.h"
#include "common/phal_L4/dma/dma.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/i2c/i2c.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/psched/psched.h"
#include "common/queue/queue.h"
#include "common/phal_L4/usart/usart.h"

/* Module Includes */
#include "main.h"
#include "testbench.h"

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_USART2TX_PA2,
    GPIO_INIT_USART2RX_PA15,
    GPIO_INIT_ANALOG(FORCE_SENSE0_GPIO_Port, FORCE_SENSE0_Pin),
    /*
    GPIO_INIT_ANALOG(FORCE_SENSE1_GPIO_Port, FORCE_SENSE1_Pin),
    GPIO_INIT_ANALOG(FORCE_SENSE2_GPIO_Port, FORCE_SENSE2_Pin),
    GPIO_INIT_ANALOG(FORCE_SENSE3_GPIO_Port, FORCE_SENSE3_Pin),
    */
    // Status LED
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_OUTPUT_LOW_SPEED)
};

/* USART Configuration */
dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t huart = {
    .baud_rate   = 115200,
    .word_length = WORD_8,
    .hw_flow_ctl = HW_DISABLE,
    .mode        = MODE_TX_RX,
    .stop_bits   = SB_ONE,
    .parity      = PT_NONE,
    .obsample    = OB_DISABLE,
    .ovsample    = OV_16,
    .adv_feature.rx_inv    = false,
    .adv_feature.tx_inv    = false,
    .adv_feature.auto_baud = false,
    .adv_feature.data_inv  = false,
    .adv_feature.msb_first = false,
    .adv_feature.overrun   = false,
    .adv_feature.dma_on_rx_err = false,
    .tx_dma_cfg = &usart_tx_dma_config,
    .rx_dma_cfg = &usart_rx_dma_config
};

/* ADC Configuration */
ADCInitConfig_t adc_config = {
    .clock_prescaler = ADC_CLK_PRESC_6,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .overrun         = true,
    .dma_mode        = ADC_DMA_CIRCULAR
};

ADCChannelConfig_t adc_channel_config[] = {
    {.channel=FORCE_SENSE0_ADC_CHNL,   .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_2_5}
    //{.channel=FORCE_SENSE1_ADC_CHNL,   .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_2_5},
    //{.channel=FORCE_SENSE2_ADC_CHNL,   .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_2_5},
    //{.channel=FORCE_SENSE3_ADC_CHNL,   .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_2_5}
};

volatile ADCReadings_t force_readings;

// Priority level currently medium -- check DMA_CCR for alt values
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &force_readings,
            sizeof(force_readings) / sizeof(force_readings.force_0), 0b01);

/* SPI Configuration */
/*
dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config = {
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = EEPROM_NSS_GPIO_Port,
    .nss_gpio_pin = EEPROM_NSS_Pin,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1
};
*/

/* Clock Configuration */
/*
#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source          = SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz = TargetCoreClockrateHz,
    .ahb_clock_target_hz    = (TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz   = (TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz   = (TargetCoreClockrateHz / (1)),
};
*/

ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .system_clock_target_hz     =80000000,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =160000000,
    .ahb_clock_target_hz        =80000000,
    .apb1_clock_target_hz       =80000000,// / 16,
    .apb2_clock_target_hz       =80000000 / 16,
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* Function Prototypes */
void preflightChecks(void);
void ledBlink(void);
void usartTxUpdate();
void readPeriodic(void);
extern void HardFault_Handler();

q_handle_t q_tx_usart;
force_t forceConfig;

int main(void){

    //qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    //qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_tx_usart, FORCE_MAX_TX_LENGTH);
    
    /* HAL Initialization */
    

    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initDMA(&adc_dma_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initDMA(&adc_dma_config))
    {
        HardFault_Handler();
    }
    PHAL_startTxfer(&adc_dma_config);
    PHAL_startADC(ADC1);

    huart.rx_dma_cfg->circular = true;
    if(!PHAL_initUSART(USART2, &huart, APB1ClockRateHz))
    {
        HardFault_Handler();
    }
    
    initForce(&forceConfig, &q_tx_usart);

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    taskCreate(ledBlink, 500);
    taskCreate(readPeriodic, 15);
    taskCreateBackground(usartTxUpdate);

    schedStart();

    return 0;
}

/*
void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        case 0:
            huart_l.rx_dma_cfg->circular = true;
            if(!PHAL_initUSART(USART_L, &huart_l, APB1ClockRateHz))
            {
                HardFault_Handler();
            }
            huart_r.rx_dma_cfg->circular = true;
            break;
        case 1:
            
            break;
        case 2:
            //initCANParse(&q_rx_can);
            if(daqInit(&q_tx_can))
                HardFault_Handler();
            initFaultLibrary(FAULT_NODE_NAME, &q_tx_can, ID_FAULT_SYNC_MAIN_MODULE);
            break;
        default:
            registerPreflightComplete(1);
            state = 255; // prevent wrap around
    }
}
*/

void ledBlink()
{
    PHAL_toggleGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);
}

/* USART Message Handling */
uint8_t tmp_left[FORCE_MAX_TX_LENGTH] = {'\0'};
uint8_t tmp_right[FORCE_MAX_TX_LENGTH] = {'\0'};
void usartTxUpdate()
{
    if (PHAL_usartTxDmaComplete(&huart) &&
        qReceive(&q_tx_usart, tmp_left) == SUCCESS_G)
    {
        PHAL_usartTxDma(USART2, &huart, (uint16_t *) tmp_left, strlen(tmp_left));
    }
}

void readPeriodic(void)
{
    //force_t * config = &forceConfig;
    //ADCReadings_t * readings = &force_readings;
    //forceSetParam(config, readings);
    forceSetParam(&forceConfig, &force_readings);
}

void HardFault_Handler()
{
    PHAL_writeGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }
}