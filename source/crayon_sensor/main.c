/* System Includes */
#include "stm32l496xx.h"
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

/* Module Includes */
#include "car.h"
#include "can_parse.h"
#include "cooling.h"
#include "daq.h"
#include "main.h"
#include "power_monitor.h"


GPIOInitConfig_t gpio_config[] = {
    // LV Status
    GPIO_INIT_ANALOG(LV_24V_V_SENSE_GPIO_Port, LV_24V_V_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_24V_I_SENSE_GPIO_Port, LV_24V_I_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_12V_V_SENSE_GPIO_Port, LV_12V_V_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_5V_V_SENSE_GPIO_Port, LV_5V_V_SENSE_Pin),
};

/* ADC Configuration */
ADCInitConfig_t adc_config = {
    .clock_prescaler = ADC_CLK_PRESC_16,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .overrun         = true,
    .dma_mode        = ADC_DMA_CIRCULAR
};

/* With 11 items, 16 prescaler, and 640 sample time, each channel gets read every 1.4ms */
ADCChannelConfig_t adc_channel_config[] = {
    {.channel=V_MC_SENSE_ADC_CHNL,     .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=V_BAT_SENSE_ADC_CHNL,    .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=SHOCK_POT_L_ADC_CHNL,    .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=SHOCK_POT_R_ADC_CHNL,    .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_24V_V_SENSE_ADC_CHNL, .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_24V_I_SENSE_ADC_CHNL, .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_12V_V_SENSE_ADC_CHNL, .rank=7,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_5V_V_SENSE_ADC_CHNL,  .rank=8,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_5V_I_SENSE_ADC_CHNL,  .rank=9,  .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=LV_3V3_V_SENSE_ADC_CHNL, .rank=10, .sampling_time=ADC_CHN_SMP_CYCLES_640_5},
    {.channel=THERM_MUX_D_ADC_CHNL,    .rank=11, .sampling_time=ADC_CHN_SMP_CYCLES_640_5}
};
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &adc_readings,
            sizeof(adc_readings) / sizeof(adc_readings.lv_3v3_v_sense), 0b01);

/* SPI Configuration */
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

/* Clock Configuration */
#define TargetCoreClockrateHz 80000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* Function Prototypes */
void preflightAnimation(void);
void preflightChecks(void);
void heartBeatLED();
void canTxUpdate(void);
void send_fault(uint16_t, bool);
extern void HardFault_Handler();

q_handle_t q_tx_can;
q_handle_t q_rx_can;

uint8_t can_tx_fails; // number of CAN messages that failed to transmit

int main(void){
    /* Data Struct Initialization */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    can_tx_fails = 0;

    /* HAL Initialization */
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }


    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 60, 750);

    taskCreate(coolingPeriodic, 200);
    taskCreate(heartBeatLED, 500);
    taskCreate(carHeartbeat, 100);
    // taskCreate(carPeriodic, 15);
    // taskCreate(setFanPWM, 1);
    taskCreate(updatePowerMonitor, 1000);
    // taskCreate(updateFaults, 5);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    taskCreate(memFg, MEM_FG_TIME);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);
    taskCreateBackground(memBg);

    schedStart();

    return 0;
}

void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        case 0:
            if(!PHAL_initCAN(CAN1, false))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
            spi_config.data_rate = APB2ClockRateHz / 16; // 5 MHz
            if (!PHAL_SPI_init(&spi_config))
                HardFault_Handler();
            if (initMem(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, &spi_config, 1, 1) != E_SUCCESS)
                HardFault_Handler();
           break;
        case 1:
            if(!PHAL_initADC(ADC1, &adc_config, adc_channel_config,
                            sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
            {
                HardFault_Handler();
            }
            if(!PHAL_initDMA(&adc_dma_config))
            {
                HardFault_Handler();
            }
            initPowerMonitor();
            PHAL_startTxfer(&adc_dma_config);
            PHAL_startADC(ADC1);
           break;
        case 2:
           /* Module Initialization */
           carInit();
           coolingInit();
           break;
       case 3:
           initCANParse(&q_rx_can);
           if(daqInit(&q_tx_can))
               HardFault_Handler();
           initFaultLibrary(FAULT_NODE_NAME, &q_tx_can, ID_FAULT_SYNC_MAIN_MODULE);
           break;
        default:
            registerPreflightComplete(1);
            state = 255; // prevent wrap around
    }
}

void preflightAnimation(void) {
    static uint32_t time;

    PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 0);
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6)
    {
        case 0:
        case 5:
            PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 1);
            break;
        case 1:
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
        case 2:
        case 3:
            PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
            break;
    }
}

void heartBeatLED(void)
{
    static uint8_t trig;
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
         PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    else PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);

    // Send every other time (1000 ms)
    if (trig) SEND_MCU_STATUS(q_tx_can, sched.skips, (uint8_t) sched.fg_time.cpu_use,
                                           (uint8_t) sched.bg_time.cpu_use,
                                           sched.error, can_tx_fails);
    trig = !trig;
}

void canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    if (qReceive(&q_tx_can, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
    {
        if (!PHAL_txCANMessage(&tx_msg)) ++can_tx_fails;
    }
}

void CAN1_RX0_IRQHandler()
{
    if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN1->RF0R &= !(CAN_RF0R_FOVR0);
        CAN1->RF0R &= !(CAN_RF0R_FOVR0);

    if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN1->RF0R &= !(CAN_RF0R_FULL0);
        CAN1->RF0R &= !(CAN_RF0R_FULL0);

    if (CAN1->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;
        rx.Bus = CAN1;

        // Get either StdId or ExtId
        rx.IDE = CAN_RI0R_IDE & CAN1->sFIFOMailBox[0].RIR;
        if (rx.IDE)
        {
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_STID_Pos;
        }

        rx.DLC = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
        rx.Data[1] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
        rx.Data[2] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
        rx.Data[5] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
        rx.Data[6] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        CAN1->RF0R |= (CAN_RF0R_RFOM0);
        CAN1->RF0R |= (CAN_RF0R_RFOM0);

        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}

void main_module_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.main_module_bl_cmd.cmd == BLCMD_RST)
        Bootloader_ResetForFirmwareDownload();
}

void HardFault_Handler()
{
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }
}