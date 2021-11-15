/* System Includes */
#include "stm32l432xx.h"
#include "system_stm32l4xx.h"
#include "can_parse.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/quadspi/quadspi.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"


/* Module Includes */
#include "main.h"
#include "bitstream.h"

/* PER HAL Initilization Structures */
GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,

    // QuadSPI Chip Selects
    GPIO_INIT_OUTPUT(QUADSPI_CS_FPGA_GPIO_Port, QUADSPI_CS_FPGA_Pin, GPIO_OUTPUT_LOW_SPEED),

    // QuadSPI Data/CLK
    GPIO_INIT_AF(QUADSPI_CLK_GPIO_Port, QUADSPI_CLK_Pin, 10, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(QUADSPI_IO0_GPIO_Port, QUADSPI_IO0_Pin, 10, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(QUADSPI_IO1_GPIO_Port, QUADSPI_IO1_Pin, 10, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(QUADSPI_IO2_GPIO_Port, QUADSPI_IO2_Pin, 10, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(QUADSPI_IO3_GPIO_Port, QUADSPI_IO3_Pin, 10, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    // I2C Bus
    GPIO_INIT_AF(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 4, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 4, GPIO_OUTPUT_LOW_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(I2C_WRITE_CONTROL_GPIO_Port, I2C_WRITE_CONTROL_Pin, GPIO_OUTPUT_LOW_SPEED),

    // Status LEDs
    GPIO_INIT_OUTPUT(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_OUTPUT_LOW_SPEED),

    // FPGA Control
    GPIO_INIT_OUTPUT(FPGA_CFG_RST_GPIO_Port, FPGA_CFG_RST_Pin, GPIO_OUTPUT_LOW_SPEED)
};

#define TargetCoreClockrateHz 80000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =160000000,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 16),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (16 + 16)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (16 + 16)),
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* Function Prototypes */
void canReceiveTest();
void canSendTest();
void Error_Handler();
void SysTick_Handler();
void canTxUpdate();
void blinkTask();
void PHAL_FaultHandler();
extern void HardFault_Handler();

q_handle_t q_tx_can;
q_handle_t q_rx_can;

void heartbeat_task()
{
    PHAL_toggleGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);
}

void atask()
{
    PHAL_toggleGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
}

void btask()
{
    PHAL_toggleGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin);
}

int main (void)
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

    /* HAL Initilization */
    if (0 != PHAL_configureClockRates(&clock_config))
        PHAL_FaultHandler();

    if (1 != PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        PHAL_FaultHandler();
        

    if (1 != PHAL_initCAN(CAN1, false))
        PHAL_FaltHandler();

    if (1 != PHAL_qspiInit())
        PHAL_FaultHandler();
    
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    PHAL_writeGPIO(FPGA_CFG_RST_GPIO_Port, FPGA_CFG_RST_Pin, 0);
    PHAL_writeGPIO(QUADSPI_CS_FPGA_GPIO_Port, QUADSPI_CS_FPGA_Pin, 1);
    PHAL_writeGPIO(QUADSPI_CS_FLASH_GPIO_Port, QUADSPI_CS_FLASH_Pin, 1);

    /* Module init */
    bitstreamInit();
    schedInit(APB1ClockRateHz * 2); // See Datasheet DS11451 Figure. 4 for clock tree
    // initCANParse(&q_rx_can);

    /* Task Creation */
    // taskCreate(canRxUpdate, RX_UPDATE_PERIOD);
    // taskCreate(canTxUpdate, 5);
    taskCreate(bitstream10Hz, 100);
    // taskCreate(bitstream100Hz, 10);
    taskCreate(heartbeat_task, 1000);
    // taskCreate(atask, 300);
    // taskCreate(btask, 400);
    schedStart();

    return 0;
}

void blinkTask()
{
    PHAL_toggleGPIO(GPIOB, 3);
}

void PHAL_FaultHandler()
{
    asm("bkpt");
    HardFault_Handler();
}


// *** Compulsory CAN Tx/Rx callbacks ***
void canTxUpdate()
{
    CanMsgTypeDef_t tx_msg;
    if (qReceive(&q_tx_can, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
    {
        PHAL_txCANMessage(&tx_msg);
    }
}

void CAN1_RX0_IRQHandler()
{
    if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN1->RF0R &= !(CAN_RF0R_FOVR0); 

    if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN1->RF0R &= !(CAN_RF0R_FULL0); 

    if (CAN1->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;

        // Get either StdId or ExtId
        if (CAN_RI0R_IDE & CAN1->sFIFOMailBox[0].RIR)
        { 
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
        }

        rx.DLC = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
        rx.Data[1] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        rx.Data[2] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
        rx.Data[5] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
        rx.Data[6] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        canProcessRxIRQs(&rx);

        CAN1->RF0R     |= (CAN_RF0R_RFOM0); 

        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}