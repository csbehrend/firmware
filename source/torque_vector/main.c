/* System Includes */
#include "stm32l432xx.h"
#include "system_stm32l4xx.h"
#include "can_parse.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/quadspi/quadspi.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/bootloader/bootloader_common.h"
#include "Layer_0.h"
#include "tv_data.h"
#include "rt_nonfinite.h"


/* Module Includes */
#include "main.h"
#include "bitstream.h"

/* PER HAL Initilization Structures */
GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,
    // QuadSPI Chip Selects
    GPIO_INIT_OUTPUT(QUADSPI_CS_FLASH_GPIO_Port, QUADSPI_CS_FLASH_Pin, GPIO_OUTPUT_LOW_SPEED),
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
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_OUTPUT_LOW_SPEED)
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
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
void PHAL_FaltHandler();
extern void HardFault_Handler();

q_handle_t q_tx_can;
q_handle_t q_rx_can;

int main (void)
{
    // Main stack pointer is saved as the first entry in the .isr_entry
    Bootloader_ConfirmApplicationLaunch();

    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

    /* HAL Initilization */
    if (0 != PHAL_configureClockRates(&clock_config))
        PHAL_FaltHandler();

    if (1 != PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        PHAL_FaltHandler();
        
    if (1 != PHAL_initCAN(CAN1, false))
        PHAL_FaltHandler();

    if (1 != PHAL_qspiInit())
        PHAL_FaltHandler();
    
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* Module init */
    // bitstreamInit();
    schedInit(SystemCoreClock); // See Datasheet DS11451 Figure. 4 for clock tree
    initCANParse(&q_rx_can);

    PHAL_writeGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, 1);

    /* Task Creation */
    taskCreate(blinkTask, 500);
    taskCreate(bitstream10Hz, 100);
    taskCreate(bitstream100Hz, 10);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);
    schedStart();

    return 0;
}



static void argInit_1x4_real_T(double result[4])
{
  int idx1;
  for (idx1 = 0; idx1 < 4; idx1++) {
    result[idx1] = 0.0;
  }
}

static void argInit_1x6_real_T(double result[6])
{
  int idx1;
  for (idx1 = 0; idx1 < 6; idx1++) {
    result[idx1] = 0.0;
  }
}

void blinkTask()
{
    PHAL_toggleGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);

    double dv[6];
    double FY[4];
    double Fx_max[4];
    double b_omega_w_tmp[4];
    double c_omega_w_tmp[4];
    double omega_w_tmp[4];
    double Vg[2];
    double acker_steer_angles[2];
    double center_steer_angle_tmp;
    /* Initialize function 'Layer_0' input arguments. */
    /* Initialize function input argument 'omega_w'. */
    argInit_1x4_real_T(omega_w_tmp);
    center_steer_angle_tmp = 0.0;
    /* Initialize function input argument 'FZ'. */
    /* Initialize function input argument 'SL'. */
    /* Initialize function input argument 'vel'. */
    /* Call the entry-point 'Layer_0'. */
    argInit_1x6_real_T(dv);
    b_omega_w_tmp[0] = omega_w_tmp[0];
    b_omega_w_tmp[1] = omega_w_tmp[1];
    b_omega_w_tmp[2] = omega_w_tmp[2];
    b_omega_w_tmp[3] = omega_w_tmp[3];
    c_omega_w_tmp[0] = omega_w_tmp[0];
    c_omega_w_tmp[1] = omega_w_tmp[1];
    c_omega_w_tmp[2] = omega_w_tmp[2];
    c_omega_w_tmp[3] = omega_w_tmp[3];
    Layer_0(omega_w_tmp, &center_steer_angle_tmp, center_steer_angle_tmp,
            center_steer_angle_tmp, &center_steer_angle_tmp, b_omega_w_tmp,
            c_omega_w_tmp, dv, acker_steer_angles, FY, Vg, Fx_max);

}

void PHAL_FaltHandler()
{
    asm("bkpt");
    HardFault_Handler();
}

// *** Compulsory CAN Tx/Rx callbacks ***
void bootloader_request_reset_CALLBACK(CanParsedData_t* data)
{
    Bootloader_ResetForFirmwareDownload();
}

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