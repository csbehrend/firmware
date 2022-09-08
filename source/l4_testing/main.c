/* System Includes */
#include "stm32l432xx.h"
#include "common/psched/psched.h"
#include "common/queue/queue.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/phal_L4/gpio/gpio.h"

/* Module Includes */
#include "main.h"
#include "can_parse.h"

GPIOInitConfig_t gpio_config[] = {
    // TODO: LED port
    // TODO: Button port
    // TODO: CAN ports
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
void Error_Handler();
extern void HardFault_Handler();

// TODO: queue definitions

int main (void)
{
    /* Data Struct Initialization */
    // TODO: init queues

    /* HAL Initilization */
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    // TODO: init CAN

    // TODO: configure button interrupt

    /* Task Creation */
    schedInit(SystemCoreClock);
    // TODO: LED task
    // TODO: CAN background tasks
    schedStart();
    
    return 0;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}