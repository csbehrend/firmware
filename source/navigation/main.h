/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software for controlling navigation
            sensor acquisition
 * @version 0.1
 * @date 2022-12-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_

//STM32L471RET

// Status Indicators
#define ERR_LED_GPIO_Port   (GPIOB)
#define ERR_LED_Pin         (5)
#define CONN_LED_GPIO_Port  (GPIOB)
#define CONN_LED_Pin        (7)
#define CONN_LED_MS_THRESH  (500)
#define HEARTBEAT_GPIO_Port (GPIOB)
#define HEARTBEAT_Pin       (6)

#endif