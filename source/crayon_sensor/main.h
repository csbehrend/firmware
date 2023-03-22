/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software for controlling SDC based on
 *         faults and cooling based on temperatures
 * @version 0.1
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_


//STM32L496VGT6

#include "common/faults/fault_nodes.h"

#define FAULT_NODE_NAME NODE_MAIN_MODULE

// LV Status
#define FORCE_SENSE0_GPIO_Port    (GPIOA)
#define FORCE_SENSE0_Pin          (6)
#define FORCE_SENSE0_ADC_CHNL     (11)

#define FORCE_SENSE1_GPIO_Port    (GPIOA)
#define FORCE_SENSE1_Pin          (7)
#define FORCE_SENSE1_ADC_CHNL     (12)

#define FORCE_SENSE2_GPIO_Port     (GPIOB)
#define FORCE_SENSE2_Pin           (0)
#define FORCE_SENSE2_SENSE_ADC_CHNL      (15)

#define FORCE_SENSE3_GPIO_Port    (GPIOB)
#define FORCE_SENSE3_Pin          (1)
#define FORCE_SENSE3_ADC_CHNL     (16)

#endif