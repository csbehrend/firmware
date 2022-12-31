// /**
//  * @file gps.h
//  * @author Chris McGalliard (cmcgalli@purdue.edu)
//  * @brief 
//  * @version 0.1
//  * @date 2022-12-28
//  * 
//  * 
//  */

#ifndef _GPS_H
#define _GPS_H

// GPS handle
typedef struct {
    
} GPS_Handle_t;

//Hex generated using u-center 22.07 Generation 9 Advanced Configuration View
//All configurations are saved RAM layer only

//Hex to enable GPS at L1C/A Signal
static const gnssConfig[] = {'B5', '62', '06', '8A', '9F', '00', '01', '00', '00', '00', '01', '00', '31', '10', '01', '02', '00', '31', '10', '00', '03', '00', '31', '10', '00', '04', '00', '31', '10', '00', '05', '00', '31', '10', '00', '07', '00', '31', '10', '00', '09', '00', '31', '10', '00', '0A', '00', '31', '10', '00', '0B', '00', '31', '10', '00', '0D', '00', '31', '10', '00', '0E', '00', '31', '10', '00', '00', '00', '00', '00', '00', '11', '00', '31', '10', '00', '12', '00', '31', '10', '00', '13', '00', '31', '10', '00', '14', '00', '31', '10', '00', '15', '00', '31', '10', '00', '17', '00', '31', '10', '00', '18', '00', '31', '10', '00', '1A', '00', '31', '10', '00', '1C', '00', '31', '10', '00', '1D', '00', '31', '10', '00', '1F', '00', '31', '10', '01', '20', '00', '31', '10', '00', '21', '00', '31', '10', '00', '22', '00', '31', '10', '00', '23', '00', '31', '10', '00', '24', '00', '31', '10', '00', '25', '00', '31', '10', '00', '26', '00', '31', '10', '00', '00', '00', '00', '00', '00', 'D3', '8B'};

//Main Config Hex
//CFG-UART1-BAUDRATE 9600
//CFG-UART1-STOPBITS 1
//CFG-UART1-DATABITS 0 (Eight)
//CFG-UART1-PARITY 0 (None)
//CFG-UART1-ENABLED 1
//CFG-SIGNAL-GPS_ENA 1
//CFG-MSGOUT-UBX_NAV_PVT_UART1 1 (Enable PVT message on UART1 at rate of 1)
//CFG-UART1INPROT-UBX 1 -> Assign input messages to be over UBX
//CFG-UART1OUTPROT-UBX 1 -> Assign output messages to be over UBX
//CFG-NAVSPG-DYNMODEL 0 (Porable mode)
static const mainConfig[] = {
    'b5', '62', '06', '8a', '39', '00', '00', '01', '00', '00', '07', '00', '91', '20', '01', '02', '00', '52', '20', '01', '03', '00', '52', '20', '00', '04', '00', '52', '20', '00', '05', '00', '52', '10', '01', '01', '00', '73', '10', '01', '21', '00', '11', '20', '00', '1f', '00', '31', '10', '01', '01', '00', '74', '10', '01', '01', '00', '52', '40', '80', '25', '00', '00', '41', '62'
};

#endif //_GPS_H