/*
 * msp_hal.h
 *
 *  Created on: 28. 7. 2018
 *  Author	  : Martin
 */

#ifndef MSP_HAL_H_
#define MSP_HAL_H_

#include "driverlib/MSP430FR5xx_6xx/driverlib.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "Defines.h"


void Init_RTC(Calendar* pCalendar);
void Init_GPIO( void );
void Init_Clock( void );
void Init_UART_A0( void );
void Init_UART_A3( void );
void delayMilis( uint32_t u32NumOfMiliseconds );

#endif /* MSP_HAL_H_ */
