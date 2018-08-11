///*
// * msp_hal.c
// *
// *  Created on: 28. 7. 2018
// *  Author	  : Martin
// */
//#include "msp_hal.h"
//
//Calendar calendar;
//
//
///*
// * GPIO Initialization
// */
//void Init_GPIO()
//{
//    // Set all GPIO pins to output low for low power
//    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setOutputLowOnPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );
//    GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN0 );
//
//    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
//    GPIO_setAsOutputPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );
//
//    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
//    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
//    GPIO_setAsPeripheralModuleFunctionInputPin(
//    GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );
//
//    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
//    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
//    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN1,
//    GPIO_PRIMARY_MODULE_FUNCTION );
//
//    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
//    GPIO_setAsPeripheralModuleFunctionInputPin(
//    GPIO_PORT_PJ,
//    GPIO_PIN4 + GPIO_PIN5,
//    GPIO_PRIMARY_MODULE_FUNCTION );
//
//    // Disable the GPIO power-on default high-impedance mode
//    // to activate previously configured port settings
//    PMM_unlockLPM5();
//}
//
///*
// * Clock System Initialization
// */
//void Init_Clock()
//{
//    // Set DCO frequency to 8 MHz
//    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_6 );
//    //Set external clock frequency to 32.768 KHz
//    CS_setExternalClockSource( 32768, 0 );
//    //Set ACLK=LFXT
//    CS_initClockSignal( CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    // Set SMCLK = DCO with frequency divider of 1
//    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    // Set MCLK = DCO with frequency divider of 1
//    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    //Start XT1 with no time out
//    CS_turnOnLFXT( CS_LFXT_DRIVE_3 );
//}
//
///*
// * UART Communication Initialization
// */
//void Init_UART_A0()
//{
//    // Configure UART 9600 bit per second at 8MHz clock source
//    EUSCI_A_UART_initParam param = { 0 };
//    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
//    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
//    param.firstModReg |= ( UCBRF0 );
//    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
//    param.parity |= EUSCI_A_UART_NO_PARITY;
//    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
//    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
//    param.uartMode |= EUSCI_A_UART_MODE;
//    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
//
//    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A0_BASE, &param ) )
//    {
//        return;
//    }
//
//    EUSCI_A_UART_enable( EUSCI_A0_BASE );
//
//    EUSCI_A_UART_clearInterrupt( EUSCI_A0_BASE,
//    EUSCI_A_UART_RECEIVE_INTERRUPT );
//
//    // Enable USCI_A0 RX interrupt
//    EUSCI_A_UART_enableInterrupt( EUSCI_A0_BASE,
//    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt
//
//    // Enable global interrupt
//    __enable_interrupt();
//}
//
//void Init_UART_A3()
//{
//    // Configure UART 9600 bit per second at 8MHz clock source
//    EUSCI_A_UART_initParam param = { 0 };
//    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
//    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
//    param.firstModReg |= ( UCBRF0 );
//    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
//    param.parity |= EUSCI_A_UART_NO_PARITY;
//    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
//    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
//    param.uartMode |= EUSCI_A_UART_MODE;
//    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
//
//    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A3_BASE, &param ) )
//    {
//        return;
//    }
//
//    EUSCI_A_UART_enable( EUSCI_A3_BASE );
//
//    EUSCI_A_UART_clearInterrupt( EUSCI_A3_BASE,
//    EUSCI_A_UART_RECEIVE_INTERRUPT );
//
//    // Enable USCI_A3 RX interrupt
//    EUSCI_A_UART_enableInterrupt( EUSCI_A3_BASE,
//    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt
//
//    // Enable global interrupt
//    __enable_interrupt();
//}
//
///*
// * Real Time Clock Initialization
// */
//void Init_RTC(Calendar* pCalendar)
//{
//    // Initialize RTC with the specified Calendar above
//    RTC_C_initCalendar( RTC_C_BASE, pCalendar,
//    RTC_C_FORMAT_BINARY );
//
//    //Start RTC Clock
//    RTC_C_startClock( RTC_C_BASE );
//}
//
//void delayMilis( uint32_t u32NumOfMiliseconds )
//{
//    uint32_t u32pomMilis = u32NumOfMiliseconds * 8;
//
//    do
//    {
//        _delay_cycles( 1000 );
//    }
//    while( u32pomMilis-- > 0 );
//}
//
//
//
