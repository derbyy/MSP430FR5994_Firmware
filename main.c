#include "uart.h"

SunTimes_T sunTimes;
Calendar calendar;

static bool testSync = true;
static uint8_t u8Mode = 0x00;
/**
 * main.c
 */
void main( void )
{
    // Stop watchdog timer
    WDT_A_hold( WDT_A_BASE );
    uint8_t u8_Error = ERR_INIT_VAL;

    //Initialization
    Init_GPIO();
    Init_Clock();
    Init_UART_A0();
    Init_UART_A3();

    while( 1 )
    {
        if( b_UART_DataReceived )
        {
            GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
            u8_Error = uart_u8DateParser( &b_RTC_SynchronizationDone, &calendar, &sunTimes );
            GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
            ClearFlags();
        }
        if( /*( calendar.Hours == SYNC_TIME_HOUR )*/(  testSync ) /*|| ( u8_Error != ERR_NO_ERR )*/ )
        {
            //RTC_ESP8266_Synchronization();
            testSync = false;
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
            delayMilis(10);
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
            u8_RTC_ActualDay = calendar.DayOfMonth;
        }
//        if( ( calendar.DayOfMonth != u8_RTC_ActualDay ) && ( calendar.Hours == 0x00 ) && ( calendar.Minutes == 0x00 ) && ( calendar.Seconds ) >= 0x00 )
//        {
//            b_RTC_SynchronizationDone = false;
//        }
//        delayMilis( DELAY_SECONDS );
//        sendRTCtoPC();

    }
}

void sendRTCtoPC()
{
    uint8_t cnt;
    char buffer[ 100 ];
    memset( buffer, 0, sizeof( buffer ) );
    Calendar calendar = RTC_C_getCalendarTime( RTC_C_BASE );

    snprintf( buffer, sizeof( buffer ), "%04d%02d%02d%02d%02d%02d\r\n", calendar.Year, calendar.Month, calendar.DayOfMonth, calendar.Hours, calendar.Minutes, calendar.Seconds );
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );

    for( cnt = 0; cnt < strlen( buffer ); cnt++ )
    {
        // Send Ackknowledgement to Host PC
        EUSCI_A_UART_transmitData( EUSCI_A0_BASE, buffer[ cnt ] );
    }

    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A0_BASE, EUSCI_A_UART_BUSY ) )
        ;

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
}

uint8_t u8_Atoi( uint8_t* u8_Buffer, uint16_t* p_u16_Num )
{
    uint8_t u8_Error = ERR_INIT_VAL;
    uint16_t u16_Final_mun = 0;
    uint8_t u8_Cnt;
    *p_u16_Num = 0;

    for( u8_Cnt = 0; u8_Buffer[ u8_Cnt ] != 0x00; ++u8_Cnt )
    {
        if( u8_Buffer[ u8_Cnt ] >= 0x30 && u8_Buffer[ u8_Cnt ] <= 0x39 )
        {
            u16_Final_mun = u16_Final_mun * 10 + ( u8_Buffer[ u8_Cnt ] - 0x30 );
            u8_Error = ERR_NO_ERR;
        }
        else
        {
            u8_Error = ERR_ATOI_WRONG_VALUE;
            break;
        }
    }
    if( u8_Error == ERR_NO_ERR )
    {
        *p_u16_Num = u16_Final_mun;
    }

    return u8_Error;
}

void ClearFlags()
{
    memset( u8_UART_DataReceiveBuffer, 0, sizeof( u8_UART_DataReceiveBuffer ) );
    b_UART_DataReceived = FALSE;
    u8_UART_Index = 0x00;
}

uint8_t uart_u8DateParser(bool* bSyncDone, Calendar* pCalendar, SunTimes_T* pSuntimes )
{
    uint8_t u8_Error = ERR_INIT_VAL;
    uint8_t u8_ParseState = DATE_CRC;
    uint16_t u16_Number = 0;
    char pszDateArray[ NUM_OF_BYTES ];
    char* pszSubDateArray;

    memset( pszDateArray, 0, sizeof( pszDateArray ) );
    memcpy( pszDateArray, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) - 1 );

    //Received data should have the same size
    if( sizeof( pszDateArray ) != NUM_OF_BYTES )
    {
        return u8_Error = ERR_DATA_CORRUPTED;
    }

    unsigned char u8DataBuffer[ NUM_OF_BYTES ];
    unsigned char u8CalculatedCrc = 0x00;
    unsigned char u8ReceivedCrc = 0x00;

    //Set default values into array and copy received data without CRC and new line character
    memset( u8DataBuffer, 0, sizeof( u8DataBuffer ) );
    memcpy( u8DataBuffer, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) - 3 );

    //Calculate 8-bit CRC from received data
    u8CalculatedCrc = u8_Crc8( u8DataBuffer, NUM_OF_BYTES - 2 );

    //Get sended CRC to temp array
    memset( u8DataBuffer, 0, sizeof( u8DataBuffer ) );
    memcpy( u8DataBuffer, &u8_UART_DataReceiveBuffer[NUM_OF_BYTES - 2], 2 * sizeof(u8_UART_DataReceiveBuffer[0]) );
    //Convert last 2 bytes (CRC) into one hexadecimal number
    u8ReceivedCrc = u8_HexToBin( u8DataBuffer, 0x02 );

    //Check calculated CRC with received CRC
    if( u8CalculatedCrc == u8ReceivedCrc )
    {
        u8_ParseState = DATE_YEAR;
    }
    else
    {
       return u8_Error = ERR_DATA_CORRUPTED;
    }

    //Separate year from received data
    pszSubDateArray = strtok( pszDateArray, DELIMITER );
    u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
    if( u8_Error != ERR_NO_ERR )
    {
        return u8_Error;
    }
    pCalendar->Year = u16_Number;

    //Set next state of data parsing
    u8_ParseState = DATE_MONTH;

    while( pszSubDateArray != NULL )
    {
        pszSubDateArray = strtok( NULL, DELIMITER );
        if( pszSubDateArray != NULL )
        {
            if( u8_ParseState == DATE_FINISH )
            {
                u8_Error = ERR_NO_ERR;
                Init_RTC( pCalendar );
                *bSyncDone = true;
                break;
            }
            u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
            if( u8_Error != ERR_NO_ERR )
            {
                break;
            }
            switch( (ParserState_T) u8_ParseState )
            {
                case DATE_MONTH :
                {
                    pCalendar->Month = (uint8_t) u16_Number;
                    u8_ParseState = DATE_DAY;
                    break;
                }
                case DATE_DAY :
                {
                    pCalendar->DayOfMonth = (uint8_t) u16_Number;
                    u8_ParseState = DATE_HOUR;
                    break;
                }
                case DATE_HOUR :
                {
                    pCalendar->Hours = (uint8_t) u16_Number;
                    u8_ParseState = DATE_MINUTE;
                    break;
                }
                case DATE_MINUTE :
                {
                    pCalendar->Minutes = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SECOND;
                    break;
                }
                case DATE_SECOND :
                {
                    pCalendar->Seconds = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNRISE_HOUR;
                    break;
                }
                case DATE_SUNRISE_HOUR :
                {
                    pSuntimes->u8_Sunrise_Hour = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNRISE_MINUTE;
                    break;
                }
                case DATE_SUNRISE_MINUTE :
                {
                    pSuntimes->u8_Sunrise_Minute = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNSET_HOUR;
                    break;
                }
                case DATE_SUNSET_HOUR :
                {
                    pSuntimes->u8_Sunset_Hour = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNSET_MINUTE;
                    break;
                }
                case DATE_SUNSET_MINUTE :
                {
                    pSuntimes->u8_Sunset_Minute = (uint8_t) u16_Number;
                    u8_ParseState = DATE_FINISH;
                    break;
                }
                default :
                {
                    u8_Error = ERR_UNDEFINED_STATE;
                    break;
                }
            }
        }
        else
        {
            u8_Error = ERR_DATA_CORRUPTED;
            break;
        }
    }
    return u8_Error;
}

bool UART_DataExtract( uint16_t u16_UART_Buffer, uint8_t* u8_UART_DataBuffer )
{
    bool b_UART_BufferIsReceived = FALSE;

    u8_UART_DataBuffer[ u8_UART_Index++ ] = u16_UART_Buffer;
    if( u8_UART_DataBuffer[ u8_UART_Index - 1 ] == 0x0A )
    {
        return b_UART_BufferIsReceived = TRUE;
    }

    return b_UART_BufferIsReceived;
}

unsigned char u8_Crc8( unsigned char* strData, uint8_t u8Length )
{
    unsigned char crc = 0x00;
    uint8_t u8Cnt;

    if( strData == NULL )
    {
        return 0;
    }
    crc &= 0xff;
    for( u8Cnt = 0; u8Cnt < u8Length; u8Cnt++ )
    {
        crc = crc8_table[ crc ^ strData[ u8Cnt ] ];
    }
    return crc;
}

unsigned char u8_HexToBin( unsigned char* crcBuffer, uint8_t u8Length )
{
    unsigned char u8MSB;
    unsigned char u8LSB;
    uint8_t u8Cnt;

    for(u8Cnt = 0; u8Cnt < u8Length; u8Cnt++)
    {
        if( u8Cnt == 0x00 )
        {
            u8MSB = crcBuffer[ u8Cnt ];
            if( crcBuffer[ u8Cnt ] > '9' )
            {
                u8MSB -= 7;
            }
        }
        if( u8Cnt == 0x01 )
        {
            u8LSB = crcBuffer[ u8Cnt ];
            if( crcBuffer[ u8Cnt ] > '9' )
            {
                u8LSB -= 7;
            }
            break;
        }
    }
    return ( u8MSB << 4 ) | ( u8LSB & 0x0F );
}

void RTC_ESP8266_Synchronization()
{
    const char wakeUpCharacter = '!';
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P6, GPIO_PIN0,
    GPIO_PRIMARY_MODULE_FUNCTION );
    // Send Ackknowledgement to Host PC
    EUSCI_A_UART_transmitData( EUSCI_A3_BASE, wakeUpCharacter );

    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A3_BASE, EUSCI_A_UART_BUSY ) )
    {
        ;
    }

    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
}



/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );
    GPIO_setOutputHighOnPin( GPIO_PORT_P3, GPIO_PIN0 );

    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );

    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN1,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_PJ,
    GPIO_PIN4 + GPIO_PIN5,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_6 );
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource( 32768, 0 );
    //Set ACLK=LFXT
    CS_initClockSignal( CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    //Start XT1 with no time out
    CS_turnOnLFXT( CS_LFXT_DRIVE_3 );
}

/*
 * UART Communication Initialization
 */
void Init_UART_A0()
{
    // Configure UART 9600 bit per second at 8MHz clock source
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
    param.firstModReg |= ( UCBRF0 );
    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
    param.parity |= EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode |= EUSCI_A_UART_MODE;
    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A0_BASE, &param ) )
    {
        return;
    }

    EUSCI_A_UART_enable( EUSCI_A0_BASE );

    EUSCI_A_UART_clearInterrupt( EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT );

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt( EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt

    // Enable global interrupt
    __enable_interrupt();
}

void Init_UART_A3()
{
    // Configure UART 9600 bit per second at 8MHz clock source
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
    param.firstModReg |= ( UCBRF0 );
    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
    param.parity |= EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode |= EUSCI_A_UART_MODE;
    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A3_BASE, &param ) )
    {
        return;
    }

    EUSCI_A_UART_enable( EUSCI_A3_BASE );

    EUSCI_A_UART_clearInterrupt( EUSCI_A3_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT );

    // Enable USCI_A3 RX interrupt
    EUSCI_A_UART_enableInterrupt( EUSCI_A3_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt

    // Enable global interrupt
    __enable_interrupt();
}

/*
 * Real Time Clock Initialization
 */
void Init_RTC(Calendar* pCalendar)
{
    // Initialize RTC with the specified Calendar above
    RTC_C_initCalendar( RTC_C_BASE, pCalendar,
    RTC_C_FORMAT_BINARY );

    //Start RTC Clock
    RTC_C_startClock( RTC_C_BASE );
}

void delayMilis( uint32_t u32NumOfMiliseconds )
{
    uint32_t u32pomMilis = u32NumOfMiliseconds * 8;

    do
    {
        _delay_cycles( 1000 );
    }
    while( u32pomMilis-- > 0 );
}

/*
 * ADC 12-bit Initialization
 */
//void Init_ADC12()
//{
//    /* Initialize ADC12B module */
//    ADC12_B_initParam adc_initParam = { 0 };
//    adc_initParam.clockSourceSelect |= ADC12_B_CLOCKSOURCE_ACLK;
//    adc_initParam.clockSourceDivider |= ADC12_B_CLOCKDIVIDER_1;
//    adc_initParam.clockSourcePredivider |= ADC12_B_CLOCKPREDIVIDER__1;
//    adc_initParam.sampleHoldSignalSourceSelect |= ADC12_B_SAMPLEHOLDSOURCE_SC;
//    adc_initParam.internalChannelMap |= ADC12_B_MAPINTCH3;
//
//    ADC12_B_init( ADC12_B_BASE, &adc_initParam );
//
//    /* Enable ADC12B module */
//    ADC12_B_enable( ADC12_B_BASE );
//
//    /* Setup sampling rate */
//    ADC12_B_setupSamplingTimer( ADC12_B_BASE, ADC12_B_CYCLEHOLD_128_CYCLES, ADC12_B_CYCLEHOLD_128_CYCLES, ADC12_B_MULTIPLESAMPLESDISABLE );
//
//    /* Configure ADC12B module */
//    ADC12_B_configureMemoryParam adc_confMem = { 0 };
//    adc_confMem.refVoltageSourceSelect |= ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
//    adc_confMem.inputSourceSelect |= ADC12_B_INPUT_A12;
//    adc_confMem.memoryBufferControlIndex |= ADC12_B_MEMORY_0;
//    adc_confMem.differentialModeSelect |= ADC12_B_DIFFERENTIAL_MODE_DISABLE;
//    adc_confMem.endOfSequence |= ADC12_B_NOTENDOFSEQUENCE;
//    adc_confMem.windowComparatorSelect |= ADC12_B_WINDOW_COMPARATOR_DISABLE;
//
//    ADC12_B_configureMemory( ADC12_B_BASE, &adc_confMem );
//
//    /* Clear memory buffer 0 interrupt */
//    ADC12_B_clearInterrupt( ADC12_B_BASE, 0, ADC12_B_IFG0 );
//
//    /* Enable memory buffer 0 interrupt */
//    ADC12_B_enableInterrupt( ADC12_B_BASE, ADC12_B_IE0, 0, 0 );
//
//    /* Configure internal reference */
//    while( Ref_A_isRefGenBusy( REF_A_BASE ) );
//    Ref_A_setReferenceVoltage( REF_A_BASE, REF_A_VREF2_5V );
//    Ref_A_enableReferenceVoltage( REF_A_BASE );
//}

void liveTemp()
{
    //Initialize the ADC12B Module
    /*
     * Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Use Temperature Sensor and Battery Monitor internal channels
     */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ACLK;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_TEMPSENSEMAP;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Sets up the sampling timer pulse mode
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_128_CYCLES,
                               ADC12_B_CYCLEHOLD_128_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);

    // Maps Temperature Sensor input channel to Memory 0 and select voltage references
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A1 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_TCMAP;
    configureMemoryParam.refVoltageSourceSelect =
        ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect =
        ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect =
        ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    // Clear memory buffer 0 interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,
                           0,
                           ADC12_B_IFG0
                           );

    // Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
                            ADC12_B_IE0,
                            0,
                            0);

    // Configure internal reference
    while(Ref_A_isRefGenBusy(REF_A_BASE));              // If ref generator busy, WAIT
    Ref_A_enableTempSensor(REF_A_BASE);
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF2_0V);
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    // Start timer
    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 13;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &param);

    __bis_SR_register(LPM3_bits | GIE);       // Enter LPM3. Delay for Ref to settle.

    // Change timer delay to 1/8 second
    Timer_A_setCompareValue(TIMER_A0_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_0,
                            0x1000
                            );

    while(u8Mode == 0x01)
    {
        __bis_SR_register(LPM3_bits | GIE);   // Enter LPM3, wait for ~1/8 sec timer

        //Enable/Start sampling and conversion
        /*
         * Base address of ADC12B Module
         * Start the conversion into memory buffer 0
         * Use the single-channel, single-conversion mode
         */
        ADC12_B_startConversion(ADC12_B_BASE,
                                ADC12_B_MEMORY_0,
                                ADC12_B_SINGLECHANNEL);

        __bis_SR_register(LPM3_bits | GIE);   // Wait for conversion to complete
        __bic_SR_register(GIE);

        // Select UART TXD on P2.0
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM0_H);             // Send higher byte of temperature data
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM0_L);             // Send higher byte of temperature data

        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
        __no_operation();                     // Set a breakpoint here to verify UART transmission
    }

    // Disable ADC12 and Timer_A0
    ADC12_B_disable(ADC12_B_BASE);
    Timer_A_stop(TIMER_A0_BASE);
}



/*
 * USCI_A0 Interrupt Service Routine that receives PC GUI's commands
 */
#pragma vector = USCI_A3_VECTOR
__interrupt void USCI_A3_ISR( void )
{
    //char data;
    switch( __even_in_range( UCA3IV, USCI_UART_UCTXCPTIFG ) )
    {
        case USCI_NONE :
            break;
        case USCI_UART_UCRXIFG :
            b_UART_DataReceived = UART_DataExtract( UCA3RXBUF, (uint8_t*) &u8_UART_DataReceiveBuffer );
            __bic_SR_register_on_exit( LPM3_bits ); // Exit active CPU
            break;
        case USCI_UART_UCTXIFG :
            break;
        case USCI_UART_UCSTTIFG :
            break;
        case USCI_UART_UCTXCPTIFG :
            break;
    }
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR( void )
{
    //char data;
    switch( __even_in_range( UCA0IV, USCI_UART_UCTXCPTIFG ) )
    {
        case USCI_NONE :
            break;
        case USCI_UART_UCRXIFG :
            b_UART_DataReceived = true;
            __bic_SR_register_on_exit( LPM3_bits ); // Exit active CPU
            break;
        case USCI_UART_UCTXIFG :
            break;
        case USCI_UART_UCSTTIFG :
            break;
        case USCI_UART_UCTXCPTIFG :
            break;
    }
}

