///*
// * uart.c
// *
// *  Created on: 1. 2. 2018
// *      Author: derby
// */
//#include "uart.h"
//
//void sendRTCtoPC()
//{
//    uint8_t cnt;
//    char buffer[ 100 ];
//    memset( buffer, 0, sizeof( buffer ) );
//    Calendar calendar = RTC_C_getCalendarTime( RTC_C_BASE );
//
//    snprintf( buffer, sizeof( buffer ), "%04d%02d%02d%02d%02d%02d\r\n", calendar.Year, calendar.Month, calendar.DayOfMonth, calendar.Hours, calendar.Minutes, calendar.Seconds );
//    // Select UART TXD on P2.0
//    GPIO_setAsPeripheralModuleFunctionOutputPin(
//    GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );
//
//    for( cnt = 0; cnt < strlen( buffer ); cnt++ )
//    {
//        // Send Ackknowledgement to Host PC
//        EUSCI_A_UART_transmitData( EUSCI_A0_BASE, buffer[ cnt ] );
//    }
//
//    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A0_BASE, EUSCI_A_UART_BUSY ) )
//        ;
//
//    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
//    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
//}
//
//uint8_t u8_Atoi( uint8_t* u8_Buffer, uint16_t* p_u16_Num )
//{
//    uint8_t u8_Error = ERR_INIT_VAL;
//    uint16_t u16_Final_mun = 0;
//    uint8_t u8_Cnt;
//    *p_u16_Num = 0;
//
//    for( u8_Cnt = 0; u8_Buffer[ u8_Cnt ] != 0x00; ++u8_Cnt )
//    {
//        if( u8_Buffer[ u8_Cnt ] >= 0x30 && u8_Buffer[ u8_Cnt ] <= 0x39 )
//        {
//            u16_Final_mun = u16_Final_mun * 10 + ( u8_Buffer[ u8_Cnt ] - 0x30 );
//            u8_Error = ERR_NO_ERR;
//        }
//        else
//        {
//            u8_Error = ERR_ATOI_WRONG_VALUE;
//            break;
//        }
//    }
//    if( u8_Error == ERR_NO_ERR )
//    {
//        *p_u16_Num = u16_Final_mun;
//    }
//
//    return u8_Error;
//}
//
//void ClearFlags()
//{
//    memset( u8_UART_DataReceiveBuffer, 0, sizeof( u8_UART_DataReceiveBuffer ) );
//    b_UART_DataReceived = FALSE;
//    u8_UART_Index = 0x00;
//}
//
//uint8_t uart_u8DateParser(bool* bSyncDone, Calendar* pCalendar, SunTimes_T* pSuntimes )
//{
//    uint8_t u8_Error = ERR_INIT_VAL;
//    uint8_t u8_ParseState = DATE_CRC;
//    uint16_t u16_Number = 0;
//    char pszDateArray[ NUM_OF_BYTES ];
//    char* pszSubDateArray;
//
//    memset( pszDateArray, 0, sizeof( pszDateArray ) );
//    memcpy( pszDateArray, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) - 1 );
//
//    //Received data should have the same size
//    if( sizeof( pszDateArray ) != NUM_OF_BYTES )
//    {
//        return u8_Error = ERR_DATA_CORRUPTED;
//    }
//
//    unsigned char u8DataBuffer[ NUM_OF_BYTES ];
//    unsigned char u8CalculatedCrc = 0x00;
//    unsigned char u8ReceivedCrc = 0x00;
//
//    //Set default values into array and copy received data without CRC and new line character
//    memset( u8DataBuffer, 0, sizeof( u8DataBuffer ) );
//    memcpy( u8DataBuffer, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) - 3 );
//
//    //Calculate 8-bit CRC from received data
//    u8CalculatedCrc = u8_Crc8( u8DataBuffer, NUM_OF_BYTES - 2 );
//
//    //Get sended CRC to temp array
//    memset( u8DataBuffer, 0, sizeof( u8DataBuffer ) );
//    memcpy( u8DataBuffer, &u8_UART_DataReceiveBuffer[NUM_OF_BYTES - 2], 2 * sizeof(u8_UART_DataReceiveBuffer[0]) );
//    //Convert last 2 bytes (CRC) into one hexadecimal number
//    u8ReceivedCrc = u8_HexToBin( u8DataBuffer, 0x02 );
//
//    //Check calculated CRC with received CRC
//    if( u8CalculatedCrc == u8ReceivedCrc )
//    {
//        u8_ParseState = DATE_YEAR;
//    }
//    else
//    {
//       return u8_Error = ERR_DATA_CORRUPTED;
//    }
//
//    //Separate year from received data
//    pszSubDateArray = strtok( pszDateArray, DELIMITER );
//    u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
//    if( u8_Error != ERR_NO_ERR )
//    {
//        return u8_Error;
//    }
//    pCalendar->Year = u16_Number;
//
//    //Set next state of data parsing
//    u8_ParseState = DATE_MONTH;
//
//    while( pszSubDateArray != NULL )
//    {
//        pszSubDateArray = strtok( NULL, DELIMITER );
//        if( pszSubDateArray != NULL )
//        {
//            if( u8_ParseState == DATE_FINISH )
//            {
//                u8_Error = ERR_NO_ERR;
//                Init_RTC( pCalendar );
//                *bSyncDone = true;
//                break;
//            }
//            u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
//            if( u8_Error != ERR_NO_ERR )
//            {
//                break;
//            }
//            switch( (ParserState_T) u8_ParseState )
//            {
//                case DATE_MONTH :
//                {
//                    pCalendar->DayOfMonth = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_DAY;
//                    break;
//                }
//                case DATE_DAY :
//                {
//                    pCalendar->DayOfMonth = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_HOUR;
//                    break;
//                }
//                case DATE_HOUR :
//                {
//                    pCalendar->Hours = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_MINUTE;
//                    break;
//                }
//                case DATE_MINUTE :
//                {
//                    pCalendar->Minutes = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_SECOND;
//                    break;
//                }
//                case DATE_SECOND :
//                {
//                    pCalendar->Seconds = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_SUNRISE_HOUR;
//                    break;
//                }
//                case DATE_SUNRISE_HOUR :
//                {
//                    pSuntimes->u8_Sunrise_Hour = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_SUNRISE_MINUTE;
//                    break;
//                }
//                case DATE_SUNRISE_MINUTE :
//                {
//                    pSuntimes->u8_Sunrise_Minute = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_SUNSET_HOUR;
//                    break;
//                }
//                case DATE_SUNSET_HOUR :
//                {
//                    pSuntimes->u8_Sunset_Hour = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_SUNSET_MINUTE;
//                    break;
//                }
//                case DATE_SUNSET_MINUTE :
//                {
//                    pSuntimes->u8_Sunset_Minute = (uint8_t) u16_Number;
//                    u8_ParseState = DATE_FINISH;
//                    break;
//                }
//                default :
//                {
//                    u8_Error = ERR_UNDEFINED_STATE;
//                    break;
//                }
//            }
//        }
//        else
//        {
//            u8_Error = ERR_DATA_CORRUPTED;
//            break;
//        }
//    }
//    return u8_Error;
//}
//
//bool UART_DataExtract( uint16_t u16_UART_Buffer, uint8_t* u8_UART_DataBuffer )
//{
//    bool b_UART_BufferIsReceived = FALSE;
//
//    u8_UART_DataBuffer[ u8_UART_Index++ ] = u16_UART_Buffer;
//    if( u8_UART_DataBuffer[ u8_UART_Index - 1 ] == 0x0A )
//    {
//        return b_UART_BufferIsReceived = TRUE;
//    }
//
//    return b_UART_BufferIsReceived;
//}
//
//unsigned char u8_Crc8( unsigned char* strData, uint8_t u8Length )
//{
//    unsigned char crc = 0x00;
//    uint8_t u8Cnt;
//
//    if( strData == NULL )
//    {
//        return 0;
//    }
//    crc &= 0xff;
//    for( u8Cnt = 0; u8Cnt < u8Length; u8Cnt++ )
//    {
//        crc = crc8_table[ crc ^ strData[ u8Cnt ] ];
//    }
//    return crc;
//}
//
//unsigned char u8_HexToBin( unsigned char* crcBuffer, uint8_t u8Length )
//{
//    unsigned char u8MSB;
//    unsigned char u8LSB;
//    uint8_t u8Cnt;
//
//    for(u8Cnt = 0; u8Cnt < u8Length; u8Cnt++)
//    {
//        if( u8Cnt == 0x00 )
//        {
//            u8MSB = crcBuffer[ u8Cnt ];
//            if( crcBuffer[ u8Cnt ] > '9' )
//            {
//                u8MSB -= 7;
//            }
//        }
//        if( u8Cnt == 0x01 )
//        {
//            u8LSB = crcBuffer[ u8Cnt ];
//            if( crcBuffer[ u8Cnt ] > '9' )
//            {
//                u8LSB -= 7;
//            }
//            break;
//        }
//    }
//    return ( u8MSB << 4 ) | ( u8LSB & 0x0F );
//}
//
//void RTC_ESP8266_Synchronization()
//{
//    const char wakeUpCharacter = '!';
//    // Select UART TXD on P2.0
//    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P6, GPIO_PIN0,
//    GPIO_PRIMARY_MODULE_FUNCTION );
//    // Send Ackknowledgement to Host PC
//    EUSCI_A_UART_transmitData( EUSCI_A3_BASE, wakeUpCharacter );
//
//    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A3_BASE, EUSCI_A_UART_BUSY ) )
//    {
//        ;
//    }
//
//    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
//    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
//}
//
//
