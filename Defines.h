/*
 * Errors.h
 *
 *  Created on: 28. 7. 2018
 *  Author	  : Martin
 */

#ifndef DEFINES_H_
#define DEFINES_H_

/*-----------------ERRORS---------------------------------*/
#define ERR_INIT_VAL                (0xFF)
#define ERR_NO_ERR                  (0x00)
#define ERR_ATOI_WRONG_VALUE        (0x01)
#define ERR_DATA_CORRUPTED          (0x02)
#define ERR_UNDEFINED_STATE         (0x03)
/*--------------------------------------------------------*/


#define FALSE                       (0x00)
#define TRUE                        (0x01)
#define MAX_STR_LENGTH              (0x28)
#define DELAY_SECONDS               (1000)
#define NUM_OF_BYTES                (34)
#define DELIMITER                   ("-")
#define SYNC_TIME_HOUR              (4)


typedef struct SunTimes
{
    uint8_t u8_Sunrise_Hour;
    uint8_t u8_Sunrise_Minute;
    uint8_t u8_Sunset_Hour;
    uint8_t u8_Sunset_Minute;
} SunTimes_T;

typedef enum ParserState
{
    DATE_YEAR = 0x01,
    DATE_MONTH,
    DATE_DAY,
    DATE_HOUR,
    DATE_MINUTE,
    DATE_SECOND,
    DATE_SUNRISE_HOUR,
    DATE_SUNRISE_MINUTE,
    DATE_SUNSET_HOUR,
    DATE_SUNSET_MINUTE,
    DATE_CRC,
    DATE_FINISH
} ParserState_T;

#endif /* DEFINES_H_ */
