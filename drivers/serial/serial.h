/*******************************************************************************
 * @filename: serial.h
 * @Author	: fitz-nguyen
 * @Date	: Jun 10, 2017
 * @Email	: powerelectronics.vn@gmail.com	
 ******************************************************************************/
#ifndef DRIVERS_SERIAL_SERIAL_H_
#define DRIVERS_SERIAL_SERIAL_H_
/*******************************************************************************
 *                               HISTORY
 *	Jun 10, 2017		1.0.0	fitz-nguyen	: Initial revision
 ******************************************************************************/

/******************************************************************************/
/*                               INCLUDE                                      */
/******************************************************************************/
#include "platform.h"
#include "FreeRTOS.h"
#include "queue.h"
/******************************************************************************/
/*                             PUBLIC TYPEDEF                                 */
/******************************************************************************/
/**!
 * Serial baud rate
 */
typedef enum
{
	SERIAL_BAUDRATE_115200 = 115200UL,
	SERIAL_BAUDRATE_460800 = 460800UL,
	SERIAL_BAUDRATE_921600 = 921600UL
}baudrate_t;
typedef enum
{
	SERIAL_ERR_NONE      = 0,
	SERIAL_ERR_OVERRUN   = 1,
	SERIAL_ERR_FRAME     = 2,
	SERIAL_ERR_IDLE_LINE = 3
}serial_error_t;
/**!
 * RX callback function prototype
 */
typedef void (*serial_callback_t)(uint8_t error, uint8_t byte);
/**!
 * Serial configuration structure
 */
typedef struct
{
	baudrate_t        baudrate;
	serial_callback_t callback;
	uint16_t          buffer_size;
}serial_t;
/******************************************************************************/
/**!                            PUBLIC SYMBOLS                                */
/******************************************************************************/

/******************************************************************************/
/**!                          PUBLIC VARIABLES                                */
/******************************************************************************/

/******************************************************************************/
/**!                          INLINE FUNCTIONS                                */
/******************************************************************************/

/******************************************************************************/
/**!                    PUBLIC FUNCTIONS PROTOTYPES                           */
/******************************************************************************/
/**
 * @brief Initializing serial port
 * @param[in]   serial	: serial configuration
 * @param[out]  none
 * @return      none
 * @details: To initializing serial hardware port before using
 */
PUBLIC void Serial_Init(serial_t* serial);
/**
 * @brief To send number of data bytes via serial port
 * @param[in]   data	: address of data package
 * @param[in]   length	: number of data bytes
 * @param[out]  none
 * @return      none
 * @details: To send number of data bytes via serial port
 */
PUBLIC void Serial_Send(uint8_t* data, uint16_t length);
#endif /* DRIVERS_SERIAL_SERIAL_H_ */
/******************************************************************************/
/**!                           END OF FILE                                    */
/******************************************************************************/
/*                   All rights reserved © 2017 PE JSC                        */
/******************************************************************************/
