/*
 * @filename: serial.c
 * @Author	: fitz-nguyen
 * @Date	: Jun 10, 2017
 * @Email	: powerelectronics.vn@gmail.com
 */
/******************************************************************************/
/**!                           REVISIONS HISTORY                              */
/******************************************************************************/
/**!
 *		Jun 10, 2017	1.0.0	fitz-nguyen	: Initial revision
 */
/******************************************************************************/
/**!                               INCLUDE                                    */
/******************************************************************************/
//<! Standard include
#include <stdio.h>
//<! Upper layer to link definitions
#include "serial.h"
//<! Lower layer to import definitions
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_misc.h"
/******************************************************************************/
/**!                            LOCAL TYPEDEF                                 */
/******************************************************************************/
/**
 * Serial HW status
 */
typedef enum
{
	SERIAL_UNITIALIZED = 0,//!< SERIAL_UNITIALIZED
	SERIAL_INTIALIZED  = 1 //!< SERIAL_INTIALIZED
}serial_status_t;
/******************************************************************************/
/**!                            LOCAL SYMBOLS                                 */
/******************************************************************************/
#define UARTx_HW                          USART1
#define SERIAL_TX_PORT                    GPIOA
#define SERIAL_RX_PORT                    GPIOA
#define SERIAL_TX_PIN                     GPIO_Pin_2
#define SERIAL_RX_PIN                     GPIO_Pin_3
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/******************************************************************************/
/**!                         EXPORTED VARIABLES                               */
/******************************************************************************/

/******************************************************************************/
/**!                          LOCAL VARIABLES                                 */
/******************************************************************************/
LOCAL serial_callback_t _serial_callback = NULL;
LOCAL serial_status_t   _SerialStatus    = SERIAL_UNITIALIZED;
/******************************************************************************/
/**!                    LOCAL FUNCTIONS PROTOTYPES                            */
/******************************************************************************/
/**
 * @brief To one byte via serial port
 * @param[in]   byte    : one byte data
 * @param[out]  none
 * @return      EOK
 * @details: To one byte via serial port
 */
LOCAL int _serial_SendByte(uint8_t byte);
/******************************************************************************/
/**!                        EXPORTED FUNCTIONS                                */
/******************************************************************************/
/**
 * @brief To re-direct printf() function
 * @param[in]   file: one byte data
 * @param[in]   ptr : data address
 * @param[out]  len : data's length
 * @return      EOK
 * @details: To re-direct printf() function
 */
IMPORT int _write(int file, char* ptr, int len);
/**
 * @brief Initializing serial port
 * @param[in]   serial	: serial configuration
 * @param[out]  none
 * @return      none
 * @details: To initializing serial hardware port before using
 */
PUBLIC void Serial_Init(serial_t* serial)
{
	//<! Check serial status to avoid re-initialing serial hardware
	if (_SerialStatus != SERIAL_UNITIALIZED) return;

	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStruct;
	NVIC_InitTypeDef  NVIC_InitStruct;

	//<! Set configuration data USART, GPIO, Interrupt
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_BaudRate = serial->baudrate;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;

	//<! Initializing GPIO PA2, PA3
	GPIO_InitStruct.GPIO_Pin = SERIAL_TX_PIN;
	GPIO_PinAFConfig(SERIAL_TX_PORT, GPIO_PinSource2, GPIO_AF_1);
	GPIO_Init(SERIAL_TX_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SERIAL_RX_PIN;
	GPIO_PinAFConfig(SERIAL_RX_PORT, GPIO_PinSource3, GPIO_AF_1);
	GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStruct);

    //<! Initializing NVIC for USART1
    NVIC_Init(&NVIC_InitStruct);
    //<! Initializing UART
	USART_Init(UARTx_HW, &USART_InitStruct);
	USART_ITConfig(UARTx_HW, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UARTx_HW, USART_IT_ERR, ENABLE);

	USART_Cmd(UARTx_HW, ENABLE);

	//<! Initialize callback function
	_serial_callback = serial->callback;

	//<! Change serial hardware status
	_SerialStatus = SERIAL_INTIALIZED;
}
/**
 * @brief To send number of data bytes via serial port
 * @param[in]   data	: address of data package
 * @param[in]   length	: number of data bytes
 * @param[out]  none
 * @return      none
 * @details: To send number of data bytes via serial port
 */
PUBLIC void Serial_Send(uint8_t* data, uint16_t length)
{
	if (SERIAL_UNITIALIZED == _SerialStatus)
		assert_failed((uint8_t*)__FILE__, (uint32_t)__LINE__);
	//<! Sending data
	_write(0, (char*)data, length);
}

/**
 * @brief To one byte via serial port
 * @param[in]   byte    : one byte data
 * @param[out]  none
 * @return      EOK
 * @details: To one byte via serial port
 */
LOCAL int _serial_SendByte(uint8_t byte)
{
    while (SET != USART_GetFlagStatus(UARTx_HW, USART_FLAG_TXE));
    USART_SendData(UARTx_HW, byte);
    return EOK;
}

/**
 * @brief To re-direct printf() function
 * @param[in]   file: one byte data
 * @param[in]   ptr : data address
 * @param[out]  len : data's length
 * @return      EOK
 * @details: To re-direct printf() function
 */
PUBLIC int _write(int file, char* ptr, int len)
{
    file++;
    int i = 0;
    for (i = 0; i < len; i++)
        _serial_SendByte(ptr[i]);
    return EOK;
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    _serial_SendByte(ch);
  return ch;
}
/**
 * @brief UART1 Interrupt Service Routine
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @details: UART1 Interrupt Service Routine
 */
HIDDEN_PROTOTYPE void USART1_IRQHandler(void)
{
	//<! Get interrupt flag
	if (SET == USART_GetITStatus(UARTx_HW, USART_IT_RXNE))
	{
		//<! Get data
		uint8_t byte= (uint8_t)USART_ReceiveData(UARTx_HW) & 0xFF;
		//<! Call callback function to let other usage
		if (!_serial_callback) _serial_callback(SERIAL_ERR_NONE, byte);
	}
	//<! Get overrun interrupt flag
	else if (SET == USART_GetITStatus(UARTx_HW, USART_IT_ORE))
	{
		//<! Callback for upper layer
		if (!_serial_callback) _serial_callback(SERIAL_ERR_OVERRUN, 0);
	}
	//<! Get framing error interrupt flag
	else if (SET == USART_GetITStatus(UARTx_HW, USART_IT_FE))
	{
		//<! Callback for upper layer
		if (!_serial_callback) _serial_callback(SERIAL_ERR_FRAME, 0);
	}
	//<! Get Idle line error interrupt flag
	else if (SET == USART_GetITStatus(UARTx_HW, USART_IT_IDLE))
	{
		//<! Callback for upper layer
		if (!_serial_callback) _serial_callback(SERIAL_ERR_IDLE_LINE, 0);
	}
}
/******************************************************************************/
/**!                          LOCAL FUNCTIONS                                 */
/******************************************************************************/

/******************************************************************************/
/*                   All rights reserved � 2017 PE JSC                        */
/******************************************************************************/
