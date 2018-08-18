/*
 * @filename: clock.c
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
#include "clock.h"
/******************************************************************************/
/**!                            LOCAL TYPEDEF                                 */
/******************************************************************************/

/******************************************************************************/
/**!                            LOCAL SYMBOLS                                 */
/******************************************************************************/

/******************************************************************************/
/**!                         EXPORTED VARIABLES                               */
/******************************************************************************/

/******************************************************************************/
/**!                          LOCAL VARIABLES                                 */
/******************************************************************************/

/******************************************************************************/
/**!                    LOCAL FUNCTIONS PROTOTYPES                            */
/******************************************************************************/

/******************************************************************************/
/**!                        EXPORTED FUNCTIONS                                */
/******************************************************************************/
/**
 * @brief Initializing peripherals clock
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @details: To initializing peripherals clock
 */
PUBLIC void hwClockConfig(void)
{
    //<! Enable system configure clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//<! GPIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//<! UART
    RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//<! TIMER
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	//<! I2C
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}
/******************************************************************************/
/**!                          LOCAL FUNCTIONS                                 */
/******************************************************************************/

/******************************************************************************/
/*                   All rights reserved © 2017 PE JSC                        */
/******************************************************************************/
