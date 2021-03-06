/*******************************************************************************
 * @filename: clock.h
 * @Author	: fitz-nguyen
 * @Date	: Jun 10, 2017
 * @Email	: powerelectronics.vn@gmail.com	
 ******************************************************************************/
#ifndef KERNEL_INIT_CLOCK_H_
#define KERNEL_INIT_CLOCK_H_
/*******************************************************************************
 *                               HISTORY
 *	Jun 10, 2017		1.0.0	fitz-nguyen	: Initial revision
 ******************************************************************************/

/******************************************************************************/
/*                               INCLUDE                                      */
/******************************************************************************/
#include "platform.h"
#include "stm32f0xx_rcc.h"
/******************************************************************************/
/*                             PUBLIC TYPEDEF                                 */
/******************************************************************************/

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
 * @brief Initializing peripherals clock
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @details: To initializing peripherals clock
 */
PUBLIC void hwClockConfig(void);

#endif /* KERNEL_INIT_CLOCK_H_ */
/******************************************************************************/
/**!                           END OF FILE                                    */
/******************************************************************************/
/*                   All rights reserved � 2017 PE JSC                        */
/******************************************************************************/
