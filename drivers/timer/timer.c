/*******************************************************************************
 * @filename: timer.c
 * @Author	: fitz-nguyen
 * @Date	: Jun 26, 2017
 * @Email	: powerelectronics.vn@gmail.com	
 */
/******************************************************************************/
/**!                           REVISIONS HISTORY                              */
/******************************************************************************/
/**!
 *		Jun 26, 2017	1.0.0	fitz-nguyen	: Initial revision
 */
/******************************************************************************/
/**!                               INCLUDE                                    */
/******************************************************************************/
#include "timer.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_misc.h"
/******************************************************************************/
/**!                            LOCAL TYPEDEF                                 */
/******************************************************************************/
/**
 * @brief Software timer
 */
typedef struct
{
    uint32_t xInterval;
    uint32_t xTimeout;
    uint8_t  xStarted;
    void     (*callback)(void);
}xSWTimer_t;
/******************************************************************************/
/**!                            LOCAL SYMBOLS                                 */
/******************************************************************************/
#define SYS_TIMER_HW_UNIT           ( TIM16 )
#define SYS_TIMER_CLK               ( 1000000UL )
#define SYS_TIMER_NUM_SW_TIMER_MAX  ( 5 )
#define SYS_TIMER_IRQ_VECTOR        ( TIM16_IRQn )
/******************************************************************************/
/**!                         EXPORTED VARIABLES                               */
/******************************************************************************/

/******************************************************************************/
/**!                    LOCAL FUNCTIONS PROTOTYPES                            */
/******************************************************************************/
LOCAL volatile uint32_t _xSystemTimerMsCounter = 0;
LOCAL volatile uint8_t  _xSystemSWTimersNum = 0;
LOCAL xSWTimer_t        _xTimers[SYS_TIMER_NUM_SW_TIMER_MAX] = {{0}};
/******************************************************************************/
/**!                          LOCAL VARIABLES                                 */
/******************************************************************************/

/******************************************************************************/
/**!                        EXPORTED FUNCTIONS                                */
/******************************************************************************/
/**
 * @brief To create system RTC timer, tick count increase each millisecond
 */
PUBLIC void xSysTimerInit(void)
{
    TIM_TypeDef*            TIMx = NULL;
    uint16_t                TIM_Prescaler = 0;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
    NVIC_InitTypeDef        NVIC_InitStruct  = {0};
    RCC_ClocksTypeDef       RCC_ClocksStatus = {0};

    /* Get clock status */
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    /* Calculate pre-scaler */
    TIM_Prescaler = (uint16_t)(RCC_ClocksStatus.PCLK_Frequency/SYS_TIMER_CLK);
    /* Set timer's parameters */
    TIMx = SYS_TIMER_HW_UNIT;
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Prescaler         = TIM_Prescaler;
    TIM_TimeBaseInitStruct.TIM_Period            = 1000;  // 1000us = 1ms
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0; // There is no meaningful
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIMx, ENABLE);
    /* Initialize NVIC's IRQ of used HW timer unit */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 5;
    NVIC_InitStruct.NVIC_IRQChannel = SYS_TIMER_IRQ_VECTOR;
    NVIC_Init(&NVIC_InitStruct);
    /* Reset all variable to default */
    _xSystemTimerMsCounter = 0;
    _xSystemSWTimersNum    = 0;
}
/**
 * @brief Create software timer unit
 * @param xInterval
 * @param Callback
 * @return
 *          -1: fail to create software timer
 *          0..SYS_TIMER_NUM_SW_TIMER_MAX: success
 */
PUBLIC int xCreateSwTimer(uint32_t xInterval, void (*Callback)(void))
{
    if ((xInterval == 0) || (NULL == Callback)) return -1;

    int i = 0;
    for (i = 0; i < SYS_TIMER_NUM_SW_TIMER_MAX; i++)
    {
        if (_xTimers[i].callback == NULL)
        {
            _xTimers[i].xInterval = xInterval;
            _xTimers[i].xTimeout  = 0;
            _xTimers[i].callback  = Callback;
            _xTimers[i].xStarted  = 0;
            _xSystemSWTimersNum++;
            return i;
        }
    }
    return -1;
}
/**
 * @brief Start indicated software timer
 * @param xTimerId
 * @return
 *          EOK: success
 *          -1 : fail
 */
PUBLIC int xTimerRun(int xTimerId)
{
    /* Check error */
    if ((xTimerId >= SYS_TIMER_NUM_SW_TIMER_MAX) || (xTimerId < 0)) return -1;
    /* Query timers list */
    if (_xTimers[xTimerId].callback != NULL)
    {
        /* Get current time-stamp */
        _xTimers[xTimerId].xTimeout  = xGetSystemMsTimer();
        /* Calculate timeout point */
        _xTimers[xTimerId].xTimeout += _xTimers[xTimerId].xInterval;
        /* Indicate software timer has been started */
        _xTimers[xTimerId].xStarted  = 1;
    }
    return EOK;
}
/**
 * @brief Get system timer counter value in milliseconds
 * @return
 */
PUBLIC uint32_t xGetSystemMsTimer(void)
{
    return _xSystemTimerMsCounter;
}
/**
 * @brief Timer 15 Interrupt Service Routine.
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @details: External Interrupt Service Routine.
 */
HIDDEN_PROTOTYPE void TIM16_IRQHandler(void)
{
    TIM_TypeDef* TIMx = SYS_TIMER_HW_UNIT;
    int i = 0;
    /* Increase system timer counter */
    _xSystemTimerMsCounter++;
    /* Call software handler */
    for (i = 0; i < SYS_TIMER_NUM_SW_TIMER_MAX; i++)
    {
        if ((_xTimers[i].callback != NULL) && (_xTimers[i].xStarted == 1))
        {
            /* Check timeout condition */
            if (_xSystemTimerMsCounter == _xTimers[i].xTimeout)
            {
                /* Call callback */
                _xTimers[i].callback();
                /* Reset to delete current software timer */
                _xTimers[i].xStarted  = 0;
                _xTimers[i].xInterval = 0;
                _xTimers[i].xTimeout  = 0;
                _xTimers[i].callback  = NULL;
            }
        }
    }
    /* Clear interrupt flag of timer overflow */
    TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
}
/******************************************************************************/
/**!                          LOCAL FUNCTIONS                                 */
/******************************************************************************/

/******************************************************************************/
/*                   All rights reserved © 2017 PE JSC                        */
/******************************************************************************/
