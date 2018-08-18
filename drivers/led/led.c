/*
 * @filename: led.c
 * @date    : 5/5/2018
 */
/******************************************************************************/
/**!                               INCLUDE                                    */
/******************************************************************************/
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "led.h"
/******************************************************************************/
/**!                            LOCAL TYPEDEF                                 */
/******************************************************************************/

/******************************************************************************/
/**!                            LOCAL SYMBOLS                                 */
/******************************************************************************/
#define LED_PWM_GPIO_PORT       GPIOA
#define LED_PWM_GPIO_PIN        GPIO_Pin_7
#define LED_PWM_PIN_SRC         GPIO_PinSource7
#define LED_PWM_GPIO_AF_FUNC    GPIO_AF_1
#define LED_PWM_TIMER           TIM3
#define LED_FREQ                1000

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
/*
 * @brief Initialize LED driver (control by PWM method)
 * @param none
 * @return none
 */
void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	uint16_t TIM_Period;
	/* Get clock freq */
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	TIM_Period = (uint16_t)(RCC_ClocksStatus.PCLK_Frequency/LED_FREQ)-1;

	/* Initialize GPIO */
	GPIO_InitStruct.GPIO_Pin   = LED_PWM_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(LED_PWM_GPIO_PORT, &GPIO_InitStruct);
	GPIO_PinAFConfig(LED_PWM_GPIO_PORT, LED_PWM_PIN_SRC, LED_PWM_GPIO_AF_FUNC);

	/* Initialize timer base configuration */
	TIM_BaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_BaseInitStruct.TIM_Prescaler         = 0;
	TIM_BaseInitStruct.TIM_Period            = TIM_Period;
	TIM_BaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_BaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(LED_PWM_TIMER, &TIM_BaseInitStruct);
	TIM_Cmd(LED_PWM_TIMER, ENABLE);
}

/*
 * @brief Dim LED by setting PWM duty cycle
 * @param[in] PWM duty cycle (from 0 to 100)
 * @return none
 */
void Led_Dim (uint8_t duty_cycle)
{
	TIM_OCInitTypeDef TIM_OCInitStruct;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	uint16_t TIM_Period;
	uint32_t Pulse_length;
	/* Get clock freq */
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	TIM_Period = (uint16_t)(RCC_ClocksStatus.PCLK_Frequency/LED_FREQ)-1;
	Pulse_length = ((TIM_Period + 1) * duty_cycle)/100 - 1;

	TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse       = Pulse_length;
	TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC1Init(LED_PWM_TIMER, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(LED_PWM_TIMER, TIM_OCPreload_Enable);
}
/******************************************************************************/
/**!                             END OF FILE                                  */
/******************************************************************************/
