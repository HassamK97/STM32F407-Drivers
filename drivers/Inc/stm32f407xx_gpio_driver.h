/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 6, 2024
 *      Author: Hassam
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure of a GPIO
 */
typedef struct
{
	uint8_t unPinNumber;		/*!< Possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t unPinMode;			/*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t unPinSpeed;			/*!< Possible values from @GPIO_PIN_SPEEDS >*/
	uint8_t unPinAltFuncMode;
	uint8_t unPinPuPdControl;	/*!< Possible values from @GPIO_PIN_PULL_TYPES >*/
	uint8_t unPinOP_Type;		/*!< Possible values from @GPIO_PIN_OUTPUT_TYPES >*/
}GPIO_Config_t;

/*
 * Handler structure for a GPIO Pin
 */
typedef struct{
	GPIO_Reg_Def *pGPIOx;			/*!< For holding the base address of the GPIO port to which the pin belongs >*/
	GPIO_Config_t GPIO_PinConfig;	/*!< For holding GPIO pin configuration settings >*/
}GPIO_Handler_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0			 0
#define GPIO_PIN_1			 1
#define GPIO_PIN_2			 2
#define GPIO_PIN_3			 3
#define GPIO_PIN_4			 4
#define GPIO_PIN_5			 5
#define GPIO_PIN_6			 6
#define GPIO_PIN_7			 7
#define GPIO_PIN_8			 8
#define GPIO_PIN_9			 9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_SPEEDS
 * GPIO Pin Speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_FAST		3

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO Pin Output Types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_PULL_TYPES
 * GPIO Pin Pull Up and Pull Down Options
 */
#define GPIO_PIN_NO_PUPD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/******************************************************************************************
 *						APIs for performing certain GPIO control
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void vGPIO_PeriClockControl(GPIO_Reg_Def *pGPIOx, uint8_t unEnDis);

/*
 * GPIO Initialization and De-Initialization
 */
void vGPIO_Init(GPIO_Handler_t *pGPIOHandle);
void vGPIO_DeInit(GPIO_Reg_Def *pGPIOx);

/*
 * Data Read and Write Control
 */
uint8_t unGPIO_ReadFromInputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum);
uint16_t udGPIO_ReadFromInputPort(GPIO_Reg_Def *pGPIOx);
void vGPIO_WriteToOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum, uint8_t unVal);
void vGPIO_WriteToOutputPort(GPIO_Reg_Def *pGPIOx, uint16_t udVal);
void vGPIO_ToggleOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum);

/*
 * IRQ Configuration and ISR Handling
 */
void vGPIO_IRQConfig(uint8_t unInterruptNum, uint8_t unEnDis);
void vGPIO_IRQPriorityConfig(uint8_t unInterruptNum, uint8_t unInterruptPriority);
void vGPIO_IRQHandling(uint8_t unInterruptNum);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
