/*
 * button_interrupt_ctrl.c
 *
 *  Created on: Mar 16, 2025
 *      Author: Hassam
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH 			1
#define LOW 			0
#define BTN_PRESSED 	LOW

/*********************************************************************
 * @fn      		  - vDelay
 *
 * @brief             - This function introduces ~200ms delay when system clock is 16MHz
 *
 * @param[in]         -	None
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vDelay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*********************************************************************
 * @fn      		  - main
 *
 * @brief             - This is the function which executes the main logic and program starts from here.
 *
 * @param[in]         -	None
 *
 * @return            -	int
 *
 * @Note              -	None

 */
int main (void)
{
	GPIO_Handler_t stGpioLed, stGpioBtn;

	memset(&stGpioLed, 0, sizeof(stGpioLed));
	memset(&stGpioBtn, 0, sizeof(stGpioBtn));

	stGpioLed.pGPIOx = GPIOD;
	stGpioLed.GPIO_PinConfig.unPinMode = GPIO_MODE_OUT;
	stGpioLed.GPIO_PinConfig.unPinNumber = GPIO_PIN_15;
	stGpioLed.GPIO_PinConfig.unPinSpeed = GPIO_SPEED_HIGH;
	stGpioLed.GPIO_PinConfig.unPinOP_Type = GPIO_OP_TYPE_PP;
	stGpioLed.GPIO_PinConfig.unPinPuPdControl = GPIO_PIN_NO_PUPD;

	vGPIO_Init(&stGpioLed);	/* Initializing the GPIO For LED */

	stGpioBtn.pGPIOx = GPIOA;
	stGpioBtn.GPIO_PinConfig.unPinMode = GPIO_MODE_IT_FT;
	stGpioBtn.GPIO_PinConfig.unPinNumber = GPIO_PIN_0;
	stGpioBtn.GPIO_PinConfig.unPinSpeed = GPIO_SPEED_HIGH;
	stGpioBtn.GPIO_PinConfig.unPinPuPdControl = GPIO_PIN_NO_PUPD;

	vGPIO_Init(&stGpioBtn); /* Initializing the GPIO For Button */

	vGPIO_WriteToOutputPin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	// IRQ Configurations
	vGPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	vGPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	vDelay(); /* Added 200ms delay to account for button de-bouncing */
	vGPIO_IRQHandling(GPIO_PIN_0);
	vGPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);
}
