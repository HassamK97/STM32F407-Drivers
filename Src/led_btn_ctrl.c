/*
 * led_btn_ctrl.c
 *
 *  Created on: Jun 2, 2024
 *      Author: Hassam
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - vDelay
 *
 * @brief             - This function acts as a sleep function during execution of code.
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

	stGpioLed.pGPIOx = GPIOD;
	stGpioLed.GPIO_PinConfig.unPinMode = GPIO_MODE_OUT;
	stGpioLed.GPIO_PinConfig.unPinNumber = GPIO_PIN_15;
	stGpioLed.GPIO_PinConfig.unPinSpeed = GPIO_SPEED_HIGH;
	stGpioLed.GPIO_PinConfig.unPinOP_Type = GPIO_OP_TYPE_PP;
	stGpioLed.GPIO_PinConfig.unPinPuPdControl = GPIO_PIN_NO_PUPD;

	vGPIO_Init(&stGpioLed);	/* Initializing the GPIO For LED */

	stGpioBtn.pGPIOx = GPIOA;
	stGpioBtn.GPIO_PinConfig.unPinMode = GPIO_MODE_IN;
	stGpioBtn.GPIO_PinConfig.unPinNumber = GPIO_PIN_0;
	stGpioBtn.GPIO_PinConfig.unPinSpeed = GPIO_SPEED_HIGH;
	stGpioBtn.GPIO_PinConfig.unPinPuPdControl = GPIO_PIN_NO_PUPD;

	vGPIO_Init(&stGpioBtn); /* Initializing the GPIO For Button */

	while(1)
	{
		/* LED will turn on when the button is pressed */
		if(unGPIO_ReadFromInputPin(stGpioBtn.pGPIOx, GPIO_PIN_0) == ENABLE)
		{
			vGPIO_WriteToOutputPin(stGpioLed.pGPIOx, GPIO_PIN_15, ENABLE);
			vDelay();
		}
		vGPIO_WriteToOutputPin(stGpioLed.pGPIOx, GPIO_PIN_15, DISABLE);
	}

	return 0;
}



