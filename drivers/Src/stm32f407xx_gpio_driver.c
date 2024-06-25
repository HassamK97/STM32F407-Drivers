/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 6, 2024
 *      Author: Hassam
 */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - vGPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - None

 */
void vGPIO_PeriClockControl(GPIO_Reg_Def *pGPIOx, uint8_t unEnDis)
{
	if(unEnDis == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN;
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN;
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN;
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN;
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN;
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN;
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN;
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN;
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI;
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI;
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI;
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI;
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI;
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI;
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI;
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI;
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI;
		}
	}
}

/*********************************************************************
 * @fn      		  - vGPIO_Init
 *
 * @brief             - This function initializes the given GPIO pin based various settings like input/output etc.
 *
 * @param[in]         -	Handler structure of GPIO
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_Init(GPIO_Handler_t *pGPIOHandle)
{
	uint32_t ulTemp = 0;
	uint8_t unAltRegIdx, unAltFuncPin;

	//Enable the peripheral clock
	vGPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configure mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig.unPinMode <= GPIO_MODE_ANALOG)
	{
		// Non interrupt mode
		ulTemp = (pGPIOHandle->GPIO_PinConfig.unPinMode << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));	// Clearing current data
		pGPIOHandle->pGPIOx->MODER |= ulTemp; // Setting new data
	}
	else
	{
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.unPinMode <= GPIO_MODE_IT_FT)	// Falling Edge Trigger
		{
			// Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
			// Clear the corresponding bit on RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.unPinMode <= GPIO_MODE_IT_RT)	// Rising Edge Trigger
		{
			// Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
			// Clear the corresponding bit on FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.unPinMode <= GPIO_MODE_IT_RFT)	// Falling and Rising Edge Trigger
		{
			// Configure the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
		}

		// Configure the GPIO port selection in SYSCF EXTICR
		uint8_t unExticrIndex = pGPIOHandle->GPIO_PinConfig.unPinNumber / 4;
		uint8_t unExtiPinSel = pGPIOHandle->GPIO_PinConfig.unPinNumber % 4;
		uint8_t unPortCode = GPIO_BASEADDR_TO_PORT_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[unExticrIndex] = unPortCode << (unExtiPinSel * 4);

		// Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);
	}

	// Configure speed of GPIO Pin
	ulTemp = (pGPIOHandle->GPIO_PinConfig.unPinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));	// Clearing current data
	pGPIOHandle->pGPIOx->OSPEEDR |= ulTemp; // Setting new data

	// Configure PUPD settings
	ulTemp = (pGPIOHandle->GPIO_PinConfig.unPinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.unPinNumber));	// Clearing current data
	pGPIOHandle->pGPIOx->PUPDR |= ulTemp; // Setting new data

	// Configure OP_Type
	ulTemp = (pGPIOHandle->GPIO_PinConfig.unPinOP_Type << pGPIOHandle->GPIO_PinConfig.unPinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.unPinNumber);	// Clearing current data
	pGPIOHandle->pGPIOx->OTYPER |= ulTemp; // Setting new data

	// Configure the ALT functionality
	if(pGPIOHandle->GPIO_PinConfig.unPinAltFuncMode == GPIO_MODE_ALTFN)
	{
		unAltRegIdx = (pGPIOHandle->GPIO_PinConfig.unPinNumber / 8);
		unAltFuncPin = (pGPIOHandle->GPIO_PinConfig.unPinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[unAltRegIdx] &= ~(0xF << (4 * unAltFuncPin)); // Clearing current data
		pGPIOHandle->pGPIOx->AFR[unAltRegIdx] |= (pGPIOHandle->GPIO_PinConfig.unPinAltFuncMode << (4 * unAltFuncPin)); // Setting new data
	}
}

/*********************************************************************
 * @fn      		  - vGPIO_DeInit
 *
 * @brief             - This function deinitializes the given GPIO settings based on provided GPIO peripheral.
 *
 * @param[in]         -	Base address of GPIO peripheral
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_DeInit(GPIO_Reg_Def *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RST;
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RST;
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RST;
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RST;
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RST;
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RST;
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RST;
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RST;
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RST;
	}
}

/*********************************************************************
 * @fn      		  - unGPIO_ReadFromInputPin
 *
 * @brief             - Function to read data from GPIO as an input
 *
 * @param[in]         -	Base address of GPIO peripheral
 * @param[in]         -	Pin number of GPIO peripheral port
 *
 * @return            -	0 or 1
 *
 * @Note              -	None

 */
uint8_t unGPIO_ReadFromInputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum)
{
	uint8_t unVal = 0;

	unVal = (uint8_t)((pGPIOx->IDR >> unPinNum) & 0x00000001);

	return unVal;
}

/*********************************************************************
 * @fn      		  - udGPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -	Base address of GPIO peripheral
 *
 * @return            -	0 - 65535
 *
 * @Note              -	None

 */
uint16_t udGPIO_ReadFromInputPort(GPIO_Reg_Def *pGPIOx)
{
	uint16_t udVal = 0;

	// Reading complete data from the required GPIO port
	udVal = (uint16_t)(pGPIOx->IDR);

	return udVal;
}

/*********************************************************************
 * @fn      		  - vGPIO_WriteToOutputPin
 *
 * @brief             - Function to write data on a GPIO pin
 *
 * @param[in]         -	Base address of GPIO peripheral
 * @param[in]         -	Pin number of GPIO peripheral port
 * @param[in]         - GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_WriteToOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum, uint8_t unVal)
{
	if (unVal == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field of the corresponding pin number
		pGPIOx->ODR	|= (0x1 << unPinNum);
	}
	else
	{
		// Write 0 to the output data register at the bit field of the corresponding pin number
		pGPIOx->ODR	&= ~(0x1 << unPinNum);
	}
}

/*********************************************************************
 * @fn      		  - vGPIO_WriteToOutputPort
 *
 * @brief             -	Function to write data on a GPIO port
 *
 * @param[in]         -	Base address of GPIO peripheral
 * @param[in]         -	Data to write on GPIO peripheral port
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_WriteToOutputPort(GPIO_Reg_Def *pGPIOx, uint16_t udVal)
{
	pGPIOx->ODR	= (uint32_t)udVal;
}

/*********************************************************************
 * @fn      		  - vGPIO_ToggleOutputPin
 *
 * @brief             - Function to toggle the output of GPIO pin
 *
 * @param[in]         -	Base address of GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_ToggleOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum)
{
	pGPIOx->ODR ^= (0x1 << unPinNum);
}

/*********************************************************************
 * @fn      		  - vGPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -	GPIO peripheral port pin number on which interrupt requested
 * @param[in]         -	Prioroty of the requested interrupt
 * @param[in]         -	ENABLE or DISABLE macros
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_IRQConfig(uint8_t unInterruptNum, uint8_t unInterruptPriority, uint8_t unEnDis)
{

}

/*********************************************************************
 * @fn      		  - vGPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -	GPIO peripheral port pin number on which interrupt requested
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_IRQHandling(uint8_t unInterruptNum)
{

}
