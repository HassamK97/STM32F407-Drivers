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
 * @param[in]         - base address of GPIO peripheral
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
 * @param[in]         -	handler structure of GPIO
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_Init(GPIO_Handler_t *pGPIOHandle)
{
	//Enable the peripheral clock


	// Configure mode of GPIO Pin


	// Configure speed of GPIO Pin


	// Configure PUPD settings


	// Configure OP_Type


	// Configure the ALT functionality
}

/*********************************************************************
 * @fn      		  - vGPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -	base address of GPIO peripheral
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_DeInit(GPIO_Reg_Def *pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - unGPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -	base address of GPIO peripheral
 * @param[in]         -	pin number of GPIO peripheral port
 *
 * @return            -	uint8_t
 *
 * @Note              -	None

 */
uint8_t unGPIO_ReadFromInputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum)
{
	return 0;
}

/*********************************************************************
 * @fn      		  - udGPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -	base address of GPIO peripheral
 *
 * @return            -	uint16_t
 *
 * @Note              -	None

 */
uint16_t udGPIO_ReadFromInputPort(GPIO_Reg_Def *pGPIOx)
{
	return 0;
}

/*********************************************************************
 * @fn      		  - vGPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -	base address of GPIO peripheral
 * @param[in]         -	pin number of GPIO peripheral port
 * @param[in]         - GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_WriteToOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum, uint8_t unVal)
{

}

/*********************************************************************
 * @fn      		  - vGPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -	Base address of GPIO peripheral
 * @param[in]         -	data to write on GPIO peripheral port
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_WriteToOutputPort(GPIO_Reg_Def *pGPIOx, uint16_t udVal)
{

}

/*********************************************************************
 * @fn      		  - vGPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -	base address of GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -	void
 *
 * @Note              -	None

 */
void vGPIO_ToggleOutputPin(GPIO_Reg_Def *pGPIOx, uint8_t unPinNum)
{

}

/*********************************************************************
 * @fn      		  - vGPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -	GPIO peripheral port pin number on which interrupt requested
 * @param[in]         -	prioroty of the requested interrupt
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
