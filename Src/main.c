
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include <stdio.h>
#include "stm32f7xx_it.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
#define I2CX_TIMING             0x40912732 //0x40912732 //0x00303D5D; 0x00A0689A
#define I2C_ADDRESS 			0xD0
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//void PB14_GENERATE_PULSE(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void log_init(void)
{
	huart1.Instance        = USARTx;

	huart1.Init.BaudRate   = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits   = UART_STOPBITS_1;
	huart1.Init.Parity     = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	huart1.Init.Mode       = UART_MODE_TX_RX;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
	if(HAL_UART_DeInit(&huart1) != HAL_OK)
	{
		// Error_Handler();
	}  
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		// Error_Handler();
	}
}

int _write(int fd, char * str, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)str, len , 100);
	return len;
}

/**
  * @brief  This function configure I2C1 bus.
  * @param  None
  * @retval None
  */
void I2C1_Init(void)
{
	/*##Configure the I2C peripheral ######################################*/
	hi2c1.Instance              = I2Cx_MASTER;
	hi2c1.Init.Timing           = I2CX_TIMING;
	hi2c1.Init.OwnAddress1      = I2C_ADDRESS;
//	hi2c1.Init.OwnAddress2     = 0xFF;
	hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		/* Initialization Error */
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

void BB_GPIO_Init(void)
{

	GPIO_InitTypeDef	GPIO_Init;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//Configure PI1 Pin as input floating
	GPIO_Init.Pin       = GPIO_PIN_1;
	GPIO_Init.Mode      = GPIO_MODE_IT_FALLING;
	GPIO_Init.Pull      = GPIO_NOPULL;
 	HAL_GPIO_Init(GPIOI, &GPIO_Init);

 	HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
 	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

 	//Configure PB14 Pin as ouput 
 	GPIO_Init.Pin = GPIO_PIN_14;
 	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
 	GPIO_Init.Pull = GPIO_PULLUP;
 	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
 	HAL_GPIO_Init(GPIOB, &GPIO_Init);
}

void PB14_GENERATE_PULSE(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

}
/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
 	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 	HAL_Init();

 	/* Configure the system clock */
 	SystemClock_Config();

	/* Configure LED1 */
	BSP_LED_Init(LED1);
	/* Configure User push-button button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	log_init();
	I2C1_Init();
	BB_GPIO_Init();

	printf("RESET FIRMWARE\r\n");
	while (1)
	{

	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }  
}


/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
	* 1- When Slave don't acknowledge it's address, Master restarts communication.
	* 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
	*/
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
	Error_Handler();
  }
}

void EXTI15_10_IRQHandler(void)
{
	//if button pressed
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
	{
		printf("SEND I2C Data to BBB \r\n");
		PB14_GENERATE_PULSE();

		aTxBuffer[0] = 0x04;
		aTxBuffer[1] = 0x05;
		if(HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t*)aTxBuffer, 2, 10000)!= HAL_OK)
		{
 			/* Transfer error in transmission process */
 			printf("error transfer\r\n");
		}
	}
	
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
}

void EXTI1_IRQHandler(void)
{
	//if button pressed
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
		printf("Receive I2C Data from BBB\r\n");
		if(HAL_I2C_Slave_Receive(&hi2c1, (uint8_t *)aRxBuffer, 2, 10000) == HAL_OK) {
				printf("%#x\r\n", aRxBuffer[0]);
				printf("%#x\r\n", aRxBuffer[1]);
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
