/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32746g_discovery.h"
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOB
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Definition for I2Cx_MASTER clock resources */
#define I2Cx_MASTER                             I2C1
#define RCC_PERIPHCLK_I2Cx_MASTER               RCC_PERIPHCLK_I2C1
#define RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_PCLK1
#define I2Cx_MASTER_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_MASTER_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_MASTER_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_MASTER_DMA_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()

#define I2Cx_MASTER_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_MASTER_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx_SLAVE Pins */
#define I2Cx_MASTER_SCL_PIN                     GPIO_PIN_8
#define I2Cx_MASTER_SDA_PIN                     GPIO_PIN_9
#define I2Cx_MASTER_SCL_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SDA_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SCL_SDA_AF                  GPIO_AF4_I2C1

/* Definition for I2Cx_MASTER's NVIC */
#define I2Cx_MASTER_EV_IRQn                     I2C1_EV_IRQn
#define I2Cx_MASTER_ER_IRQn                     I2C1_ER_IRQn
#define I2Cx_MASTER_EV_IRQHandler               I2C1_EV_IRQHandler
#define I2Cx_MASTER_ER_IRQHandler               I2C1_ER_IRQHandler

/* Definition for I2Cx_SLAVE's DMA */
#define I2Cx_MASTER_DMA                         DMA1   
#define I2Cx_MASTER_DMA_INSTANCE_TX             DMA1_Stream6
#define I2Cx_MASTER_DMA_INSTANCE_RX             DMA1_Stream0
#define I2Cx_MASTER_DMA_CHANNEL_TX              DMA_CHANNEL_1
#define I2Cx_MASTER_DMA_CHANNEL_RX              DMA_CHANNEL_1

/* Definition for I2Cx_SLAVE's DMA NVIC */
#define I2Cx_MASTER_DMA_TX_IRQn                 DMA1_Stream6_IRQn
#define I2Cx_MASTER_DMA_RX_IRQn                 DMA1_Stream0_IRQn
#define I2Cx_MASTER_DMA_TX_IRQHandler           DMA1_Stream6_IRQHandler
#define I2Cx_MASTER_DMA_RX_IRQHandler           DMA1_Stream0_IRQHandler


/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
