/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

uint8_t SPIx_WriteRead(uint8_t Byte);
void SPIx_Write(uint16_t Value);
uint32_t SPIx_Read(uint8_t ReadSize);
void SPIx_Error( );


	


/* USER CODE END 0 */



void doLatch()
{
	//Do Latch	
	HAL_GPIO_WritePin(GPIOA, MATRIX_ENABLE_Pin, GPIO_PIN_RESET );
	//HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, MATRIX_ENABLE_Pin, GPIO_PIN_SET );
}
#define LEFT 0
#define BOTTOM 1
#define RIGHT 2
#define TOP 3
int main(void)
{

  /* USER CODE BEGIN 1 */
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint16_t matrix_data[] = { 0xA0, 0x0A };
	int pos = 0;
	SPIx_Write( 0x0 );
	SPIx_Write( 0x0 );	
	doLatch();
	
	HAL_Delay(100);
	uint16_t data = matrix_data[0];
	uint8_t xpos = 0;
	uint8_t ypos = 0;
	int down = 1;
	uint16_t column=0;
	uint16_t row=0;
	
	//Fill from top left
	pos = 0;
	/*while( pos < 8 ){
		column = column | (1 << pos);
		row = row | (1 << pos);
		pos++;
		SPIx_Write( ~row );
		SPIx_Write( column );	
		doLatch();
		HAL_Delay(75);
	}
	HAL_Delay(100);
	//blink several times
	int delay=300;
	while( delay > 0 ){
		SPIx_Write( ~row );
		SPIx_Write( column );	
		doLatch();
		HAL_Delay(delay);
		SPIx_Write( 0 );
		SPIx_Write( 0 );	
		doLatch();
		HAL_Delay(delay);
		delay -=10;
	}*/
	
	pos = 0;
	
	column=0;
	row=0;
	
	int loop[] = {LEFT, BOTTOM, RIGHT, TOP};
	int activeSide=0;
	pos = 0;
	int ylimit=7;
	int delay=300;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//
		if( activeSide==0 && ylimit==7 && xpos==0
				&& ypos==0 ){
			//Fill from top left
			delay=300;
			pos = 0;
			column=0;
			row=0;
			while( pos < 8 ){
				column = column | (1 << pos);
				row = row | (1 << pos);
				pos++;
				SPIx_Write( ~row );
				SPIx_Write( column );	
				doLatch();
				HAL_Delay(75);
			}
			HAL_Delay(100);
			//blink several times
			int delay=300;
			while( delay > 0 ){
				SPIx_Write( ~row );
				SPIx_Write( column );	
				doLatch();
				HAL_Delay(delay);
				SPIx_Write( 0 );
				SPIx_Write( 0 );	
				doLatch();
				HAL_Delay(delay);
				delay -=10;
			}
			
			pos = 0;
			
			column=0;
			row=0;
		}
		
		
		switch( loop[activeSide] ){
			case LEFT:
				row = 1 << ypos;
				row = ~row;
				column = 1 << xpos;
				break;			
			case BOTTOM:
				column = 1 << ypos;				
				row = 0x80 >> xpos;
				row = ~row;
				break;
			case RIGHT:
				row = 0x80 >> ypos;
				row = ~row;
				column = 0x80 >> xpos;
				break;
			case TOP:
				column = 0x80 >> ypos;	
				row = 1 << xpos;
				row = ~row;
				break;
		}
		
		
		//Write to matrix
		SPIx_Write( row );
		SPIx_Write( column );
		doLatch();
		HAL_Delay(100);
		
		ypos++;		
		if( ypos > ylimit ){
			//time to switch side
			activeSide++;
			if( activeSide > 3 ){
				activeSide = 0;
				xpos++;
				ylimit--;
				if( xpos > ylimit ){
					xpos = 1;
				}
				if( ylimit < 1 ){
					ylimit = 7;
					xpos=0;
					activeSide = 0;
				}
			}
			ypos = xpos;
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USART1_TX
     PA10   ------> USART1_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : EEPROM_CS_Pin MATRIX_ENABLE_Pin */
  GPIO_InitStruct.Pin = EEPROM_CS_Pin|MATRIX_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SPIx_Error( )
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(&hspi1);
  
  /* Re- Initiaize the SPI comunication BUS */
  MX_SPI1_Init( );
}

/**
  * @brief SPI Read 4 bytes from device
  * @param  ReadSize Number of bytes to read (max 4 bytes)
  * @retval Value read on the SPI
  */
uint32_t SPIx_Read(uint8_t ReadSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue;
  
  status = HAL_SPI_Receive(&hspi1, (uint8_t*) &readvalue, ReadSize, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    SPIx_Error();
  }

  return readvalue;
}

/**
  * @brief SPI Write a byte to device
  * @param Value: value to be written
  * @retval None
  */
void SPIx_Write(uint16_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hspi1, (uint8_t*) &Value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    SPIx_Error();
  }
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
uint8_t SPIx_WriteRead(uint8_t Byte)
{

  uint8_t receivedbyte = 0;
  
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

/**
  * @brief SPI error treatment function
  * @param None
  * @retval None
  */


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
