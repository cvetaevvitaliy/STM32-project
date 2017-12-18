/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t adc_buffer[2];
float value[2];
float read_values[2];
char str[5];
char vOut1[9],vOut2[9];
int voltage=50;
char voltage2[5];

float V_25 = 1.34;
float Slope = 4.3e-3;
float Vref = 3.36;
float V_sense;
float temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		if (hadc->Instance == ADC1)
			for (int i=0; i<2; i++)
		{
					value[i]=adc_buffer[i];
		}

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	
	

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,adc_buffer,2);
	
	ssd1306_Init();
	
	//--------------frame
	SSD1306_DrawLine(0,64,128,64,White);
	SSD1306_DrawLine(0,0,128,0,White);
	SSD1306_DrawLine(128,0,128,64,White);
	SSD1306_DrawLine(0,0,0,64,White);

//	SSD1306_DrawLine(0,15,92,15,White);
	SSD1306_DrawLine(0,33,128,33,White);
	
//	SSD1306_DrawLine(44,0,44,15,White);
//	SSD1306_DrawLine(92,0,92,33,White);
	
	//---------------frame end
	
	
	//---------------acum
	SSD1306_DrawLine(4,49,27,49,White);
	SSD1306_DrawLine(4,61,27,61,White);
	SSD1306_DrawLine(3, 50, 3, 60,White);
	SSD1306_DrawLine(28, 50, 28, 60,White);
	SSD1306_DrawLine(29, 52, 29, 58,White);
	SSD1306_DrawLine(30, 52, 30, 58,White);
	SSD1306_DrawLine(31, 53, 31, 57,White);
	//---------------acum end
	//---------------acum2
	SSD1306_DrawLine(4,47,27,47,White);
	SSD1306_DrawLine(4,35,27,35,White);
	SSD1306_DrawLine(3, 36, 3, 46,White);
	SSD1306_DrawLine(28, 36, 28, 46,White);
	SSD1306_DrawLine(29, 38, 29, 44,White);
	SSD1306_DrawLine(30, 38, 30, 44,White);
	SSD1306_DrawLine(31, 39, 31, 43,White);
	//---------------acum2 end
	
	
	
	
	//---------------raw data 1 acum
	SSD1306_DrawLine(5,51,5,59,White);
	SSD1306_DrawLine(6,51,6,59,White);
	SSD1306_DrawLine(7,51,7,59,White);
	SSD1306_DrawLine(8,51,8,59,White);
	SSD1306_DrawLine(9,51,9,59,White);
	SSD1306_DrawLine(10,51,10,59,White);
	SSD1306_DrawLine(11,51,11,59,White);
	SSD1306_DrawLine(12,51,12,59,White);
	SSD1306_DrawLine(13,51,13,59,White);
	SSD1306_DrawLine(14,51,14,59,White);
	SSD1306_DrawLine(15,51,15,59,White);
	SSD1306_DrawLine(16,51,16,59,White);
	SSD1306_DrawLine(17,51,17,59,White);
	SSD1306_DrawLine(18,51,18,59,White);
	SSD1306_DrawLine(19,51,19,59,White);
	SSD1306_DrawLine(20,51,20,59,White);
	SSD1306_DrawLine(21,51,21,59,White);
	SSD1306_DrawLine(22,51,22,59,White);
	SSD1306_DrawLine(23,51,23,59,White);
	SSD1306_DrawLine(24,51,24,59,White);
	SSD1306_DrawLine(25,51,25,59,White);
	SSD1306_DrawLine(26,51,26,59,White);
	//---------------raw data acum 1 end
	
	//---------------raw data 2 acum
	SSD1306_DrawLine(5,37,5,45,White);
	SSD1306_DrawLine(6,37,6,45,White);
	SSD1306_DrawLine(7,37,7,45,White);
	SSD1306_DrawLine(8,37,8,45,White);
	SSD1306_DrawLine(9,37,9,45,White);
	SSD1306_DrawLine(10,37,10,45,White);
	SSD1306_DrawLine(11,37,11,45,White);
	SSD1306_DrawLine(12,37,12,45,White);
	SSD1306_DrawLine(13,37,13,45,White);
	SSD1306_DrawLine(14,37,14,45,White);
	SSD1306_DrawLine(15,37,15,45,White);
	SSD1306_DrawLine(16,37,16,45,White);
	SSD1306_DrawLine(17,37,17,45,White);
	SSD1306_DrawLine(18,37,18,45,White);
	SSD1306_DrawLine(19,37,19,45,White);
	SSD1306_DrawLine(20,37,20,45,White);
	SSD1306_DrawLine(21,37,21,45,White);
	SSD1306_DrawLine(22,37,22,45,White);
	SSD1306_DrawLine(23,37,23,45,White);
	SSD1306_DrawLine(24,37,24,45,White);
	SSD1306_DrawLine(25,37,25,45,White);
	SSD1306_DrawLine(26,37,26,45,White);
	//---------------raw data acum 2 end
	
	//---------------volt monitor 1
	ssd1306_SetCursor(35,37);
  ssd1306_WriteString("3.66v",Font_7x10,White);
	//---------------volt monitor 1 end
	
	
	//---------------volt monitor 2
	ssd1306_SetCursor(35,51);
  ssd1306_WriteString("3.66v",Font_7x10,White);
	//---------------volt monitor 2 end
	
//	ssd1306_SetCursor(45,6);
 // ssd1306_WriteString("3.10V",Font_16x26,White);
	
	ssd1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			read_values[1]=(value[1]/4096)*3.3;
			read_values[0]=(value[0]/4096)*3.3;
	
			
		
	if (read_values[0]>3.3){
		SSD1306_DrawLine(5,51,5,59,White);
	}
	if (read_values[0]>3.4){
		SSD1306_DrawLine(6,51,6,59,White);
		SSD1306_DrawLine(5,51,5,59,White);
	}
		if (read_values[0]>3.4){
		SSD1306_DrawLine(6,51,6,59,White);
		SSD1306_DrawLine(5,51,5,59,White);
	}	
		
			




		
	//	V_sense = value[1]/4096.0*Vref;
	//	temp = (V_25 - V_sense)/Slope + 25.0;
		
	//	sprintf(str,"%.2fV",read_values[1]);
//		sprintf(str,"%.2fC",temp);
	//	ssd1306_SetCursor(3,6);
	//	ssd1306_WriteString(str,Font_16x26,White);
	//	ssd1306_UpdateScreen();
		
		
						//------v out
					//	sprintf(vOut1,"%.2fV",read_values[0]);
					//	sprintf(vOut2,"%.2fV",read_values[1]);
					//	ssd1306_SetCursor(35,51);
					//	ssd1306_WriteString(vOut1,Font_7x10,White);
					//	ssd1306_SetCursor(35,37);
					//	ssd1306_WriteString(vOut2,Font_7x10,White);
					//	ssd1306_UpdateScreen();
						//------v out end
		Vout();
		if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) 
						{
						voltage++;
            } 
				
		if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) 
						{
							voltage=voltage-2;
						}
		
					sprintf(voltage2,"%dW",voltage);
					ssd1306_SetCursor(5,5);
					ssd1306_WriteString(voltage2,Font_16x25,White);
					ssd1306_UpdateScreen();
				
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Vout(void)
{
						sprintf(vOut1,"%.2fV",read_values[0]);
						sprintf(vOut2,"%.2fV",read_values[1]);
						ssd1306_SetCursor(35,51);
						ssd1306_WriteString(vOut1,Font_7x10,White);
						ssd1306_SetCursor(35,37);
						ssd1306_WriteString(vOut2,Font_7x10,White);
						ssd1306_UpdateScreen();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
