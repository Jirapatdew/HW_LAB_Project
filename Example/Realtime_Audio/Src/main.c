/**
  ******************************************************************************
  * @file    BSP/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.6
  * @date    06-May-2016
  * @brief   This example code shows how to use the STM324xG BSP Drivers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"
#include "stm32f4_discovery_audio.h"
#include "stdlib.h"
#include "string.h"
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void USART2_UART_Init(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t WrBuffer[WR_BUFFER_SIZE];
static uint16_t RecBuf[PCM_OUT_SIZE*2];
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];

/* Temporary data sample */
__IO uint32_t ITCounter = 0;
__IO uint32_t AudioDataReady = 0, AudioBuffOffset = 0;
__IO uint32_t ITOutAudio = 0,nextITOutAudio=0;

/* USER CODE END PV */



int main(void)
{ 
 /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4); 
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
	
	USART2_UART_Init();
  

	/* USER CODE BEGIN 2 */
//------------------- Audio --------------------------

	if(BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
    Error_Handler(); /* Record Error */
   
	/* Turn ON LED3: start record */
  BSP_LED_On(LED3); //red
	
   /* Start the record */
  if (BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], INTERNAL_BUFF_SIZE) != AUDIO_OK)
    Error_Handler(); /* Record Error */
	
	ITCounter = 0;

	
	/* Initialize audio IN at REC_FREQ */ 
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, DEFAULT_AUDIO_IN_FREQ);
	
	 /* start play*/ 
	BSP_AUDIO_OUT_Play((uint16_t*)&WrBuffer[0] , PCM_OUT_SIZE*3/2);
	
	while(1){
			
	}
	
	/* Stop audio record */
  if (BSP_AUDIO_IN_Stop() != AUDIO_OK)
    Error_Handler(); /* Record Error */

  /* Turn OFF LED3: record stopped */
  BSP_LED_Off(LED3);

//------------------- END Audio --------------------------
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/* USART2 init function */
static void USART2_UART_Init(void)
{

  huart2.Instance          = USART2;
  
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
}

/**
  * @brief  Toggle LEDs
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  BSP_LED_Toggle(LED3);
  HAL_Delay(100);
  BSP_LED_Toggle(LED4);
  HAL_Delay(100);
  BSP_LED_Toggle(LED5);
  HAL_Delay(100);
  BSP_LED_Toggle(LED6);
  HAL_Delay(100);
}

/* USER CODE BEGIN 4 */
/**
  * @brief Calculates the remaining file size and new position of the pointer.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* PDM to PCM data convert */
  BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[INTERNAL_BUFF_SIZE/2], (uint16_t*)&RecBuf[0]);
  
  /* Copy PCM data in internal buffer */ // 64 bytes copied
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
	
  if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {
		// 127 (next 4096 bytes / 2048 words (16bit) copied to WrBuffer //AudioDataReady = 1;
    ITCounter = 0;
  }
  else
  {
    ITCounter++;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /* PDM to PCM data convert */
	BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);
      
	/* Copy PCM data in internal buffer */ // 64 bytes copied
	memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
					
	if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {    
    ITCounter = 0;
  }
  else
  {		
    ITCounter++;
  }
}

/**
  * @brief  Audio IN Error callback function
  * @param  pData
  * @retval None
  */
void BSP_AUDIO_IN_Error_Callback(void)
{
  /* Stop the program with an infinite loop */
  Error_Handler();
}
/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
		BSP_LED_On(LED4);
  }
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/
/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
		BSP_AUDIO_OUT_Play((uint16_t*)&WrBuffer[ITOutAudio * (PCM_OUT_SIZE*2)], PCM_OUT_SIZE*3/2);//(*3/2,*3/2)  ,(*2,*3/2) normal ,(1,1)
	  
		if(nextITOutAudio == ITCounter){
			ITOutAudio = ITOutAudio;
		}
		else {
			ITOutAudio = nextITOutAudio;
			nextITOutAudio++;			//+=2 +=3
			nextITOutAudio%=(WR_BUFFER_SIZE/(PCM_OUT_SIZE*2));
		}
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  Error_Handler();
}



#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name/
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
