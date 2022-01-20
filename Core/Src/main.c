/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXALLOWEDONCHANNELS 6
#define CH21STATEOFF 0
#define CH21STATEOUT 1
#define CH21STATESENSE 2
#define CH21REQOFF 0
#define CH21REQOUT 1
#define CH21REQSENSE 2
#define CH21NOREQ 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart4_tx;

/* USER CODE BEGIN PV */
uint64_t receivedSequence = 0;
uint8_t receivedCounter = 0;
int8_t numberOfChannelsActive = 0;
uint32_t channelState = 0;
uint8_t channel21State = CH21STATEOFF;
uint8_t channel21Req = CH21NOREQ;
uint8_t channel21DMMCommand = CH21REQOFF;
uint8_t uartsinglemessage[71], uartbuffer[2000], uartTransmitBuffer[2000];

GPIO_TypeDef* GPIOsequence[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
uint32_t PinSequence[] = {CH1_Pin, CH2_Pin, CH3_Pin, CH4_Pin, CH5_Pin, CH6_Pin, CH7_Pin, CH8_Pin, CH9_Pin, CH10_Pin, CH11_Pin, CH12_Pin, CH13_Pin, CH14_Pin, CH15_Pin, CH16_Pin, CH17_Pin, CH18_Pin, CH19_Pin, CH20_Pin};
uint8_t channelSequence[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 1, 2, 3, 4, 5, 6, 7 , 8, 9, 10, 21};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTSendDMA(void);
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
extern int __io_putchar(int ch) __attribute__((weak));
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
    //__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); // Disable Half Transfer Interrupt
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);  // turn on the LED, the logic is inverted
    printf("\n===============\nBooting\n");
    for (uint8_t i=0; i < 6; i++) {
        HAL_Delay(100); // sleep for 500 ms
        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
    }
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|CH20_Pin|CH19_Pin|CH18_Pin
                          |CH17_Pin|CH16_Pin|CH15_Pin|CH11_Pin
                          |Bus_Sense_Pin|CH1_Pin|CH2_Pin|CH3_Pin
                          |CH4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH14_Pin|CH13_Pin|CH12_Pin|CH5_Pin
                          |CH6_Pin|CH7_Pin|CH8_Pin|CH9_Pin
                          |CH10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Bus_In_GPIO_Port, Bus_In_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Data_Pin */
  GPIO_InitStruct.Pin = Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Clock_Pin Strobe_Pin */
  GPIO_InitStruct.Pin = Clock_Pin|Strobe_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CH20_Pin CH19_Pin CH18_Pin CH17_Pin
                           CH16_Pin CH15_Pin CH11_Pin Bus_Sense_Pin
                           CH1_Pin CH2_Pin CH3_Pin CH4_Pin */
  GPIO_InitStruct.Pin = CH20_Pin|CH19_Pin|CH18_Pin|CH17_Pin
                          |CH16_Pin|CH15_Pin|CH11_Pin|Bus_Sense_Pin
                          |CH1_Pin|CH2_Pin|CH3_Pin|CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CH14_Pin CH13_Pin CH12_Pin CH5_Pin
                           CH6_Pin CH7_Pin CH8_Pin CH9_Pin
                           CH10_Pin */
  GPIO_InitStruct.Pin = CH14_Pin|CH13_Pin|CH12_Pin|CH5_Pin
                          |CH6_Pin|CH7_Pin|CH8_Pin|CH9_Pin
                          |CH10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Bus_In_Pin */
  GPIO_InitStruct.Pin = Bus_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Bus_In_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == Clock_Pin) {
        receivedSequence = receivedSequence << 1;
        receivedSequence |= HAL_GPIO_ReadPin(GPIOB, Bus_Sense_Pin);
        receivedCounter++;
    }
    else if(GPIO_Pin == Strobe_Pin) {
        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
        uartbuffer[0] = '\0';

        if (receivedCounter != 48)	{
            sprintf((char *)uartsinglemessage, "Warning: 48 clocks not received, %d\n", receivedCounter);
            strcat((char *)uartbuffer, (char *)uartsinglemessage);
        }
        if (receivedSequence != 0x0) {
            // process the command
            uint8_t counter;
            for (counter = 0; counter < 20; counter++) {
                // Even clock pulses -> turn relays off
                if ((receivedSequence & ((uint64_t)1 << (2 * counter))) != 0) {
                    // turn off channel (GPIO_PIN_RESET -> LOW)
                    HAL_GPIO_WritePin(
                        GPIOsequence[channelSequence[counter]-1],
                        PinSequence[channelSequence[counter]-1],
                        GPIO_PIN_RESET
                    );
                    numberOfChannelsActive--;
                    channelState &= ~((long)1 << (channelSequence[counter] - 1));
                    sprintf((char *)uartsinglemessage, "CH%d OFF\n", channelSequence[counter]);
                    strcat((char *)uartbuffer, (char *)uartsinglemessage);
                    if (numberOfChannelsActive < 0) {
                        sprintf((char *)uartsinglemessage, "Warning too many ssr off requests\n");
                        strcat((char *)uartbuffer, (char *)uartsinglemessage);
                        numberOfChannelsActive = 0;
                     }
                }
                // Odd clock pulses -> turn relays on
                if ((receivedSequence & ((uint64_t)1 << (2 * counter + 1))) != 0) {
                    // turn on channel (GPIO_PIN_SET -> HIGH)
                    if (numberOfChannelsActive < MAXALLOWEDONCHANNELS) {
                        HAL_GPIO_WritePin(
                            GPIOsequence[channelSequence[counter] - 1], PinSequence[channelSequence[counter] - 1],
                            GPIO_PIN_SET
                        );
                        numberOfChannelsActive++;
                        channelState |= (1 << (channelSequence[counter] - 1));
                        sprintf((char *)uartsinglemessage, "CH%d ON\n", channelSequence[counter]);
                        strcat((char *)uartbuffer, (char *)uartsinglemessage);
                    }
                    else {
                        sprintf((char *)uartsinglemessage, "Error, maximum allowed of on ssrs reached. CH%d did not switch on\n", channelSequence[counter]);
                        strcat((char *)uartbuffer, (char *)uartsinglemessage);;
                    }
                }
            }
            // channel 21
            if ((channelState & 0xFFC00) == 0) {
                // If channels 11 to 20 not used, turn off both bus2 switches
                channel21Req = CH21REQOFF;
            }
            else {
                // But turn the bus on again, on the next command for CH11 to CH20 use
                channel21Req = channel21DMMCommand;
            }
            if ((receivedSequence & ((uint64_t)1<<(2*20))) !=0) {
                // switch bus2 to out
                channel21Req = CH21REQOUT;
                channel21DMMCommand = CH21REQOUT;
            }
            if ((receivedSequence & ((uint64_t)1<<(2*20+1))) !=0) {
                // switch bus2 to sense
                channel21Req = CH21REQSENSE;
                channel21DMMCommand = CH21REQSENSE;
            }

            if ((channel21Req == CH21REQOFF) && channel21State != CH21STATEOFF) {
                HAL_GPIO_WritePin(GPIOA, Bus_Sense_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, Bus_In_Pin, GPIO_PIN_RESET);
                channel21State = CH21STATEOFF;
                numberOfChannelsActive--;
                sprintf((char *)uartsinglemessage, "CH21 OFF\n");
                strcat((char *)uartbuffer, (char *)uartsinglemessage);
            }
            if ((channel21Req == CH21REQOUT) && (channel21State != CH21STATEOUT)) {
                if ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS - 1) || ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS) && (channel21State != CH21STATEOFF))) {
                    HAL_GPIO_WritePin(GPIOA, Bus_Sense_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, Bus_In_Pin, GPIO_PIN_SET);
                    if (channel21State == CH21STATEOFF) numberOfChannelsActive++;
                    channel21State = CH21STATEOUT;
                    sprintf((char *)uartsinglemessage, "CH21 OUT\n");
                    strcat((char *)uartbuffer, (char *)uartsinglemessage);
                }
                else {
                    sprintf((char *) uartsinglemessage, "Error, maximum allowed of on ssrs reached. Bus2 Mux did not switch on\n");
                    strcat((char *)uartbuffer, (char *)uartsinglemessage);
                }
            }
            if ((channel21Req == CH21REQSENSE) && channel21State != CH21STATESENSE ) {
                if ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS - 1) || ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS) && (channel21State != CH21STATEOFF))) {
                    HAL_GPIO_WritePin(GPIOC, Bus_In_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOA, Bus_Sense_Pin, GPIO_PIN_SET);
                    if (channel21State == CH21STATEOFF) numberOfChannelsActive++;
                    channel21State = CH21STATESENSE;
                    sprintf((char *)uartsinglemessage, "CH21 SENSE\n");
                    strcat((char *)uartbuffer, (char *)uartsinglemessage);
                }
                else {
                    sprintf((char *)uartsinglemessage, "Error, maximum allowed of on ssrs reached. Bus2 Mux did not switch on\n");
                    strcat((char *)uartbuffer, (char *)uartsinglemessage);
                }
            }
        }
        channel21Req = CH21NOREQ;
        receivedSequence = 0x0;
        receivedCounter = 0;
        if (uartbuffer[0] != '\0') {
            UARTSendDMA();
        }
    }
}

void UARTSendDMA(void) {
    // Block until the previous transmission is complete.
    // In the meantime the uartTransmitBuffer will keep buffering
    // all output
    while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) != 1);
    strcpy((char *)uartTransmitBuffer, (char *)uartbuffer);
    HAL_UART_Transmit_DMA(
        &huart4,
        uartTransmitBuffer,
        strlen((char *)uartTransmitBuffer)
    );
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

