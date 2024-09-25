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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart4_tx;

/* USER CODE BEGIN PV */
uint32_t timeSinceLastClock = 0;
uint64_t receivedSequence = 0;
uint8_t receivedCounter = 0;
uint32_t channelState = 0;
uint8_t uartsinglemessage[71], uartbuffer[2000], uartTransmitBuffer[2000];

GPIO_TypeDef* GPIOsequence[] = {CH1_GPIO_Port, CH2_GPIO_Port, CH3_GPIO_Port, CH4_GPIO_Port, CH5_GPIO_Port, CH6_GPIO_Port, CH7_GPIO_Port, CH8_GPIO_Port, CH9_GPIO_Port, CH10_GPIO_Port, CH11_GPIO_Port, CH12_GPIO_Port, CH13_GPIO_Port, CH14_GPIO_Port, CH15_GPIO_Port, CH16_GPIO_Port, CH17_GPIO_Port, CH18_GPIO_Port, CH19_GPIO_Port, CH20_GPIO_Port};
uint32_t PinSequence[] = {CH1_Pin, CH2_Pin, CH3_Pin, CH4_Pin, CH5_Pin, CH6_Pin, CH7_Pin, CH8_Pin, CH9_Pin, CH10_Pin, CH11_Pin, CH12_Pin, CH13_Pin, CH14_Pin, CH15_Pin, CH16_Pin, CH17_Pin, CH18_Pin, CH19_Pin, CH20_Pin};
uint8_t scan2000_20ChannelSequence[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 1, 2, 3, 4, 5, 6, 7 , 8, 9, 10, 21, 22};    // CH11..CH20, CH1..CH10, Bank 2 to OUT, Bank 2 to 4W
uint8_t scan2000ChannelOffSequence[] = {17, 19, 21, 23, 8, 14, 0, 2, 4, 5, 12};      // CH1..CH10, 4W
uint8_t scan2000ChannelOnSequence[] = {16, 18, 20, 22, 9, 13, 15, 1, 3, 6, 11};      // CH1..CH10, 4W
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // turn on the LED, the logic is inverted
  printf("\n===============\nBooting\n");
  for (uint8_t i=0; i < 6; i++) {
    HAL_Delay(100); // sleep for 100 ms
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    if (now - timeSinceLastClock > 1) {
        receivedCounter = 0;
        receivedSequence = 0;
        timeSinceLastClock = now;
    }
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
int decode_10channels(uint32_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister) {
    // Check always high bits
    if ((command & SCAN_2000_ALWAYS_HIGH_BITS) != SCAN_2000_ALWAYS_HIGH_BITS) {
        return 1;
    }
    // Remove the bits, that are always high
    command &= ~SCAN_2000_ALWAYS_HIGH_BITS;
    *relaySetRegister = 0x0000;
    *relayUnsetRegister = 0x0000;

    // There is no need to run the loop, if there is nothing to do. Every second command
    // only contains the bits, that are always high. We can ignore those commands.
    if (command != 0x00000000) {
        // 10 channels + 1 4W relay
        for (uint8_t i = 0; i < 11; i++) {
            // Compile a list of channels, that are to be turned on
            // The scanner card supports up to 20 channels on two separate buses (CH1-CH10 and CH11-CH20),
            // but the DMM might only support 10 channels.
            // We therefore use channels CH1-CH5 and CH11-CH15 on the scanner card and skip CH6-CH10,
            // because CH1-CH5 and CH6-CH10 are connected to the same bus. This is why we use the
            // i + (i/5) * 5 term to skip CH6-CH10.
            // The MSB is the 4W relay, the LSB is CH1

            *relaySetRegister |= !!(command & (1 << scan2000ChannelOnSequence[i])) << (i + (i / 5) * 5);
            // The list of channels, that are to tbe turned off
            *relayUnsetRegister |= !!(command & (1 << scan2000ChannelOffSequence[i])) << (i + (i / 5) * 5);
        }
    }
    // Always unset CH6-CH10 and CH-16-CH20
    // There is no need to not set the relaySetRegister, because unsetting a relay takes precidence.
    *relayUnsetRegister |= 0b11111000001111100000;

    return 0;
}

int decode_20channels(uint64_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister) {
    *relaySetRegister = 0x0000;
    *relayUnsetRegister = 0x0000;

    // There is no need to run the loop, if there is nothing to do.
    if (command != 0x00000000) {
        // Process the channels (incl. CH21, 4W mode)
        for (uint8_t i = 0; i < 22; i++) {
            *relayUnsetRegister |= command & (1 << (2 * (scan2000_20ChannelSequence[i] - 1)));    // Even clock pulses -> turn relays off
            *relaySetRegister |= command & (1 << (2 * (scan2000_20ChannelSequence[i] - 1) + 1));  // Odd clock pulses -> turn relays on
        }
    }
    return 0;
}

bool validateRelayState(uint32_t channelState) {
    // A valid state is the following:
    // - If the the two relay banks are connected, only one relay may be opened
    // - If the banks are disconnected (4W mode), one relay in each bank may be connected
    int countBank1 = __builtin_popcountl(channelState & 0x003FF);
    int countBank2 = __builtin_popcountl(channelState & 0xFFC00);
    bool ch21Enabled = (0x100000 & channelState);
    return
        (!ch21Enabled && (countBank1 + countBank2 <= 1))   // If "CH21" (4W Relay) is disabled
        || (ch21Enabled && (countBank1 <= 1 && countBank2 <= 1)); // If "CH21" (4W Relay) is enabled
}

void setRelays(uint32_t newChannelState) {
    // If we have a new state, update the relays
    if (newChannelState != channelState) {
        channelState = newChannelState;

        // First disconnect all channels, that need to be disconnected
        for (uint8_t i=0; i<20; i++) {
            if (!(channelState & (1 << i))) {
                HAL_GPIO_WritePin(GPIOsequence[i], PinSequence[i], GPIO_PIN_RESET);
            }
        }

        // If CH11-CH20 are turned off, disconect them from the all buses to reduce the isolation capacitance
        // else connect it either to the 4W output or the sense output
        if (!(channelState & 0xFFC00)) {
            HAL_GPIO_WritePin(Bus_Sense_GPIO_Port, Bus_Sense_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Bus_In_GPIO_Port, Bus_In_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(Bus_Sense_GPIO_Port, Bus_Sense_Pin, !(channelState & (1 << 20)));       // TODO: Check if that channel setting is sent
            HAL_GPIO_WritePin(Bus_In_GPIO_Port, Bus_In_Pin, !!(channelState & (1 << 20)));
        }

        // Finally connect all channels, that need to be connected
        for (uint8_t i=0; i<20; i++) {
            if (channelState & (1 << i)) {
                HAL_GPIO_WritePin(GPIOsequence[i], PinSequence[i], GPIO_PIN_SET);
            }
        }
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Clock_Pin) {
        // Clock in a new bit
        receivedSequence = receivedSequence << 1;
        receivedSequence |= HAL_GPIO_ReadPin(Data_GPIO_Port, Data_Pin);
        receivedCounter++;
        timeSinceLastClock = HAL_GetTick();
    } else if (GPIO_Pin == Strobe_Pin) {
        // The sequence is over, decode it now
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        uartbuffer[0] = '\0';
        uint32_t newChannelState = channelState;

        if (receivedCounter == 24) {
            // We have a command for a 10 channel SCAN2000/SCAN2001 card
            uint32_t relaySetRegister = 0x00,  relayUnsetRegister = 0x00;
            int result = decode_10channels((uint32_t)receivedSequence, &relaySetRegister, &relayUnsetRegister);
            if (result) {
                // Terminate here and print an error. Do note, we only received 24 bit (6 byte).
                // newlib-nano does not support printing 64-bit numbers (%llx), so we drop the leading zeros
                // convert to unsigned long.
                sprintf((char *)uartsinglemessage, "Error. Invalid command received: 0x%06lx\nDropping command.\n", (uint32_t)(receivedSequence));
                strcat((char *)uartbuffer, (char *)uartsinglemessage);
            } else {
                // Now apply the updates
                newChannelState |= relaySetRegister;    // closed channels
                newChannelState &= ~relayUnsetRegister; // opened channels
            }
        } else if (receivedCounter == 48) {
            uint32_t relaySetRegister = 0x00,  relayUnsetRegister = 0x00;
            decode_20channels((uint32_t)receivedSequence, &relaySetRegister, &relayUnsetRegister);
            // Now apply the updates
            newChannelState |= relaySetRegister;    // closed channels
            newChannelState &= ~relayUnsetRegister; // opened channels
        } else {
            // Do not process the command, if it is of unknown size
            sprintf((char *)uartsinglemessage, "Error. Invalid command length: %u\nDropping command.\n", receivedCounter);
            strcat((char *)uartbuffer, (char *)uartsinglemessage);
        }

        // Test if the new state is valid and if so, apply it
        bool validState = validateRelayState(newChannelState);

        if (validState) {
            setRelays(newChannelState);
            /*sprintf((char *)uartsinglemessage, "Relay state: 0x%lx\n", channelState);
            strcat((char *)uartbuffer, (char *)uartsinglemessage);*/
        } else {
            sprintf((char *)uartsinglemessage, "Error. Invalid relay state: 0x%lx\nDropping command.\n", newChannelState);
            strcat((char *)uartbuffer, (char *)uartsinglemessage);
        }

        receivedSequence = 0x00;
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

