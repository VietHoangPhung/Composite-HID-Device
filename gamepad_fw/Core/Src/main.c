/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/** mode **/
typedef enum {
    MODE_GAMEPAD,
    MODE_MOUSE,
    MODE_COUNT
} Mode;

typedef struct
__attribute__((__packed__))
{
  uint8_t reportId;                        // Report ID = 0x02
  uint8_t leftClick:1;
  uint8_t rightClick:1;
  uint8_t midClick:1;
  uint8_t pad:5;
  int8_t pointerX;
  int8_t pointerY;
  int8_t wheel;
} mouseReport_t;


typedef struct
__attribute__((__packed__))
{
	uint8_t reportId; 	                 	//Report ID 0x01
	uint16_t buttons;
	int8_t leftX;
	int8_t leftY;
	int8_t rightX;
	int8_t rightY;
}gamepadReport_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BTN_NUM 			16
#define ADC_CHANNELS		4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/** Buttons with corresponding pins and port **/
GPIO_TypeDef* port[16] = {GPIOA, 	  GPIOA,	  GPIOB, 	  GPIOB, 	  GPIOB, 	   GPIOB, 	    GPIOB, 	     GPIOA, 	  GPIOB, 	  GPIOB, 	   GPIOA, 	   GPIOB, 	    GPIOB, 	 	GPIOB, 	 	GPIOB, 	 	GPIOB};
uint16_t pin[16] = 		 {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_12, GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_9, GPIO_PIN_15, GPIO_PIN_4, GPIO_PIN_14, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_8};

/* USER CODE BEGIN PV */
uint16_t adcValues[ADC_CHANNELS];
Mode modeState = MODE_GAMEPAD;

mouseReport_t mouseReport = {0x02, 0, 0, 0, 0, 0, 0};
gamepadReport_t gamepadReport = {0x01, 0, 0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

uint8_t switchMode(void);
//int getValue(int channel);
void configureADCChannel(uint32_t channel);
void getAllADCValue(void);
void gamepadProcess(void);
void mouseProcess(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern void USBD_CUSTOM_HID_SendReport();

//void USBD_CUSTOM_HID_SendReport();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void (*modeHandlers[])(void) = {gamepadProcess, mouseProcess};

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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  mouseReport.reportId = 2;
  gamepadReport.reportId = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getAllADCValue();
	  if(switchMode())
	  {
		  for(int i = 0; i < 5; i++)
		  {
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			  HAL_Delay(1000);
		  }
		  modeState++;
		  if(modeState >= MODE_COUNT)
			  modeState = MODE_GAMEPAD;
	  }
	  modeHandlers[modeState]();
  }

  /* USER CODE END 3 */
}

/** my functions declaration **/

uint8_t switchMode(void)
{
	return (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)));
}

void configureADCChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;  // Rank is irrelevant in single-channel mode
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void getAllADCValue(void)
{
    for (int i = 0; i < ADC_CHANNELS; i++)
    {
        // Configure each channel
        configureADCChannel(ADC_CHANNEL_0 + i);

        // Start ADC conversion
        HAL_ADC_Start(&hadc1);

        // Poll for conversion and store value
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            adcValues[i] = HAL_ADC_GetValue(&hadc1);
        }
    }

    // Stop ADC once after reading all channels
    HAL_ADC_Stop(&hadc1);
}

//int getValue(int channel)
//{
//	int value = 0;
//	ADC_ChannelConfTypeDef sConfig = {0};
//	sConfig.Channel = channel;
//	sConfig.Rank = 1;
//	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	HAL_ADC_Start(&hadc1);
//	HAL_Delay(1);
//	HAL_ADC_PollForConversion(&hadc1, 10);
//	value = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);
//	return value;
//}

//void getAllADCValue(void)
//{
//	  adcValues[0] = getValue(ADC_CHANNEL_0);
//	  adcValues[1] = getValue(ADC_CHANNEL_1);
//	  adcValues[2] = getValue(ADC_CHANNEL_2);
//	  adcValues[3] = getValue(ADC_CHANNEL_3);
//}

//void getAllADCValue(void)
//{
//    if (HAL_ADC_Start(&hadc1) != HAL_OK)
//    {
//        Error_Handler();
//        adcValues[0] = adcValues[1] = adcValues[2] = adcValues[3] = 2047;
//    }
//
//    for (int i = 0; i < 4; i++)
//    {
//        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
//        {
//            adcValues[i] = HAL_ADC_GetValue(&hadc1);
//        }
//        else
//        {
//            adcValues[i] = 2047;
//        }
//    }
//
//    // Stop the ADC
//    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
//    {
//        Error_Handler();
//    }
//}

void gamepadProcess(void)
{
	// Get axes values
//    gamepadReport.leftX = (2047 - adcValues[0]) * 255 / 4095;		// -127 - 127
//    gamepadReport.leftY = (2047 - adcValues[1]) * 255 / 4095;
//    gamepadReport.rightX = (2047 - adcValues[2]) * 255 / 4095;
//    gamepadReport.rightY = (2047 - adcValues[3]) * 255 / 4095;

	    gamepadReport.leftX = (127 - adcValues[0]);		// -127 - 127
	    gamepadReport.leftY = (127 - adcValues[1]);
	    gamepadReport.rightX = (127 - adcValues[2]);
	    gamepadReport.rightY = (127 - adcValues[3]);

    // Reset buttons
    gamepadReport.buttons = 0;

    // Get buttons state
    for (int i = 0; i < 16; i++)
    {
        if (HAL_GPIO_ReadPin(port[i], pin[i]) == GPIO_PIN_RESET)
        {
            gamepadReport.buttons |= (1 << i);
        }
    }

    // Send report to the host
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &gamepadReport, sizeof(gamepadReport));
}

void mouseProcess(void)
{
	// Get axes values
//    mouseReport.pointerX = (2047 - adcValues[0]) * 150 / 4095;		// -127 - 127
//    mouseReport.pointerY = (2047 - adcValues[1]) * 150 / 4095;
	mouseReport.pointerX = (127 - adcValues[0]) * 32 / 255;	// -127 - 127
	mouseReport.pointerY = (127 - adcValues[1]) * 32 / 255;

    // Get buttons state
    mouseReport.leftClick = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
    mouseReport.rightClick = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET);
    mouseReport.midClick = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);

    // Get wheel values
    //mouseReport.wheel = __HAL_TIM_GET_COUNTER(&htim2) - 127;

    // Send report to the host
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &mouseReport, sizeof(mouseReport));
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  //hadc1.Init.EOCSelection = EOC_SEQ_CONV;
  //hadc1.Init.EOCSelection = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA9 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB12
                           PB13 PB14 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
