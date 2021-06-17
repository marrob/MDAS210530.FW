/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include "common.h"
#include "vt100.h"
#include "LiveLed.h"
#include "StringPlus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _AdcChannelsTypeDef{
  uint16_t MV341_I;
  uint16_t MV205_1_I;
  uint16_t MV205_2_I;
  uint16_t U_MAIN;
  uint16_t MV341_Temp;
  uint16_t MV205_1_Temp;
  uint16_t MV205_2_Temp;
  //uint16_t Vref;
}AdcChannelsTypeDef;
#define ADC_CH_COUNT sizeof(AdcChannelsTypeDef)/sizeof(uint16_t)

typedef struct _MeasurementsTypeDef{
  double MV341_I;
  double MV205_1_I;
  double MV205_2_I;
  double U_MAIN;
  double MV341_Temp;
  double MV205_1_Temp;
  double MV205_2_Temp;
  double Vref;
}MeasurementsTypeDef;



typedef struct _DeviceTypeDef
{
  MeasurementsTypeDef Meas;
  struct _status
  {
    uint32_t MainCycleTime;
  }Status;


  uint32_t AdcUpdated;
}DeviceTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef hLiveLed;
uint32_t UartRxTimestamp;
DeviceTypeDef     Device;
AdcChannelsTypeDef   AdcChannelsResult;
char UartRxBuffer[UART_BUFFER_SIZE];
char UartTxBuffer[UART_BUFFER_SIZE];
char Receive[UART_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void MV205Enable(void);
void AcdTask(void);
void StateMachineTask(void);
void UartTask(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Measurements --------------------------------------------------------------*/
void AcdTask(void)
{
  static uint32_t timestamp=0;
  if(HAL_GetTick() - timestamp > 1000){
    timestamp = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AdcChannelsResult, ADC_CH_COUNT);
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  DeviceDbgLog("HAL_ADC_ConvCpltCallback %lu", Device.AdcUpdated++);

  double lsb = 3.3/4096; //0.000805

  //Ha a táp 10V, akkor R229 és R228 között 1.31V-mérhető... Uin=9.55V és 9.21V-ot jelez
  Device.Meas.U_MAIN = AdcChannelsResult.U_MAIN * lsb* 1 / (2.4/(15 + 2.4));

  //pl:  0.050A * 0.1R * x20 = 0.1V
  //Imért = (ADC * LSB) / 20 / 0.1R
  Device.Meas.MV341_I = (AdcChannelsResult.MV341_I * lsb) / 20 / 0.1;
  Device.Meas.MV205_1_I = (AdcChannelsResult.MV205_1_I * lsb) / 20 / 0.1;
  Device.Meas.MV205_2_I = (AdcChannelsResult.MV205_2_I * lsb) / 20 / 0.1;

  //0C = 0.5
  //10fok változás = 0.1V változás
  Device.Meas.MV341_Temp = ((AdcChannelsResult.MV341_Temp * lsb) - 0.5) * 100;
  Device.Meas.MV205_1_Temp = ((AdcChannelsResult.MV205_1_Temp * lsb) - 0.5) * 100;
  Device.Meas.MV205_2_Temp = ((AdcChannelsResult.MV205_2_Temp * lsb) - 0.5) * 100;

  //Device.Meas.Vref = AdcChannelsResult.Vref * lsb;

  //DeviceDbgLog("U_MAIN:%02fV",  Device.Meas.U_MAIN);
}

/* UART ----------------------------------------------------------------------*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
    HAL_UART_Receive_DMA(&huart1, (uint8_t*) UartRxBuffer, UART_BUFFER_SIZE);
}

void UartTask(void)
{
  static uint32_t timestamp=0;

  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();


    if(strlen(UartRxBuffer)!=0)
    {

      for(uint8_t i=0; i<strlen(UartRxBuffer);i++)
        if(UartRxBuffer[i]=='\r')
          UartRxBuffer[i]=0;

      for(uint8_t i=0; i<strlen(UartRxBuffer);i++)
        if(UartRxBuffer[i]=='\n')
          UartRxBuffer[i]=0;


      if(!strcmp(UartRxBuffer , "*OPC?"))
      {
        strcpy(UartTxBuffer, "*OPC");
      }
    }



    if(strlen(UartTxBuffer)!=0)
    {
      HAL_UART_DMAStop(&huart1);
      HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, sizeof(UartTxBuffer), 100);
      memset(UartTxBuffer,0x00,UART_BUFFER_SIZE);
      memset(UartRxBuffer,0x00,UART_BUFFER_SIZE);
      HAL_UART_Receive_DMA(&huart1, (uint8_t*) UartRxBuffer, UART_BUFFER_SIZE);
    }

  }
}

/* StateMachineTask -----------------------------------------------------------*/
void StateMachineTask(void)
{
  static uint32_t timestamp=0;

  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();

    if(HAL_GPIO_ReadPin(LOCK1_GPIO_Port,LOCK1_Pin ) == GPIO_PIN_SET)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);


    if(HAL_GPIO_ReadPin(LOCK2_GPIO_Port, LOCK2_Pin ) == GPIO_PIN_SET)
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      else
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
}


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
  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  DelayMs(500);
  MV205Enable();

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

#ifdef DEBUG
  printf(VT100_ATTR_RED);
    DeviceUsrLog("This is a DEBUG version.");
  printf(VT100_ATTR_RESET);
#endif

  DeviceUsrLog("Manufacturer:%s, Name:%s, Version:%04X",DEVICE_MNF, DEVICE_NAME, DEVICE_FW);
  UartRxTimestamp = HAL_GetTick();

  HAL_UART_Receive_DMA (&huart1, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t timestamp=0;

    LiveLedTask(&hLiveLed);
    AcdTask();
    StateMachineTask();
    UartTask();

    Device.Status.MainCycleTime = HAL_GetTick() - timestamp;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MV205_1_EN_Pin|MV205_2_EN_Pin|LED3_Pin|LED2_Pin
                          |LED1_Pin|LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MV205_1_EN_Pin MV205_2_EN_Pin LED3_Pin LED2_Pin
                           LED1_Pin LIVE_LED_Pin */
  GPIO_InitStruct.Pin = MV205_1_EN_Pin|MV205_2_EN_Pin|LED3_Pin|LED2_Pin
                          |LED1_Pin|LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_EXT_Pin LOCK2_Pin LOCK1_Pin */
  GPIO_InitStruct.Pin = INT_EXT_Pin|LOCK2_Pin|LOCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* printf -------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

void MV205Enable(void){

  HAL_GPIO_WritePin(MV205_1_EN_GPIO_Port, MV205_1_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MV205_2_EN_GPIO_Port, MV205_2_EN_Pin, GPIO_PIN_RESET);
 // HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
