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
#include <stdio.h>
#include <stdlib.h>

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
}AdcChannelsTypeDef;
#define ADC_CH_COUNT sizeof(AdcChannelsTypeDef)/sizeof(uint16_t)

typedef enum _CtrlStatesTypeDef
{
  SDEV_START,                   //0
  SDEV_WAIT,                    //1
  SDEV_IDLE,                    //2
  SDEV_MV341_WARMING,           //3
  SDEV_MV341_WARM_CPLT,         //4
  SDEV_MV205_1_WARMING,         //5
  SDEV_MV205_1_WARM_CPLT,       //6
  SDEV_MV205_2_WARMING,         //7
  SDEV_MV205_2_WARM_CPLT,       //8
  SDEV_WARMING_SEQ_CPLT,        //9
}CtrlStatesTypeDef;

typedef struct _DeviceTypeDef
{
  struct
  {
    CtrlStatesTypeDef Next;
    CtrlStatesTypeDef Curr;
    CtrlStatesTypeDef Pre;
  }State;

  struct _Meas
  {
    double MV341_I_mA;
    double MV205_1_I_mA;
    double MV205_2_I_mA;
    double U_MAIN;
    double MV341_Temp;
    double MV205_1_Temp;
    double MV205_2_Temp;
  }Meas;

  struct _DasClock
  {
    uint8_t DI;
    uint8_t DO;
    double  AI[ADC_CH_COUNT];
  }DasClock;

  struct _Diag
  {
    uint32_t AdcUpdatedCnt;
    uint32_t MV341_WarmUpMs;
    uint32_t MV205_1_WarmUpMs;
    uint32_t MV205_2_WarmUpMs;

    uint32_t RS485ResponseCnt;
    uint32_t RS485RequestCnt;
    uint32_t RS485UnknwonCnt;
    uint32_t RS485NotMyCmdCnt;

    uint32_t UartTaskCnt;
    uint32_t UART_Receive_IT_ErrorCounter;
    uint32_t UartErrorCounter;
    uint32_t UpTimeSec;
  }Diag;
}DeviceTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** RS485 ***/
#define RS485_BUFFER_SIZE   40
#define RS485_TX_HOLD_MS     1
#define RS485_CMD_LENGTH    10
#define RS485_ARG1_LENGTH   10
#define RS485_ARG2_LENGTH   10
#define CLIENT_TX_ADDR      0x20
#define CLIENT_RX_ADDR      0x02

/*** DasClock ***/
#define DAS_DI_LOCK1        (uint8_t) 1<<0
#define DAS_DI_LOCK2        (uint8_t) 1<<1
#define DAS_DI_EXT_EN       (uint8_t) 1<<2
#define DAS_DI_MV1_EN       (uint8_t) 1<<3
#define DAS_DI_MV2_EN       (uint8_t) 1<<4

#define DAS_DO_MV1_EN       (uint8_t) 1<<0
#define DAS_DO_MV2_EN       (uint8_t) 1<<1

#define DAS_AI_MV341_I_MA     0
#define DAS_AI_MV205_1_I_MA   1
#define DAS_AI_MV205_2_I_MA   2
#define DAS_AI_U_MAIN         3
#define DAS_AI_MV341_TEMP     4
#define DAS_AI_MV205_1_TEMP   5
#define DAS_AI_MV205_2_TEMP   6

/*** Control ***/
#define INTER_STATE_DEALY_MS    5000
#define MV341_I_LIMIT_MA        300
#define MV205_1_I_LIMIT_MA      200
#define MV205_2_I_LIMIT_MA      200
#define ADC_UPDATE_MS           500


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
DeviceTypeDef     Device;
AdcChannelsTypeDef   AdcChannelsResult;

/*** RS485 ***/
char UartRxBuffer[RS485_BUFFER_SIZE];
char UartTxBuffer[RS485_BUFFER_SIZE];
char        UartCharacter;
uint8_t     UartRxBufferPtr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/*** LiveLed ***/
void LiveLedOff(void);
void LiveLedOn(void);
void SetLock2Led(FunctionalState enable);
void SetLock1Led(FunctionalState enable);
void SetMV205EnLed(FunctionalState enable);

/*** DASClock ***/
void AcdTask(void);
void ControlTask(void);
uint8_t ReadDI(void);
void WriteDO(uint8_t state);

/*** UART/RS485 ***/
char* RS485Parser(char *line);
void UartTask(void);
void RS485DirRx(void);
void RS485DirTx(void);
void RS485TxTask(void);

/*** Tools ***/
void UpTimeTask(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Measurements --------------------------------------------------------------*/
void AcdTask(void)
{
  static uint32_t timestamp=0;
  if(HAL_GetTick() - timestamp > ADC_UPDATE_MS)
  {
    timestamp = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AdcChannelsResult, ADC_CH_COUNT);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  Device.Diag.AdcUpdatedCnt++;
  double lsb = 3.3/4096; //0.000805

  //pl:  0.050A * 0.1R * x20 = 0.1V
  //Imért = (ADC * LSB) / 20 / 0.1R * 1000mA
  Device.Meas.MV341_I_mA = (AdcChannelsResult.MV341_I * lsb) / 20 / 0.1 * 1000;
  Device.DasClock.AI[DAS_AI_MV341_I_MA] = Device.Meas.MV341_I_mA;

  Device.Meas.MV205_1_I_mA = (AdcChannelsResult.MV205_1_I * lsb) / 20 / 0.1 * 1000;
  Device.DasClock.AI[DAS_AI_MV205_1_I_MA] = Device.Meas.MV205_1_I_mA;

  Device.Meas.MV205_2_I_mA = (AdcChannelsResult.MV205_2_I * lsb) / 20 / 0.1 * 1000;
  Device.DasClock.AI[DAS_AI_MV205_2_I_MA] = Device.Meas.MV205_2_I_mA;

  //Ha a táp 10V, akkor R229 és R228 között 1.31V-mérhető... Uin=9.55V és 9.21V-ot jelez
  Device.Meas.U_MAIN = AdcChannelsResult.U_MAIN * lsb* 1 / (2.4/(15 + 2.4));
  Device.DasClock.AI[DAS_AI_U_MAIN] = Device.Meas.U_MAIN;

  //0C = 0.5
  //10fok változás = 0.1V változás
  Device.Meas.MV341_Temp = ((AdcChannelsResult.MV341_Temp * lsb) - 0.5) * 100;
  Device.DasClock.AI[DAS_AI_MV341_TEMP] = Device.Meas.MV341_Temp;

  Device.Meas.MV205_1_Temp = ((AdcChannelsResult.MV205_1_Temp * lsb) - 0.5) * 100;
  Device.DasClock.AI[DAS_AI_MV205_1_TEMP] = Device.Meas.MV205_1_Temp;

  Device.Meas.MV205_2_Temp = ((AdcChannelsResult.MV205_2_Temp * lsb) - 0.5) * 100;
  Device.DasClock.AI[DAS_AI_MV205_2_TEMP] = Device.Meas.MV205_2_Temp;
}

/* UART-RS485-----------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *context)
{
  if(HAL_UART_Receive_IT(context, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UART_Receive_IT_ErrorCounter++;
  else
  {
    if(UartCharacter == '\n')
    {
      UartRxBuffer[UartRxBufferPtr] = '\0';
      strcpy(UartTxBuffer, RS485Parser(UartRxBuffer));
      UartRxBufferPtr = 0;
    }
    else
      UartRxBuffer[UartRxBufferPtr++] = UartCharacter;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Device.Diag.UartErrorCounter++;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

  if(HAL_UART_Receive_IT(huart, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UART_Receive_IT_ErrorCounter++;
}

char* RS485Parser(char *line)
{
  unsigned int addr = 0;
  char buffer[RS485_BUFFER_SIZE];
  char cmd[RS485_CMD_LENGTH];
  char arg1[RS485_ARG1_LENGTH];
  char arg2[RS485_ARG2_LENGTH];

  memset(buffer, 0x00, RS485_BUFFER_SIZE);
  memset(cmd,0x00, RS485_CMD_LENGTH);
  memset(arg1,0x00, RS485_ARG1_LENGTH);
  memset(arg2,0x00, RS485_ARG2_LENGTH);

  uint8_t params = sscanf(line, "#%x %s %s %s",&addr, cmd, arg1, arg2);
  if(addr != CLIENT_RX_ADDR)
  {
    Device.Diag.RS485NotMyCmdCnt++;
    return NULL;
  }
  Device.Diag.RS485RequestCnt++;
  if(params == 2)
  {
    if(!strcmp(cmd, "*OPC?"))
    {
      strcpy(buffer, "OK");
    }
    else if(!strcmp(cmd, "*RDY?"))
    {
      strcpy(buffer, "OK");
    }
    else if(!strcmp(cmd, "*WHOIS?"))
    {
      sprintf(buffer, "*WHOIS %s", DEVICE_NAME);
    }
    else if(!strcmp(cmd, "*VER?"))
    {
      sprintf(buffer, "*VER %s", DEVICE_FW);
    }
    else if(!strcmp(cmd, "*UID?"))
    {
      sprintf(buffer, "*UID %4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
    }
    else if(!strcmp(cmd,"UPTIME?"))
    {
       sprintf(buffer, "UPTIME %08lX", Device.Diag.UpTimeSec);
    }
    else if(!strcmp(cmd,"DI?"))
    {
       sprintf(buffer, "DI %02X", Device.DasClock.DI);
    }
    else if(!strcmp(cmd,"DO?"))
    {
       sprintf(buffer, "DO %02X", Device.DasClock.DO);
    }
    else
    {
      Device.Diag.RS485UnknwonCnt++;
    }
  }
  else if(params == 3)
  {
    if(!strcmp(cmd,"DO"))
    {
      Device.DasClock.DO = strtol(arg1, NULL, 16);
      strcpy(buffer, "OK");
    }
    else if(!strcmp(cmd,"AI?"))
    {
       uint8_t ch = strtol(arg1, NULL, 10);
       if(ch < ADC_CH_COUNT)
         sprintf(buffer, "AI %d %0.3f", ch, Device.DasClock.AI[ch] );
    }
    else
    {
      Device.Diag.RS485UnknwonCnt++;
    }
  }
  else
  {
    Device.Diag.RS485UnknwonCnt++;
  }
  static char resp[2 * RS485_BUFFER_SIZE];
  memset(resp, 0x00, RS485_BUFFER_SIZE);

  sprintf(resp, "#%02X %s", CLIENT_TX_ADDR, buffer);

  uint8_t length = strlen(resp);
  resp[length] = '\n';
  resp[length + 1] = '\0';
  return resp;
}

void RS485TxTask(void)
{
  uint8_t txn=strlen(UartTxBuffer);
  if( txn != 0)
  {
    Device.Diag.RS485ResponseCnt++;
    RS485DirTx();
    DelayMs(RS485_TX_HOLD_MS);
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, txn, 100);
    UartTxBuffer[0] = 0;
    RS485DirRx();
  }
}

void RS485DirTx(void)
{
  HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
}

void RS485DirRx(void)
{
  HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}


/* StateMachineTask -----------------------------------------------------------*/
void ControlTask(void)
{
  static uint32_t timestamp = 0;

  switch (Device.State.Curr)
  {
     case SDEV_START:
     {
         Device.State.Next = SDEV_WAIT;
         timestamp = HAL_GetTick();
       break;
     }

     case SDEV_WAIT:
     {
       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
       {
         Device.State.Next = SDEV_IDLE;
       }
       break;
     }

     case SDEV_IDLE:
     {
       if(Device.DasClock.DI & DAS_DI_EXT_EN)
       {
         Device.State.Next = SDEV_MV205_1_WARMING;
       }
       else
       {
         Device.State.Next = SDEV_MV341_WARMING;
       }
       break;
     }
     case SDEV_MV341_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Diag.MV341_WarmUpMs = 0;
       }

       if(Device.Meas.MV341_I_mA < MV341_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV341_WARM_CPLT;
         Device.Diag.MV341_WarmUpMs = HAL_GetTick() -  timestamp;
       }
       else
       {
         if(Device.DasClock.DI & DAS_DI_EXT_EN)
         {
           Device.Diag.MV341_WarmUpMs = 0;
           Device.State.Next = SDEV_MV205_1_WARMING;
         }
       }
       break;
     }
     case SDEV_MV341_WARM_CPLT:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.DasClock.DO |= DAS_DO_MV1_EN;
       }

       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS)
       {
         Device.State.Next = SDEV_MV205_1_WARMING;
       }
       break;
     }
     case SDEV_MV205_1_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Diag.MV205_1_WarmUpMs = 0;
       }

       if(Device.Meas.MV205_1_I_mA < MV205_1_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV205_1_WARM_CPLT;
         Device.Diag.MV205_1_WarmUpMs = HAL_GetTick() -  timestamp;
       }
       break;
     }
     case SDEV_MV205_1_WARM_CPLT:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         Device.DasClock.DO |= DAS_DO_MV2_EN;
         timestamp = HAL_GetTick();
       }
       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS)
       {
         Device.State.Next = SDEV_MV205_2_WARMING;
       }
       break;
     }
     case SDEV_MV205_2_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Diag.MV205_2_WarmUpMs = 0;
       }

       if(Device.Meas.MV205_2_I_mA < MV205_2_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV205_2_WARM_CPLT;
         Device.Diag.MV205_2_WarmUpMs = HAL_GetTick() -  timestamp;
       }
       break;
     }
     case SDEV_MV205_2_WARM_CPLT:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
       }
       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS)
       {
         Device.State.Next = SDEV_WARMING_SEQ_CPLT;
       }
       break;
     }
     case SDEV_WARMING_SEQ_CPLT:
     {
       break;
     }
  }

  Device.State.Pre = Device.State.Curr;
  Device.State.Curr = Device.State.Next;
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

  DelayMs(250);

  /*** Clocks ***/
  Device.DasClock.DO &= ~(DAS_DO_MV1_EN);
  Device.DasClock.DO &= ~(DAS_DO_MV2_EN);

  /*** RS485 ***/
  RS485DirRx();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t timestamp;
  while (1)
  {
    if(HAL_GetTick() - timestamp > 100)
    {
      timestamp = HAL_GetTick();
      Device.DasClock.DI = ReadDI();
      WriteDO(Device.DasClock.DO);

      SetMV205EnLed((Device.DasClock.DO & DAS_DO_MV1_EN) && (Device.DasClock.DO & DAS_DO_MV1_EN));
      SetLock1Led(Device.DasClock.DI & DAS_DI_LOCK1);
      SetLock2Led(Device.DasClock.DI & DAS_DI_LOCK2);
      if(Device.DasClock.DO & DAS_DO_MV1_EN)
        Device.DasClock.DI = DAS_DI_MV1_EN;
      if(Device.DasClock.DO & DAS_DO_MV2_EN)
        Device.DasClock.DI = DAS_DI_MV2_EN;
    }

    LiveLedTask(&hLiveLed);
    AcdTask();
    UpTimeTask();
    ControlTask();
    RS485TxTask();

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
  huart1.Init.BaudRate = 230400;
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
  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)&UartCharacter, 1) != HAL_OK)
  {
    Device.Diag.UART_Receive_IT_ErrorCounter++;
  }
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
  HAL_GPIO_WritePin(GPIOB, MV205_1_EN_Pin|MV205_2_EN_Pin|MV205EN_LED_Pin|LOCK2_LED_Pin
                          |LOCK1_LED_Pin|LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MV205_1_EN_Pin MV205_2_EN_Pin MV205EN_LED_Pin LOCK2_LED_Pin
                           LOCK1_LED_Pin LIVE_LED_Pin */
  GPIO_InitStruct.Pin = MV205_1_EN_Pin|MV205_2_EN_Pin|MV205EN_LED_Pin|LOCK2_LED_Pin
                          |LOCK1_LED_Pin|LIVE_LED_Pin;
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

/* LiveLed -------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

/* DasClock-------------------------------------------------------------------*/
void SetMV205EnLed(FunctionalState enable)
{
  if(enable == ENABLE)
    HAL_GPIO_WritePin(MV205EN_LED_GPIO_Port, MV205EN_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(MV205EN_LED_GPIO_Port, MV205EN_LED_Pin, GPIO_PIN_RESET);
}

void SetLock1Led(FunctionalState enable)
{
  /*
  if(enable == ENABLE)
    HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_RESET);
   */
}

void SetLock2Led(FunctionalState enable)
{
  if(enable == ENABLE)
    HAL_GPIO_WritePin(LOCK2_LED_GPIO_Port, LOCK2_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LOCK2_LED_GPIO_Port, LOCK2_LED_Pin, GPIO_PIN_RESET);
}

uint8_t ReadDI(void)
{
  uint8_t state = 0;

  if( HAL_GPIO_ReadPin(LOCK1_GPIO_Port,LOCK1_Pin) == GPIO_PIN_SET)
    state |= DAS_DI_LOCK1;

  if( HAL_GPIO_ReadPin(LOCK2_GPIO_Port,LOCK2_Pin) == GPIO_PIN_SET)
    state |= DAS_DI_LOCK2;

  if(HAL_GPIO_ReadPin(INT_EXT_GPIO_Port, INT_EXT_Pin ) == GPIO_PIN_SET)
    state |= DAS_DI_EXT_EN;

  return state;
}

void WriteDO(uint8_t state)
{
  if(state & DAS_DO_MV1_EN)
    HAL_GPIO_WritePin(MV205_1_EN_GPIO_Port, MV205_1_EN_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(MV205_1_EN_GPIO_Port, MV205_1_EN_Pin, GPIO_PIN_SET);

  if(state & DAS_DO_MV2_EN)
    HAL_GPIO_WritePin(MV205_2_EN_GPIO_Port, MV205_2_EN_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(MV205_2_EN_GPIO_Port, MV205_2_EN_Pin, GPIO_PIN_SET);
};

/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;

  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.Diag.UpTimeSec++;
  }
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
