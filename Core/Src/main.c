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
  double MV341_I_mA;
  double MV205_1_I_mA;
  double MV205_2_I_mA;
  double U_MAIN;
  double MV341_Temp;
  double MV205_1_Temp;
  double MV205_2_Temp;
  double Vref;
}MeasurementsTypeDef;

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
  uint8_t ExtRefOscEnabled;
  MeasurementsTypeDef Meas;

  struct _status
  {
    uint32_t AdcUpdatedCnt;
    uint32_t MainCycleTime;
    uint32_t UartTaskCnt;
    uint32_t SuccessParsedCmdCnt;
    uint32_t MV341_WarmUpMs;
    uint32_t MV205_1_WarmUpMs;
    uint32_t MV205_2_WarmUpMs;
    uint8_t Lock_1;
    uint8_t Lock_2;
    uint8_t MV205_1_En;
    uint8_t MV205_2_En;
    uint32_t UpTimeSec;
    uint32_t QueryCnt;
  }Status;

  struct
  {
    CtrlStatesTypeDef Next;
    CtrlStatesTypeDef Curr;
    CtrlStatesTypeDef Pre;
  }State;



}DeviceTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_BUFFER_SIZE        40
#define CMDLINE_UNKNOWN_PARAMETER_ERROR    "!UNKNOWN PARAMETER ERROR '%s'"
#define INTER_STATE_DEALY_MS    5000
#define MV341_I_LIMIT_MA        300
#define MV205_1_I_LIMIT_MA      200
#define MV205_2_I_LIMIT_MA      200
#define ADC_UPDATE_MS           500


#define DAS_DI_LOCK1      (uint8_t) 1<<0
#define DAS_DI_LOCK2      (uint8_t) 1<<1
#define DAS_DI_MV205_1    (uint8_t) 1<<2
#define DAS_DI_MV205_2    (uint8_t) 1<<3

#define DAS_DO_MV205_1_EN (uint8_t) 1<<0
#define DAS_DO_MV205_2_EN (uint8_t) 1<<1

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
char UartRxBuffer[RS485_BUFFER_SIZE];
char UartTxBuffer[RS485_BUFFER_SIZE];
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
void ControlTask(void);
void UartTask(void);
uint8_t DoesExtRefOscEnable(void);
void ControlBothMv205Enable(void);
void SetMV205_1(FunctionalState enable);
void SetMV205_2(FunctionalState enable);
void SetLock2Led(FunctionalState enable);
void SetLock1Led(FunctionalState enable);
void SetMV205EnLed(FunctionalState enable);
FunctionalState GetLock2();
FunctionalState GetLock1();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Measurements --------------------------------------------------------------*/
void AcdTask(void)
{
  static uint32_t timestamp=0;
  if(HAL_GetTick() - timestamp > ADC_UPDATE_MS){
    timestamp = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AdcChannelsResult, ADC_CH_COUNT);
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  Device.Status.AdcUpdatedCnt++;

  double lsb = 3.3/4096; //0.000805

  //Ha a táp 10V, akkor R229 és R228 között 1.31V-mérhető... Uin=9.55V és 9.21V-ot jelez
  Device.Meas.U_MAIN = AdcChannelsResult.U_MAIN * lsb* 1 / (2.4/(15 + 2.4));

  //pl:  0.050A * 0.1R * x20 = 0.1V
  //Imért = (ADC * LSB) / 20 / 0.1R * 1000mA
  Device.Meas.MV341_I_mA = (AdcChannelsResult.MV341_I * lsb) / 20 / 0.1 * 1000;
  Device.Meas.MV205_1_I_mA = (AdcChannelsResult.MV205_1_I * lsb) / 20 / 0.1 * 1000;
  Device.Meas.MV205_2_I_mA = (AdcChannelsResult.MV205_2_I * lsb) / 20 / 0.1 * 1000;

  //0C = 0.5
  //10fok változás = 0.1V változás
  Device.Meas.MV341_Temp = ((AdcChannelsResult.MV341_Temp * lsb) - 0.5) * 100;
  Device.Meas.MV205_1_Temp = ((AdcChannelsResult.MV205_1_Temp * lsb) - 0.5) * 100;
  Device.Meas.MV205_2_Temp = ((AdcChannelsResult.MV205_2_Temp * lsb) - 0.5) * 100;

  //Device.Meas.Vref = AdcChannelsResult.Vref * lsb;

}

/* UART ----------------------------------------------------------------------*/
/*Todo: Ida DMA megaszakitása kellene, ez itt rossz*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  DeviceErrLog("Megtelt");
}

void UartTask(void)
{

  if(strlen(UartRxBuffer)==0)
    return;

  uint8_t terminated = 0;
  for(uint8_t i=0; i<strlen(UartRxBuffer);i++)
    if(UartRxBuffer[i]=='\r' || UartRxBuffer[i]=='\n')
    {
      UartRxBuffer[i]=0;
      terminated = 1;
    }

  if(!terminated)
     return;
  else
    HAL_UART_DMAStop(&huart1);

  if(!strcmp(UartRxBuffer , "*OPC?"))
  {
    Device.Status.SuccessParsedCmdCnt++;
    strcpy(UartTxBuffer, "*OPC");
  }
  else if(!strcmp(UartRxBuffer , "MV341_I_mA?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV341_I_mA);
  }
  else if(!strcmp(UartRxBuffer , "MV205_1_I_mA?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV205_1_I_mA);
  }
  else if(!strcmp(UartRxBuffer , "MV205_2_I_mA?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV205_2_I_mA);
  }
  else if(!strcmp(UartRxBuffer , "U_MAIN?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.U_MAIN);
  }
  else if(!strcmp(UartRxBuffer , "MV341_Temp?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV341_Temp);
  }
  else if(!strcmp(UartRxBuffer , "MV205_1_Temp?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV205_1_Temp);
  }
  else if(!strcmp(UartRxBuffer , "MV205_2_Temp?"))
  {
    sprintf(UartTxBuffer, "%0.3f", Device.Meas.MV205_2_Temp);
  }
  else if(!strcmp(UartRxBuffer , "MV341_WarmUpMs?"))
  {
    sprintf(UartTxBuffer, "%ld", Device.Status.MV341_WarmUpMs);
  }
  else if(!strcmp(UartRxBuffer , "MV205_1_WarmUpMs?"))
  {
    sprintf(UartTxBuffer, "%ld", Device.Status.MV205_1_WarmUpMs);
  }
  else if(!strcmp(UartRxBuffer , "MV205_2_WarmUpMs?"))
  {
    sprintf(UartTxBuffer, "%ld", Device.Status.MV205_2_WarmUpMs);
  }
  else if(!strcmp(UartRxBuffer , "Lock_1?"))
  {
    sprintf(UartTxBuffer, "%d", Device.Status.Lock_1);
  }
  else if(!strcmp(UartRxBuffer , "Lock_2?"))
  {
    sprintf(UartTxBuffer, "%d", Device.Status.Lock_2);
  }
  else if(!strcmp(UartRxBuffer , "MV205_1_En?"))
  {
    sprintf(UartTxBuffer, "%d", Device.Status.MV205_1_En);
  }
  else if(!strcmp(UartRxBuffer , "MV205_2_En?"))
  {
    sprintf(UartTxBuffer, "%d", Device.Status.MV205_2_En);
  }
  else if(!strcmp(UartRxBuffer , "State?"))
  {
    sprintf(UartTxBuffer, "%d", Device.State.Curr);
  }
  else if(!strcmp(UartRxBuffer , "Seconds?"))
  {
    sprintf(UartTxBuffer, "%ld", Device.Status.UpTimeSec);
  }
  else if(!strcmp(UartRxBuffer , "QueryCnt?"))
  {
    sprintf(UartTxBuffer, "%ld", Device.Status.QueryCnt);
    Device.Status.QueryCnt++;
  }
  else
  {
    sprintf(UartTxBuffer, CMDLINE_UNKNOWN_PARAMETER_ERROR, UartRxBuffer);
  }

  uint8_t resp_len =strlen(UartTxBuffer);
  UartTxBuffer[resp_len]= '\r'; /*Todo:Ezt javisd ki \n-re 2022.02.03*/
  UartTxBuffer[++resp_len]= 0;

  if(strlen(UartTxBuffer)!=0)
  {
    //!!!DeviceDbgLog("Rx:%s, Tx:%s",UartRxBuffer,UartTxBuffer);
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, sizeof(UartTxBuffer), 100);
    memset(UartTxBuffer,0x00,RS485_BUFFER_SIZE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t*) UartRxBuffer, RS485_BUFFER_SIZE);
  }
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
         DeviceDbgLog("ControlTask: SDEV_START -> SDEV_WAIT");
       break;
     }

     case SDEV_WAIT:
     {
       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
       {
         Device.State.Next = SDEV_IDLE;
         DeviceDbgLog("ControlTask: SDEV_WAIT -> SDEV_IDLE");
       }
       break;
     }

     case SDEV_IDLE:
     {
       if(DoesExtRefOscEnable())
       {
         DeviceDbgLog("ControlTask.Mode: External ref osc...");
         Device.State.Next = SDEV_MV205_1_WARMING;
         DeviceDbgLog("ControlTask: SDEV_IDLE -> SDEV_MV205_1_WARMING");
       }
       else
       {
         DeviceDbgLog("ControlTask.Mode: Internal ref osc...");
         Device.State.Next = SDEV_MV341_WARMING;
         DeviceDbgLog("ControlTask: SDEV_IDLE -> SDEV_MV341_WARMING");
       }
       break;
     }
     case SDEV_MV341_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Status.MV341_WarmUpMs = 0;
       }

       if(Device.Meas.MV341_I_mA < MV341_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV341_WARM_CPLT;
         Device.Status.MV341_WarmUpMs = HAL_GetTick() -  timestamp;
         DeviceDbgLog("ControlTask: SDEV_MV341_WARMING -> SDEV_MV341_WARM_CPLT");
       }
       else
       {
         if(DoesExtRefOscEnable())
         {
           DeviceDbgLog("ControlTask.Mode: External ref osc...");
           Device.Status.MV341_WarmUpMs = 0;
           Device.State.Next = SDEV_MV205_1_WARMING;
           DeviceDbgLog("ControlTask: SDEV_MV341_WARMING -> SDEV_MV205_1_WARMING")
         }
       }
       break;
     }
     case SDEV_MV341_WARM_CPLT:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         SetMV205_1(ENABLE);
       }

       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS)
       {

         Device.State.Next = SDEV_MV205_1_WARMING;
         DeviceDbgLog("ControlTask: SDEV_MV341_WARM_CPLT -> SDEV_MV205_1_WARMING");
       }
       break;
     }
     case SDEV_MV205_1_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Status.MV205_1_WarmUpMs = 0;
       }

       if(Device.Meas.MV205_1_I_mA < MV205_1_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV205_1_WARM_CPLT;
         Device.Status.MV205_1_WarmUpMs = HAL_GetTick() -  timestamp;
         DeviceDbgLog("ControlTask: SDEV_MV341_WARMING -> SDEV_MV205_1_WARM_CPLT");
       }
       break;
     }
     case SDEV_MV205_1_WARM_CPLT:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         SetMV205_2(ENABLE);
         timestamp = HAL_GetTick();
         DeviceDbgLog("SDEV_MV205_1_WARM_CPLT.Tick:%ld", HAL_GetTick());
       }

       if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS)
       {
         Device.State.Next = SDEV_MV205_2_WARMING;
         DeviceDbgLog("ControlTask: SDEV_MV205_1_WARM_CPLT -> SDEV_MV205_2_WARMING");
       }
       break;
     }
     case SDEV_MV205_2_WARMING:
     {
       if(Device.State.Pre != Device.State.Curr)
       {
         timestamp = HAL_GetTick();
         Device.Status.MV205_2_WarmUpMs = 0;
       }

       if(Device.Meas.MV205_2_I_mA < MV205_2_I_LIMIT_MA)
       {
         Device.State.Next = SDEV_MV205_2_WARM_CPLT;
         Device.Status.MV205_2_WarmUpMs = HAL_GetTick() -  timestamp;
         DeviceDbgLog("ControlTask: SDEV_MV205_2_WARMING -> SDEV_MV205_2_WARM_CPLT");
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
         DeviceDbgLog("ControlTask: SDEV_MV205_2_WARM_CPLT -> SDEV_WARMING_SEQ_CPLT");
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

  DelayMs(500);

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

#ifdef DEBUG
  printf(VT100_ATTR_RED);
    DeviceUsrLog("This is a DEBUG version.");
  printf(VT100_ATTR_RESET);
#endif

  DeviceUsrLog("Manufacturer:%s, Name:%s, Version:%04X",DEVICE_MNF, DEVICE_NAME, DEVICE_FW);
  HAL_UART_Receive_DMA (&huart1, (uint8_t*)UartRxBuffer, RS485_BUFFER_SIZE);

  SetMV205_1(DISABLE);
  SetMV205_2(DISABLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t timestamp=0;
    static uint32_t secTimestamp=0;

    Device.Status.MainCycleTime = timestamp;

    LiveLedTask(&hLiveLed);
    AcdTask();
    ControlTask();
    UartTask();

    if(HAL_GetTick() - secTimestamp >= 1000){
      secTimestamp = HAL_GetTick();
      Device.Status.UpTimeSec++;
    }

    if(Device.Status.MV205_1_En && Device.Status.MV205_2_En)
      SetMV205EnLed(ENABLE);
    else
      SetMV205EnLed(DISABLE);

    if(GetLock1()==ENABLE)
    {
      Device.Status.Lock_1 = 1;
      SetLock1Led(ENABLE);
    }
    else
    {
      Device.Status.Lock_1 = 0;
      SetLock1Led(DISABLE);
    }

    if(GetLock2()==ENABLE)
    {
      Device.Status.Lock_2 = 1;
      SetLock2Led(ENABLE);
    }
    else
    {
      Device.Status.Lock_2 = 0;
      SetLock2Led(DISABLE);
    }

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

/* printf -------------------------------------------------------------------*/

//UART
/*int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}*/

//SWO

int _write(int file, char *ptr, int len)
{
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
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

void SetMV205_1(FunctionalState enable)
{
  if(enable == ENABLE)
  {
    Device.Status.MV205_1_En = 1;
    HAL_GPIO_WritePin(MV205_1_EN_GPIO_Port, MV205_1_EN_Pin, GPIO_PIN_RESET);
  }
  else
  {
    Device.Status.MV205_1_En = 0;
    HAL_GPIO_WritePin(MV205_1_EN_GPIO_Port, MV205_1_EN_Pin, GPIO_PIN_SET);
  }
}

void SetMV205_2(FunctionalState enable)
{
  if(enable == ENABLE)
  {
    Device.Status.MV205_2_En = 1;
    HAL_GPIO_WritePin(MV205_2_EN_GPIO_Port, MV205_2_EN_Pin, GPIO_PIN_RESET);

  }
  else
  {
    Device.Status.MV205_2_En = 0;
    HAL_GPIO_WritePin(MV205_2_EN_GPIO_Port, MV205_2_EN_Pin, GPIO_PIN_SET);
  }
}


FunctionalState GetLock1()
{

  if( HAL_GPIO_ReadPin(LOCK1_GPIO_Port,LOCK1_Pin) == GPIO_PIN_SET)
    return ENABLE;
  else
    return DISABLE;
}

FunctionalState GetLock2()
{

  if( HAL_GPIO_ReadPin(LOCK2_GPIO_Port,LOCK2_Pin) == GPIO_PIN_SET)
    return ENABLE;
  else
    return DISABLE;
}



void SetMV205EnLed(FunctionalState enable)
{
  if(enable == ENABLE)
    HAL_GPIO_WritePin(MV205EN_LED_GPIO_Port, MV205EN_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(MV205EN_LED_GPIO_Port, MV205EN_LED_Pin, GPIO_PIN_RESET);
}

void SetLock1Led(FunctionalState enable)
{
  if(enable == ENABLE)
    HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LOCK1_LED_GPIO_Port, LOCK1_LED_Pin, GPIO_PIN_RESET);
}

void SetLock2Led(FunctionalState enable)
{
  if(enable == ENABLE)
    HAL_GPIO_WritePin(LOCK2_LED_GPIO_Port, LOCK2_LED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LOCK2_LED_GPIO_Port, LOCK2_LED_Pin, GPIO_PIN_RESET);
}

uint8_t DoesExtRefOscEnable(){
  uint8_t temp;
  temp = HAL_GPIO_ReadPin(LOCK2_GPIO_Port, LOCK2_Pin ) == GPIO_PIN_SET;
  return temp;
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

