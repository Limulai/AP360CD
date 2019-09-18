
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "api.h"
#include "FreeRTOS_CLI.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId FlashLedsTaskHandle;
osThreadId ButtonTaskHandle;
osThreadId TouchTaskHandle;
osThreadId EngineTaskHandle;
osThreadId CliHandle;
osThreadId AdcTaskHandle;
osMessageQId eventsQueueHALHandle;
osTimerId settingTimerHandle;
osTimerId fanTimerHandle;
osTimerId cleanAccTimerHandle;
osTimerId hvDelayTimerHandle;
osTimerId adjFanGradullyTimerHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// _deviceState deviceState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
void StartFlashLedsTask(void const * argument);
void StartButtonTask(void const * argument);
void StartTouchTask(void const * argument);
void StartEngineTask(void const * argument);
void StartCli(void const * argument);
void StartAdcTask(void const * argument);
void settingTimerCallback(void const * argument);
void fanTimerCallback(void const * argument);
void cleanAccTimerCallback(void const * argument);
void hvDelayTimerCallback(void const * argument);
void adjFanGradullyCallback(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void prvUARTCommandConsoleTask(void const * pvParameters );
extern void vRegisterCLICommands( void );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define CYCLE_CLEAN_ACC (1000*60)   //unit: ms  1000*60 for 1 miniuts
#define DURATION_CLEAN_ACC (14*24*60) //unit: min 14*24*60 for 14 days
/* USER CODE END 0 */

int fan1,fan2,fan3,fan4;//风扇每个等级标志
uint8_t fan1flag,fan2flag,fan3flag,fan4flag;//风扇按定时标志
uint8_t a = 0;//未修改时是start-touch函数中的局部变量
char slideflag,temp1=1;
/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  #ifdef ENABLE_IWDG
  MX_IWDG_Init();
  #endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
						//MX_IWDG_Init();
						/* USER CODE BEGIN 2 */
						// deviceState.deviceStatus = OFF;
						// deviceState.fanLever = LEVEL_LOW;
						// deviceState.blinkBits = 0x00;
						// deviceState.settingMode = FALSE;
						// deviceState.timer.timerValue = HOUR_2;
						// deviceState.timer.enable = FALSE;
						// deviceState.clean.time2clean = FALSE;
						// deviceState.gotFaultevent = FALSE;
						// deviceState.faultEnable = FALSE;
						// deviceState.hvFlag = FALSE;
  SetGlobalDeviceStatus(OFF);
	//上电默认不开风扇
  SetGlobalFanlevel(LEVEL_SILENT);
  SetGlobalBlinkBits(0x00);
  SetGlobalSettingMode(FALSE);
  SetGlobalTimerValue(HOUR_2);
  SetGlobalTimerEnable(FALSE);
  SetGlobalTime2Clean(FALSE);
  SetGlobalFaultEvent(FALSE);
  SetGlobalFaultEnable(FALSE);
  SetGlobalHvflag(FALSE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  uint8_t cleanAccTemp[2];
  GetCleanAccFromEEPROM(cleanAccTemp);
  SetGlobalCleanAcc(cleanAccTemp);


  StartSignal();
	HAL_ADC_Start_IT(&hadc1);
												/* fresh fault enable flag from eeprom */
	FreshenFaultEnableFlagFromEEPROM();

	vRegisterCLICommands();

  const  char str_test[40] = "kkk hellow world";


  HAL_UART_Transmit(&huart1, (uint8_t *) str_test, strlen( ( char * ) str_test ), strlen( ( char * ) str_test ) );
   HAL_UART_Transmit(&huart1, "hahha", 5, 5 ); 
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of settingTimer */
  osTimerDef(settingTimer, settingTimerCallback);
  settingTimerHandle = osTimerCreate(osTimer(settingTimer), osTimerOnce, NULL);

  /* definition and creation of fanTimer */
  osTimerDef(fanTimer, fanTimerCallback);
  fanTimerHandle = osTimerCreate(osTimer(fanTimer), osTimerOnce, NULL);

  /* definition and creation of cleanAccTimer */
  osTimerDef(cleanAccTimer, cleanAccTimerCallback);
  cleanAccTimerHandle = osTimerCreate(osTimer(cleanAccTimer), osTimerPeriodic, NULL);

  /* definition and creation of hvDelayTimer */
  osTimerDef(hvDelayTimer, hvDelayTimerCallback);
  hvDelayTimerHandle = osTimerCreate(osTimer(hvDelayTimer), osTimerOnce, NULL);

  /* definition and creation of adjFanGradullyTimer */
  osTimerDef(adjFanGradullyTimer, adjFanGradullyCallback);
  adjFanGradullyTimerHandle = osTimerCreate(osTimer(adjFanGradullyTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of FlashLedsTask */
  osThreadDef(FlashLedsTask, StartFlashLedsTask, osPriorityNormal, 0, 128);
  FlashLedsTaskHandle = osThreadCreate(osThread(FlashLedsTask), NULL);

  /* definition and creation of ButtonTask */
  osThreadDef(ButtonTask, StartButtonTask, osPriorityIdle, 0, 128);
  ButtonTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  /* definition and creation of TouchTask */
  osThreadDef(TouchTask, StartTouchTask, osPriorityIdle, 0, 128);
  TouchTaskHandle = osThreadCreate(osThread(TouchTask), NULL);

  /* definition and creation of EngineTask */
  osThreadDef(EngineTask, StartEngineTask, osPriorityIdle, 0, 128);
  EngineTaskHandle = osThreadCreate(osThread(EngineTask), NULL);

  /* definition and creation of Cli */
  osThreadDef(Cli, StartCli, osPriorityIdle, 0, 128);
  CliHandle = osThreadCreate(osThread(Cli), NULL);

  /* definition and creation of AdcTask */
  osThreadDef(AdcTask, StartAdcTask, osPriorityIdle, 0, 128);
  AdcTaskHandle = osThreadCreate(osThread(AdcTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	//vUARTCommandConsoleStart();
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of eventsQueueHAL */
/* what about the sizeof here??? cd native code */
  osMessageQDef(eventsQueueHAL, 16, uint8_t);
  eventsQueueHALHandle = osMessageCreate(osMessageQ(eventsQueueHAL), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  QueueHandle_t eventsQueue;
  eventsQueue = xQueueCreate(10, sizeof(event));
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analog WatchDog 1 
    */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 1241;
  AnalogWDGConfig.LowThreshold = 250;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 29;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 29;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 58;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  HAL_GPIO_WritePin(GPIOA, LED_POWER_RED_Pin|LED_CLEAN_Pin|LED_TIMER_Pin|LED_TIMER_PLUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_TURBO_Pin|LED_HIGH_Pin|LED_MID_Pin|LED_LOW_Pin 
                          |LED_POWER_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HV_ONOFF_GPIO_Port, HV_ONOFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : POWER_BUTTON_Pin */
  GPIO_InitStruct.Pin = POWER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_POWER_RED_Pin LED_POWER_GREEN_Pin LED_TIMER_Pin LED_TIMER_PLUS_Pin */
  GPIO_InitStruct.Pin = LED_POWER_RED_Pin|LED_CLEAN_Pin|LED_TIMER_Pin|LED_TIMER_PLUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_TURBO_Pin LED_HIGH_Pin LED_MID_Pin LED_LOW_Pin 
                           LED_CLEAN_Pin HV_ONOFF_Pin */
  GPIO_InitStruct.Pin = LED_TURBO_Pin|LED_HIGH_Pin|LED_MID_Pin|LED_LOW_Pin 
                          |LED_POWER_GREEN_Pin|HV_ONOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OD_TouchIC_Pin */
  GPIO_InitStruct.Pin = OD_TouchIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OD_TouchIC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartFlashLedsTask function */
void StartFlashLedsTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  //HAL_GPIO_TogglePin();
  /* Infinite loop */
  for(;;)
  {
    //ledsBlinky(deviceState.blinkBits);
    // 1, turn on leds
    if (GetGlobalBlinkBits() & led_clean_blink_bit) 
			cleanLedOn();
    if (GetGlobalBlinkBits() & led_power_green_blink_bit) 
			powerLedOn();
    if (GetGlobalBlinkBits() & led_timer_blink_bit) 
			timerLedOn();
    if (GetGlobalBlinkBits() & led_timer_plus_blink_bit) 
			plusTimerLedOn();
    if (GetGlobalBlinkBits() & led_low_blink_bit) 
			lowLedOn();
    if (GetGlobalBlinkBits() & led_mid_blink_bit) 
			midLedOn();
    if (GetGlobalBlinkBits() & led_high_blink_bit) 
			highLedOn();
    if (GetGlobalBlinkBits() & led_turbo_blink_bit) 
			turboLedOn();
  // 2, delay
    osDelay(500);
  // 3, turn off leds
    if (GetGlobalBlinkBits() & led_clean_blink_bit)
        cleanLedOff();
    if (GetGlobalBlinkBits() & led_power_green_blink_bit)
        powerLedOff();
    if (GetGlobalBlinkBits() & led_timer_blink_bit)
        timerLedOff();
    if (GetGlobalBlinkBits() & led_timer_plus_blink_bit)
        plusTimerLedOff();
    if (GetGlobalBlinkBits() & led_low_blink_bit)
        lowLedOff();
    if (GetGlobalBlinkBits() & led_mid_blink_bit)
        midLedOff();
    if (GetGlobalBlinkBits() & led_high_blink_bit)
        highLedOff();
    if (GetGlobalBlinkBits() & led_turbo_blink_bit)
        turboLedOff();
  // 4, delay
    osDelay(500);
		//osDelay(5000);
  }
  /* USER CODE END 5 */ 
}

/* StartButtonTask function */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonTask */
  /* Infinite loop */
  uint8_t a=0;
  event e;
  for(;;)
  {
    if (HAL_GPIO_ReadPin(POWER_BUTTON_GPIO_Port, POWER_BUTTON_Pin) == 0) {
      a++;
      if (a == 10) {
        e = BUTTON_POWER_SHORT;
        xQueueSend(eventsQueueHALHandle, &e, 10);
      }
      else if (a == 100) {
        e = BUTTON_POWER_LONG;
        xQueueSend(eventsQueueHALHandle, &e, 10);
      }
      else if (a == 200) {
        // get the turbo_mode flag
        ToggleTurboTestFlag();
        // toggle the flag
        HAL_NVIC_SystemReset();
        // then reset the mcu
      }
    }
    else {
      if ( a > 10 && a < 40) {
        // e = BUTTON_POWER_SHORT;
        // xQueueSend(eventsQueueHALHandle, &e, 10);
      }
      else if (a >= 40 && a < 80) {
        // if (GetGlobalFaultFromEEPROM() == TRUE) {
        //   SetFaultFlag2EEPROM(FALSE);
        //   FreshenFaultFlagFromEEPROM();
        // }
        // else {
        //   SetFaultFlag2EEPROM(TRUE);
        //   FreshenFaultFlagFromEEPROM();
        // }
        // e = EVENT_CLEANING_ACC;
        // deviceState.clean.time2clean = TRUE;
        // xQueueSend(eventsQueueHALHandle, &e, 10);
      }
      a = 0;
    }
    osDelay(50);
  }
  /* USER CODE END StartButtonTask */
}

/* StartTouchTask function */
void StartTouchTask(void const * argument)
{
  /* USER CODE BEGIN StartTouchTask */
  /* Infinite loop */
  //uint8_t a = 0;
  uint8_t key_value[2];
  touch_value ta = BUTTON_MAX;
  event e;
  osDelay(1000);
  for(;;)
  {
    //HAL_I2C_Master_Receive(&hi2c2, ADDR_TOUCH_READ, key_value, 2, 10); 
		  	 //HAL_I2C_Master_Receive(&hi2c1, ADDR_TOUCH_READ, key_value, 2, 10); 
    //fix_i2c_busy_bug();
		//长按事件power、clean


     if (HAL_GPIO_ReadPin(OD_TouchIC_GPIO_Port, OD_TouchIC_Pin) == 0) {
      //HAL_I2C_
      //HAL_GPIO_TogglePin(LED_CLEAN_GPIO_Port, LED_CLEAN_Pin);
      a++;
      //
      if(a == 1) {
        // record the key value
        HAL_I2C_Master_Receive(&hi2c2, ADDR_TOUCH_READ, key_value, 2, 10);
      }
      else if (a == 40) {
        // long press
        ta = InterpretTouchKeyValue(key_value);
        switch (ta) {
          case BUTTON_CLEAN : 
            //HAL_GPIO_TogglePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin);
            e = BUTTON_CLEAN_LONG;
            xQueueSend(eventsQueueHALHandle, &e, 10);
          break;			
					case BUTTON_POWER:
          //HAL_GPIO_TogglePin(LED_POWER_GREEN_GPIO_Port, LED_POWER_GREEN_Pin);
            e = BUTTON_POWER_LONG;
            xQueueSend(eventsQueueHALHandle, &e, 10);
            break;						
          default:
          break;
       }
      }
    }
		
	//短按事件
	//电源、清洁	、fan1、fan2、fan3、fan4、auto、
    else {
      if (a > 0 && a < 40) {
        // short press  
        ta = InterpretTouchKeyValue(key_value);

				switch (ta) {
            
  					case BUTTON_CLEAN : 
						
            e = BUTTON_CLEAN_SHORT;
            xQueueSend(eventsQueueHALHandle, &e, 10);
            break;
					
					  case BUTTON_ONE:
							
               e = EVENT_FAN_1;
						   fan1flag=a*35;
               xQueueSend(eventsQueueHALHandle, &e, 10);
							
						break;
						case BUTTON_TWO:
							
            e = EVENT_FAN_2;
						fan2flag=a*30;
            xQueueSend(eventsQueueHALHandle, &e, 10);
							
						break;
					 
						case BUTTON_THREE:
							
            e = EVENT_FAN_3;
            fan3flag=a*20;
            xQueueSend(eventsQueueHALHandle, &e, 10);
							
						break;
							 
      			case BUTTON_FOUR:   
            fan4flag=a;				
            e = EVENT_FAN_4;
            xQueueSend(eventsQueueHALHandle, &e, 10);
            break;
								
						case BUTTON_AUTO:
						SetGlobalTime2Clean(TRUE);
						e = EVENT_CLEANING_ACC;//测试清洁功能
         xQueueSend(eventsQueueHALHandle, &e, 10);
//            e = EVENT_AUTO;;
//            xQueueSend(eventsQueueHALHandle, &e, 10);
            break;
								
						case BUTTON_POWER:
            e = BUTTON_POWER_SHORT;;
            xQueueSend(eventsQueueHALHandle, &e, 10);
            break;															
          default:
          break;
        }
				
				
				
				//        switch (ta) {
//          case BUTTON_CLEAN : 
//            //HAL_GPIO_TogglePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin);
//            e = BUTTON_CLEAN_SHORT;
//            xQueueSend(eventsQueueHALHandle, &e, 10);
//          break;
//          case BUTTON_FAN:
//          //HAL_GPIO_TogglePin(LED_MID_GPIO_Port, LED_MID_Pin);
//            e = BUTTON_FAN_SHORT;
//            xQueueSend(eventsQueueHALHandle, &e, 10);
//          break;
//          case BUTTON_TIMER :
//          //HAL_GPIO_TogglePin(LED_LOW_GPIO_Port, LED_LOW_Pin);
//            e = BUTTON_TIMER_SHORT;
//            xQueueSend(eventsQueueHALHandle, &e, 10);
//          break;
//          default:
//          break;
//        }
       
      }
      a = 0;
    }
    osDelay(50);
  }
  /* USER CODE END StartTouchTask */
}

/* StartEngineTask function */
void StartEngineTask(void const * argument)
{
  /* USER CODE BEGIN StartEngineTask */
  /* Infinite loop */
	char temp;
  event new;
  event e;   
	if (GetTurboTestFlag()) {
    e = EVENT_TURBO_TEST;
    xQueueSend(eventsQueueHALHandle, &e, 10);
  }
  
                //浜у搧锟?????????6鍒嗛挓锟?????????娆＄疮锟?????????
  osTimerStart(cleanAccTimerHandle, CYCLE_CLEAN_ACC);
  //osTimerStart(cleanAccTimerHandle, 1000*60*6);
  for(;;)
  {

		
	if(fan1flag!=0)
	fan1flag--;
	if(fan2flag!=0)
	fan2flag--;
	if(fan3flag!=0)
	fan3flag--;
	if(fan4flag!=0)
	fan4flag--;
	
	//if(((fan1flag>10)&&(fan2flag>10))||((fan3flag>10)&&(fan2flag>10)))
		
	if((fan1flag>10)&&(fan2flag>10)&&(fan3flag>10))
		
	{
		if(temp1==1)
			{
					e = EVENT_FAN_4;
					slideflag=1;
					xQueueSend(eventsQueueHALHandle, &e, 10);
			}
	}

	
                  // feed i watch dog
    #ifdef ENABLE_IWDG
    MX_IWDG_Init();
    #endif
		
		            // fault detect form HAL_ADC_LevelOutOfWindowCallback  
		if (GetGlobalFaultEvent())
			{
					// clear the fault event flag
			SetGlobalFaultEvent(FALSE);
			if (GetGlobalFaultEnableFromEEPROM()){
						// this flag should be enable after high volt enable action
				if (GetGlobalFaultEnable()){			
						// send fault event
					e = EVENT_FAULT;
					xQueueSend(eventsQueueHALHandle, &e, 10);
				}
			}
		}

                  // Engine code
    if (xQueueReceive(eventsQueueHALHandle, &new, 10)) 
			{
      switch (GetGlobalDeviceStatus()) {
        case FAULT:
					   if (new == BUTTON_POWER_SHORT) {
            OutOfFaultStatus();
            // deviceState.deviceStatus = OFF;
            SetGlobalDeviceStatus(OFF);
            ChangeStatus2Off();
          }
						 
        break;
        case OFF:
          if (new == BUTTON_POWER_SHORT) 
					{
            // deviceState.deviceStatus = NORMAL;
           SetGlobalDeviceStatus(NORMAL);
            ChangeStatus2Normal();
          }
          else if (new == BUTTON_POWER_LONG){
            //HAL_GPIO_TogglePin(LED_POWER_RED_GPIO_Port, LED_POWER_RED_Pin);
          }
          else if (new == EVENT_TURBO_TEST) {
            SetGlobalDeviceStatus(TURBO_TEST);
            ChangeStatus2TurboTest();
          }
        break;
        case TURBO_TEST:
          if (new == EVENT_FAULT) {
            // deviceState.deviceStatus = FAULT;
            SetGlobalDeviceStatus(FAULT);
            ChangeStatus2Fault();
          }
        break;
        case NORMAL:
          if (new == BUTTON_POWER_SHORT) {
            // deviceState.deviceStatus = OFF;
            SetGlobalDeviceStatus(OFF);
            ChangeStatus2Off();
          }
					
					  else if (new == EVENT_FAN_1) {
										if(fan1%2==0)
											{
									SetGlobalFanlevel(1);
									fanLedSet(GetGlobalFanlevel());
									ResumeFan();
											fan2=0;
											fan3=0;
											fan4=0;
									    temp1=1;
										 }
										else
										{
											SetGlobalFanlevel(LEVEL_SILENT);
										ChangeStatus2Normal();
										}
                     fan1++;										
                }
						 
					else if (new == EVENT_FAN_2) {
									if(fan2%2==0)
										{
								SetGlobalFanlevel(2);
								fanLedSet(GetGlobalFanlevel());
								ResumeFan();
											fan1=0;
											fan3=0;
											fan4=0;
											temp1=1;
									}
							  else
								{
									SetGlobalFanlevel(LEVEL_SILENT);
								ChangeStatus2Normal();
								}
                fan2++;		
          }
							  
					else if (new == EVENT_FAN_3) {
									 if(fan3%2==0){
									SetGlobalFanlevel(3);
									fanLedSet(GetGlobalFanlevel());
									ResumeFan();
										 fan2=0;
											fan1=0;
											fan4=0;
										  temp1=1;
									 }
										else
										{
											SetGlobalFanlevel(LEVEL_SILENT);
											ChangeStatus2Normal();
										}
											fan3++;	
						 
          }
							 
					else if (new == EVENT_FAN_4)
						{
										 if(fan4%2==0)
										 {
										SetGlobalFanlevel(4);
										fanLedSet(GetGlobalFanlevel());
										ResumeFan();
															fan2=0;
															fan3=0;
															fan1=0;
											        temp1=0;
										 }
									
										 else
										 {
											 SetGlobalFanlevel(LEVEL_SILENT);
											 ChangeStatus2Normal();
											 temp1=1;
											 slideflag=0;
										 }
							      fan4++;	
          }
							 
					else if (new == EVENT_AUTO) {
								 if(temp%2==0)
	           		plusTimerLedOn();//对应auto
		             else
		           plusTimerLedOff();//
								 temp++;
								   
          }
//          else if (new == BUTTON_FAN_SHORT) {
//						SetGlobalFanlevel(GetGlobalFanlevel() + 1);
//            if ( GetGlobalFanlevel() >=5) {
//              // deviceState.fanLever = 1;
//              SetGlobalFanlevel(1);
//            }
//            fanLedSet(GetGlobalFanlevel());
//            ResumeFan();
//          }
          else if (new == BUTTON_TIMER_SHORT) {
            // deviceState.deviceStatus = TIMER_SETTING;
            SetGlobalDeviceStatus(TIMER_SETTING);
            ChangeStatus2TimerSetting();
          }
          else if (new == BUTTON_TIMER_LONG) {
            if (GetGlobalTimerEnable() == TRUE) {
              // deviceState.timer.enable = FALSE;
              SetGlobalTimerEnable(FALSE);
              plusTimerLedOff();
            }
          }
          else if (new == EVENT_CLEANING_ACC) {
            cleanStartBlink();
          }
          else if (new == BUTTON_CLEAN_LONG) {
            if (GetGlobalTime2Clean() == TRUE) {
              // 1, clear duration of clean
              ClearCleanAcc();
              // deviceState.clean.cleanAcc[0] = 0;
							// deviceState.clean.cleanAcc[1] = 0;
              uint8_t a[2] = {0, 0};
              SetGlobalCleanAcc(a);
              // then
              // deviceState.clean.time2clean = FALSE;
              SetGlobalTime2Clean(FALSE);
              cleanStopBlink();
            }				 
          }
					 else if (new == BUTTON_CLEAN_SHORT) {
             
								   if(temp%2==0)//此为调试
	           		plusTimerLedOn();//
		             else
		           plusTimerLedOff();//
								 temp++;
            }
          else if (new == EVENT_FAULT) {
            // deviceState.deviceStatus = FAULT;
            SetGlobalDeviceStatus(FAULT);
            ChangeStatus2Fault();
          }
          else if (new == EVENT_TIMER_ACC) {
            if (GetGlobalTimerEnable() == TRUE) {
              // deviceState.timer.enable = FALSE;
              SetGlobalTimerEnable(FALSE);
              if (GetGlobalDeviceStatus() != OFF) {
                e = BUTTON_POWER_SHORT;
                xQueueSend(eventsQueueHALHandle, &e, 10);
              }
            }
          }
        break;
        case TIMER_SETTING:
          if (new == BUTTON_POWER_SHORT) {
            // deviceState.deviceStatus = OFF;
            SetGlobalDeviceStatus(OFF);
            ChangeStatus2Off();
          }
          else if (new == BUTTON_TIMER_SHORT) {
						SetGlobalTimerValue(GetGlobalTimerValue() + 1);
            if ( GetGlobalTimerValue() >=5) {
              // deviceState.timer.timerValue = 0;
              SetGlobalTimerValue(0);
            } 
                    // stop blink for flash fan leds state
            timerSettingStopBlink();
            timerSettingStartBlink(GetGlobalTimerValue());
            osTimerStart (settingTimerHandle, 5000);

          }
          else if (new == BUTTON_TIMER_LONG) {
            // deviceState.timer.enable = FALSE;
            SetGlobalTimerEnable(FALSE);
            OutOfTimerSettingStatus();
            // deviceState.deviceStatus = NORMAL;
            SetGlobalDeviceStatus(NORMAL);
            ChangeStatus2Normal();
            
          }
          else if (new == EVENT_TIMER_SETTING_ACC 
                  || new == BUTTON_FAN_SHORT) {
            OutOfTimerSettingStatus();
            // deviceState.deviceStatus = NORMAL;
            SetGlobalDeviceStatus(NORMAL);
            ChangeStatus2Normal();
          }
          else if (new == EVENT_FAULT) {
            // devifceState.deviceStatus = FAULT;
            SetGlobalDeviceStatus(FAULT);
            ChangeStatus2Fault();
          }
        break;
        
        default:
        break;
      }     
    }
    osDelay(10);
  }
  /* USER CODE END StartEngineTask */
}

/* StartCli function */
void StartCli(void const * argument)
{
	
	
  /* USER CODE BEGIN StartCli */
  /* Infinite loop */
  prvUARTCommandConsoleTask(NULL);

	
  /* USER CODE END StartCli */
}

/* StartAdcTask function */
void StartAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAdcTask */
  char str[40];
  uint32_t value_adc_32;
  float value_adc_f;

  /* Infinite loop */
  for(;;)
  {
    // HAL_ADC_PollForConversion(&hadc1, 1000);
    // value_adc_32 = (float)HAL_ADC_GetValue(&hadc1);
    // if (GetGlobalFaultEnable()){
    //   if (value_adc_32 >= 1241 || value_adc_32 <= 250) {
    //     sprintf(str, "adc_32 value is: %u\r\n", value_adc_32);
    //     HAL_UART_Transmit(&huart1, (uint8_t *) str, strlen( ( char * ) str ), strlen( ( char * ) str ) );
    //     value_adc_f = (float)value_adc_32 / 4096 * 3.3;
    //     sprintf(str, "adc_f value is: %f\r\n\r\n", value_adc_f);
    //     //vOutputString(str);
    //     HAL_UART_Transmit(&huart1, (uint8_t *) str, strlen( ( char * ) str ), strlen( ( char * ) str ) );
    //   }
    // }
    
    
    osDelay(100);
    // next, print the value
  }
  /* USER CODE END StartAdcTask */
}

/* settingTimerCallback function */
void settingTimerCallback(void const * argument)
{
  /* USER CODE BEGIN settingTimerCallback */
  event e = EVENT_TIMER_SETTING_ACC;
  xQueueSend(eventsQueueHALHandle, &e, 10);
  /* USER CODE END settingTimerCallback */
}

/* fanTimerCallback function */
void fanTimerCallback(void const * argument)
{
  /* USER CODE BEGIN fanTimerCallback */
  event e = EVENT_TIMER_ACC;
  xQueueSend(eventsQueueHALHandle, &e, 10);
  /* USER CODE END fanTimerCallback */
}

/* cleanAccTimerCallback function */
void cleanAccTimerCallback(void const * argument)
{
  /* USER CODE BEGIN cleanAccTimerCallback */
  unsigned char x[2] = {0, 0};
  x[0]++;
  event e;
  if  (GetGlobalDeviceStatus() == NORMAL 
    || GetGlobalDeviceStatus() == TIMER_SETTING) {
      GetCleanAccFromEEPROM(x);
      if (x[0] == 255) {
        if (x[1] == 255) {
          // do nothing
        }
        else {
          x[0]++;
          x[1]++;
        }
      }
      else {
        x[0]++;
      }  
			
  
      SetCleanAcc2EEPROM(x);
      SetGlobalCleanAcc(x);
              // x shoul equal 10 * 24 * 14
      if (((uint16_t)x[1]*256 + x[0]) >= (uint16_t)DURATION_CLEAN_ACC && (GetGlobalTime2Clean() == FALSE)) {
      //if (x > 10 * 24 * 14)
        // deviceState.clean.time2clean = TRUE;
        SetGlobalTime2Clean(TRUE);
        e = EVENT_CLEANING_ACC;
         xQueueSend(eventsQueueHALHandle, &e, 10);
      }
    }
  
  // while(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(ADDR_EEPROM_WRITE), 3, 100 ) != HAL_OK);
  // HAL_I2C_Mem_Read(&hi2c1, (unsigned short)ADDR_EEPROM_READ, 
  //                   (unsigned short)WORD_ADDR_CLEAN_ACC_HIGH8BIT, 2,
  //                   x, 2, 10);
  // x[0]++;
  // while(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(ADDR_EEPROM_WRITE), 3, 100 ) != HAL_OK);
  // HAL_I2C_Mem_Write(&hi2c1, (uint16_t)ADDR_EEPROM_WRITE, 
  //                   (uint16_t)WORD_ADDR_CLEAN_ACC_HIGH8BIT, 2, 
  //                   x, 2 ,10);
  /* USER CODE END cleanAccTimerCallback */
}

/* hvDelayTimerCallback function */
void hvDelayTimerCallback(void const * argument)
{
  /* USER CODE BEGIN hvDelayTimerCallback */
  if (GetGlobalHvflag() == TRUE) {
    // deviceState.faultEnable = TRUE;
    SetGlobalFaultEnable(TRUE);
  }
  /* USER CODE END hvDelayTimerCallback */
}

/* adjFanGradullyCallback function */
void adjFanGradullyCallback(void const * argument)
{
  /* USER CODE BEGIN adjFanGradullyCallback */
  unsigned char fan_pwm_last;
  unsigned char fan_pwm_current;
  fan_pwm_last = GetFanPercentLast();
  fan_pwm_current = GetFanPercentCurrent();
  if (fan_pwm_last < fan_pwm_current) 
		{
    if (fan_pwm_current - fan_pwm_last > 10)//如果当前比旧的大10以上，就在旧的基础上加10
			{
      fan_pwm_last += 10;
    }
    else //如果当前比旧的不大于10，就取当前作旧的
			{
      fan_pwm_last = fan_pwm_current;
    }
    SetFanPWM(fan_pwm_last);
    SetFanPercentLast(fan_pwm_last);
  }
  else if (fan_pwm_last > fan_pwm_current)//如果旧的比当前的大
		{
    if (fan_pwm_last - fan_pwm_current > 10){
      fan_pwm_last -= 10;
    }
    else {
      fan_pwm_last = fan_pwm_current;
    }
    SetFanPWM(fan_pwm_last);
    SetFanPercentLast(fan_pwm_last);
  }
  else {
    // stop this timer
    osTimerStop(adjFanGradullyTimerHandle);
  }
  
  /* USER CODE END adjFanGradullyCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
