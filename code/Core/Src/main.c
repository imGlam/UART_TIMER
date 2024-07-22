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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// second change
// change from browser
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DHT_PORT GPIOB
#define DHT_PIN GPIO_PIN_6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// reset
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t counter_from_x[5]; // 0 --> 10,000
uint8_t duty_cycle_num[20];
int counter = 0;
int choice = 0;
int prev_choice;
uint32_t counter_in_callback;
uint16_t waiting;
uint16_t previous_tim;
uint16_t current_tim;
uint8_t msg1[1];
uint8_t blinking_mode[20] = "\nBlinking Mode";
uint8_t modulation_mode[20] = "\nModulation Mode";
uint8_t stop_modulation_mode[42] = "\nStop Modulation Mode";
uint8_t stop_blinking_mode[42] = "\nStop Blinking Mode";
uint8_t done[40] = "\nDone! Enter 9 to stop";
uint8_t be_on_operation[100] = "\nSelected Mode Is On Operation";
uint8_t led[5] ={LED1_Pin,LED2_Pin,LED3_Pin,LED4_Pin,LED5_Pin};
bool on_led = false,
	 led_reverse_flag = false;
uint16_t capture = 0;
uint32_t counter_increment;
uint8_t logg[100] ="\n7.Blinking Mode\n8. Modulation Mode";

// mode flag
bool modulation_flag;
bool blinking_flag;
bool choice_flag = true;
bool stop_flag;

//Initialize DHT11
float Temperature=0, Humidity=0;
uint16_t SUM, RH, TEMP;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2,
        Presence =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < time);
}

// sensor

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void DHT_Start (void)
{
	Set_Pin_Output (DHT_PORT, DHT_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);   // pull the pin low
	Delay (18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
    Delay (20);   // wait for 30us
	Set_Pin_Input(DHT_PORT, DHT_PIN);    // set as input
}

uint8_t DHT_Check_Response (void)
{
	uint8_t Response = 0;
	Delay (40);
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		Delay (80);
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go high
		Delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));  // wait for the pin to go low
	}
	return i;
}

void Display_Temp(void)
{
	uint8_t temp[20];
	sprintf(temp,"Temp: %2.f C\n\r",Temperature);
	HAL_UART_Transmit(&huart1, (uint8_t*)temp, (uint8_t)sizeof(temp), 100);
}

void Display_Humi(void)
{
	char humi[30];
	sprintf(humi,"Rh: %2.f ",Humidity);
	HAL_UART_Transmit(&huart1, (uint8_t*)humi, (uint8_t)sizeof(humi), 100);
	uint8_t pc[]="%\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)pc, (uint8_t)sizeof(pc), 100);
}


void On_Off(void)
{
	if (on_led) {
		for (int i = 0; i < 5; i++) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
		}
		uint8_t on[] = "LEDs TURNED ON\n";
		HAL_UART_Transmit(&huart1, on, sizeof(on), 100);
	} else {
		for (int i = 0; i < 5; i++) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 0);
			uint8_t off[] = "LEDs TURNED OFF\n";
			HAL_UART_Transmit(&huart1, off, sizeof(off), 100);
		}
	}
}
int a2i(uint8_t*);
void StartModulationMode();
void StopModulationMode();
void StartBlinkingMode(int);
void StopBlinkingMode();



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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, msg1, sizeof(msg1));
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		DHT_Start();
//		Presence = DHT_Check_Response();
//		Rh_byte1 = DHT_Read();
//		Rh_byte2 = DHT_Read();
//		Temp_byte1 = DHT_Read();
//		Temp_byte2 = DHT_Read();
//		SUM = DHT_Read();
//
//		TEMP = Temp_byte1;
//		RH = Rh_byte1;
//
//		Temperature = (float) TEMP;
//		Humidity = (float) RH
		if(stop_flag)
		{
			if(modulation_flag) StopModulationMode();
			else if(blinking_flag) StopBlinkingMode();
			stop_flag = false;
		}
		if(choice_flag)
		{

		switch (choice)
		{
				case 0:
					HAL_UART_Transmit_IT(&huart1, logg, sizeof(logg));
					choice = -1;
					choice_flag = false;
					break;
				case 8:
					StartModulationMode();
					choice_flag = false;
				case 9:
					break;
				default:
					if (choice!=-1)
					{
						uint8_t nhaplai[]="Error\n";
						HAL_UART_Transmit_IT(&huart1, nhaplai, sizeof(nhaplai));
						choice = 0;
					}
		}
	}
}




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720 -1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720 -1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 500 - 1;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartModulationMode()
{
	if(blinking_flag) return;
	HAL_UART_Transmit_IT(&huart1, modulation_mode, sizeof(modulation_mode));
	modulation_flag = true;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	uint8_t duty_cycle;
	for(uint32_t i = 0; (i < 10001); i = i + 100)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
		duty_cycle = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3) * 0.01;
		sprintf(duty_cycle_num , "Duty cycle : %d\n", duty_cycle);
		HAL_UART_Transmit_IT(&huart1, duty_cycle_num, sizeof(duty_cycle_num));
		HAL_Delay(400);
	}
}


void StopModulationMode()
{
	if(!modulation_flag) return;
	modulation_flag = false;
	HAL_UART_Transmit_IT(&huart1, stop_modulation_mode, sizeof(stop_modulation_mode));
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
	choice = 0;
}

void StartBlinkingMode(int x)
{
	if(modulation_flag) return;
	blinking_flag = true;
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_SET_COUNTER(&htim2, x);
}

void StopBlinkingMode()
{
	if(!blinking_flag) return;
	HAL_UART_Transmit_IT(&huart1, stop_blinking_mode, sizeof(stop_blinking_mode));
	blinking_flag = false;
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
	choice = 0;
}

int a2i(uint8_t* txt)
{
    int sum, digit, i;
    sum = 0;
    for (i = 0; i < strlen(txt); i++) {
//        digit = txt[i] - 0x30;
        sum = (sum * 10) + (txt[i] - '0');
    }
    return sum;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	previous_tim = current_tim;
	if (htim->Instance == htim1.Instance) {
		uint8_t onled[] = "LED ON\n";
		uint8_t offled[] = "LED OFF\n";
		HAL_UART_Transmit(&huart1, onled, sizeof(onled), 100);
		if (!led_reverse_flag) {
			for (int i = 0; i < 5; i++) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
				for (int i = 0; i < 2000; i++) {
					for (int j = 0; j < 500; j++) {
					}
				}
			}
		} else {
			for (int i = 4; i >= 0; i--) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
				for (int i = 0; i < 2000; i++) {
					for (int j = 0; j < 500; j++) {
					}
				}
			}
		}

		for (int i = 0; i < 5; i++) {
			HAL_GPIO_TogglePin(LED2_GPIO_Port, led[i]);
		}
		current_tim = HAL_GetTick();
		HAL_UART_Transmit(&huart1, offled, sizeof(offled), 100);
	}

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		__HAL_TIM_SET_COUNTER(&htim2, counter);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance==huart1.Instance)
  {
	  choice = a2i(msg1);
	  if(choice == 9) stop_flag = true;
	  if((modulation_flag && choice == 8) || (blinking_flag && choice == 7)) HAL_UART_Transmit(&huart1, be_on_operation, sizeof(be_on_operation), 5000);
	  else if(choice == 7 && !modulation_flag && !blinking_flag)
	  {
		  HAL_UART_Transmit(&huart1, blinking_mode, sizeof(blinking_mode), HAL_MAX_DELAY);
		  HAL_UART_Receive(&huart1, counter_from_x, sizeof(counter_from_x), HAL_MAX_DELAY);
		  counter = a2i(counter_from_x);
		  StartBlinkingMode(counter);
		  choice_flag = false;
	  }
	  if(choice != prev_choice) choice_flag = true;
	  prev_choice = choice;
	  HAL_UART_Receive_IT(&huart1, msg1, sizeof(msg1));
  }
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart1.Instance)
//	{
//
//	}
//
//}


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
