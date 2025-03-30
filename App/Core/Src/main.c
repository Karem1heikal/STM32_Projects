/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "SERVO_interface.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_IN1_PIN	GPIO_PIN_0
#define MOTOR1_IN2_PIN	GPIO_PIN_1

#define MOTOR2_IN3_PIN	GPIO_PIN_2
#define MOTOR2_IN4_PIN	GPIO_PIN_3

#define MOTOR1_GPIO_PORT	GPIOB
#define MOTOR2_GPIO_PORT	GPIOB

#define MOTOR1_PWM	TIM_CHANNEL_1
#define MOTOR2_PWM	TIM_CHANNEL_2
#define SERVO_PWM	TIM_CHANNEL_2

#define TRIG_LEFT1 GPIO_PIN_6  // Left Sensor 1
#define ECHO_LEFT1 GPIO_PIN_13
#define TRIG_LEFT2 GPIO_PIN_7  // Left Sensor 2
#define ECHO_LEFT2 GPIO_PIN_14

#define TRIG_RIGHT1 GPIO_PIN_8  // Right Sensor 1
#define ECHO_RIGHT1 GPIO_PIN_15
#define TRIG_RIGHT2 GPIO_PIN_12  // Right Sensor 2
#define ECHO_RIGHT2 GPIO_PIN_11  //Note : GPIOA -> pin_11,pin_12

#define TRIG_FRONT GPIO_PIN_4
#define ECHO_FRONT GPIO_PIN_10
#define TRIG_BACK GPIO_PIN_5
#define ECHO_BACK GPIO_PIN_12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxdata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Movement state variables
uint8_t forward = 0;
uint8_t backward = 0;
uint8_t left = 0;
uint8_t right = 0;
uint8_t rxBuffer[1];
uint8_t self_parking_enabled = 0;

//char Dist1[]="12";
//uint8_t Dist1_dash=0;

// Function prototypes
void update_movement();
void set_motor_direction(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, int forward);
void set_motor_speed(uint8_t motor1_speed, uint8_t motor2_speed);

uint32_t get_distance(GPIO_TypeDef* GPIOx, uint16_t TRIG, uint16_t ECHO);
uint8_t check_parking_space();
void self_parking_mode();
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  /*
  HAL_GPIO_WritePin(BlueTooth_GPIO_Port , BlueTooth_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);*/
  //memset(buffer,0,sizeof(buffer));
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	SERVO_voidInit(htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	// Start UART reception in interrupt mode
	HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
	HAL_TIM_Base_Start(&htim4);
	HAL_GPIO_WritePin(GPIOB, TRIG_FRONT, GPIO_PIN_RESET);
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

	//HAL_UART_Receive_IT(&huart1, &rxdata, 1);

  /* USER CODE END 2 */

 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*
		SERVO_voidRotate(TIM_CHANNEL_2, 180);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,250);
		HAL_Delay(1000);
		SERVO_voidRotate(TIM_CHANNEL_2, 90);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,450);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,450);
		HAL_Delay(1000);
		SERVO_voidRotate(TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1050);
		HAL_Delay(1000);*/
/*
		Dist1_dash = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);
		Dist1[0]=(Dist1_dash/10+'0');
		Dist1[1]=(Dist1_dash%10+'0');
		Dist1[2]= 0x0d;
		HAL_UART_Transmit(&huart1, (uint8_t*)Dist1, sizeof(Dist1), 10);*/


		if(self_parking_enabled == 1)
		{
			self_parking_mode();
		}
		else
		{
			update_movement();
		}

		//check_parking_space();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA12 PA13 
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 
                           PB4 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		char command = rxBuffer[0];
		// Update movement states based on command
		switch (command) {
		case 'W': forward = 1; backward = 0; break; // Move forward
		case 'S': backward = 1; forward = 0; break; // Move backward
		case 'A': left = 1; break;                 // Turn left
		case 'D': right = 1; break;                // Turn right
		case 'X':                                  // Stop all motion
			forward = 0;
			backward = 0;
			left = 0;
			right = 0;
			break;
		default: break;
		}

		HAL_UART_Receive_IT(&huart1, rxBuffer, 1); // Restart UART reception
	}
}
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	char command;
	if (huart->Instance == USART1) {
		command = rxBuffer[0];
		// Update movement states based on command
		switch (command) {
		case 'W': forward = 1; backward = 0; break; // Move forward
		case 'S': backward = 1; forward = 0; break; // Move backward
		case 'A': left = 1; break;                 // Turn left
		case 'D': right = 1; break;                // Turn right
		case 'X':                                  // Stop all motion
			forward = 0;
			backward = 0;
			left = 0;
			right = 0;
			break;
		case 'P':
			self_parking_enabled = 1;
			break;
		default: break;
		}


		HAL_UART_Receive_IT(&huart1, rxBuffer, 1); // Restart UART reception
	}
}
void update_movement() {
	if (forward && left) {
		// Forward + Left
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(50, 70); // Left motor slower, right motor faster
		SERVO_voidRotate(SERVO_PWM, 45);    // Turn servo to the left
		left=0;
		HAL_Delay(10);
	} else if (forward && right) {
		// Forward + Right
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(70, 50); // Right motor slower, left motor faster
		SERVO_voidRotate(SERVO_PWM,135);   // Turn servo to the right
		right=0;
		HAL_Delay(10);
	} else if (backward && left) {
		// Backward + Left
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
		set_motor_speed(30, 50); // Left motor slower, right motor faster
		SERVO_voidRotate(SERVO_PWM,45);     // Turn servo to the left
		left=0;
		HAL_Delay(10);
	} else if (backward && right) {
		// Backward + Right
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
		set_motor_speed(50, 30); // Right motor slower, left motor faster
		SERVO_voidRotate(SERVO_PWM,135);    // Turn servo to the right
		right=0;
		HAL_Delay(10);
	} else if (forward) {
		SERVO_voidRotate(SERVO_PWM,90); // Straight
		// Forward only
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(70, 70);
		right=0;
		HAL_Delay(10);
	} else if (backward) {
		SERVO_voidRotate(SERVO_PWM,90); // Straight
		// Backward only
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
		set_motor_speed(30, 30);
		HAL_Delay(10);
	}
	else {
		// Stop all motion
		forward=0;
		set_motor_speed(0, 0);
		SERVO_voidRotate(SERVO_PWM,90); // Straight
		HAL_Delay(10);
	}
}
void set_motor_speed(uint8_t motor1_speed, uint8_t motor2_speed) {
	uint32_t max_speed = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR1_PWM, (motor1_speed * max_speed) / 100);
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR2_PWM, (motor2_speed * max_speed) / 100);
}
void set_motor_direction(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, int forward) {
	if (forward) {
		HAL_GPIO_WritePin(port, pin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(port, pin2, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(port, pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin2, GPIO_PIN_SET);
	}
}
uint32_t get_distance(GPIO_TypeDef* GPIOx, uint16_t TRIG, uint16_t ECHO) {
    uint32_t start_time, end_time,pMillsec,distance;
    HAL_GPIO_WritePin(GPIOx, TRIG, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim4,0);
    while(__HAL_TIM_GET_COUNTER(&htim4)<10);
    HAL_GPIO_WritePin(GPIOx, TRIG, GPIO_PIN_RESET);
    pMillsec = HAL_GetTick();
    while(!(HAL_GPIO_ReadPin(GPIOx, ECHO)) && pMillsec + 10 >HAL_GetTick());
    start_time = __HAL_TIM_GET_COUNTER(&htim4);

    pMillsec = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    while ((HAL_GPIO_ReadPin (GPIOx, ECHO)) && pMillsec + 50 > HAL_GetTick());
    end_time = __HAL_TIM_GET_COUNTER (&htim4);
    distance = (end_time - start_time)* 0.034/2;
    HAL_Delay(10);
    return distance;
}
void self_parking_mode() {
    uint8_t space_found = 0;
    uint32_t start_time = HAL_GetTick();  // Get current time
    uint32_t search_time_limit = 10000;   // 10 seconds search limit

    // Move forward to search for a parking space
    SERVO_voidRotate(SERVO_PWM,90);
    HAL_Delay(10);
    set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
    set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
    set_motor_speed(40, 40);

    while (!space_found) {
        space_found = check_parking_space();

        // Stop searching if time limit is reached
        if (HAL_GetTick() - start_time > search_time_limit) {
        	//HAL_UART_Transmit(&huart1, (uint8_t*)Dist1, sizeof(Dist1), 10);
            set_motor_speed(0, 0);  // Stop car
            self_parking_enabled = 0;  // Exit parking mode
            return;
        }
    }

    // Stop when space is found
    set_motor_speed(0, 0);

    if (space_found == 1) {  // Left-side parking
        // Reverse and turn into space
        SERVO_voidRotate(SERVO_PWM,45);
        set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
        set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
        set_motor_speed(30, 50);

        while (get_distance(GPIOB, TRIG_BACK, ECHO_BACK) > 15) {
            HAL_Delay(100);
        }

        // Final alignment: Move forward & steer right
        SERVO_voidRotate(SERVO_PWM,135);
        set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
        set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
        set_motor_speed(30, 30);
        HAL_Delay(500);
    }

    else if (space_found == 2) { // Right-side parking
        SERVO_voidRotate(SERVO_PWM,135);
        set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
        set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
        set_motor_speed(50, 30);

        while (get_distance(GPIOB, TRIG_BACK, ECHO_BACK) > 15) {
            HAL_Delay(100);
        }

        // Final alignment: Move forward & steer left
        SERVO_voidRotate(SERVO_PWM,45);
        set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
        set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
        set_motor_speed(30, 30);
        HAL_Delay(500);
    }

    // Straighten wheels and stop
    SERVO_voidRotate(SERVO_PWM,90);
    set_motor_speed(0, 0);

    // Disable parking flag after completion
    self_parking_enabled = 0;
}
// Check Parking Space
uint8_t check_parking_space() {

    if (get_distance(GPIOB, TRIG_LEFT1, ECHO_LEFT1) > 30 && get_distance(GPIOB, TRIG_LEFT2, ECHO_LEFT2) > 30) {
        return 1;  // Left side
    }
    if (get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1) > 30 && get_distance(GPIOA, TRIG_RIGHT2, ECHO_RIGHT2) > 30) {
        return 2;  // Right side
    }
    return 0;
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
