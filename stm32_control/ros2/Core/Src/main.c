/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


uint8_t rx_data;

#define RX_BUF_SIZE 64
char rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;

/*
 * pitch: 上下舵机，PA6 / TIM3_CH1
 * yaw:   左右舵机，PA7 / TIM3_CH2
 */
int16_t pitch_angle = 90;
int16_t yaw_angle = 90;

/*
 * 安全限位，先不要直接打满 0~180
 */
#define YAW_MIN     20
#define YAW_MAX     160
#define PITCH_MIN   30
#define PITCH_MAX   150



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

static int16_t clamp_i16(int16_t value, int16_t min, int16_t max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

static uint16_t Servo_Angle_To_Pulse(uint8_t angle)
{
    if (angle > 180) {
        angle = 180;
    }

    return SERVO_MIN_PULSE +
           (uint16_t)((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180);
}

/* PA6 / TIM3_CH1：上下舵机 pitch */
void Servo_Set_UpDown(uint8_t angle)
{
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        Servo_Angle_To_Pulse(angle)
    );
}

/* PA7 / TIM3_CH2：左右舵机 yaw */
void Servo_Set_LeftRight(uint8_t angle)
{
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        Servo_Angle_To_Pulse(angle)
    );
}

static void Gimbal_Apply_Angle(void)
{
    yaw_angle = clamp_i16(yaw_angle, YAW_MIN, YAW_MAX);
    pitch_angle = clamp_i16(pitch_angle, PITCH_MIN, PITCH_MAX);

    Servo_Set_LeftRight((uint8_t)yaw_angle);
    Servo_Set_UpDown((uint8_t)pitch_angle);
}

static void UART_Send_String(const char *str)
{
    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)str,
        strlen(str),
        100
    );
}

static void Send_OK(void)
{
    UART_Send_String("#OK\r\n");
}

static void Send_ERR(void)
{
    UART_Send_String("#ERR\r\n");
}

static void Send_State(void)
{
    char tx_buf[64];

    snprintf(
        tx_buf,
        sizeof(tx_buf),
        "#STATE,%d,%d\r\n",
        yaw_angle,
        pitch_angle
    );

    UART_Send_String(tx_buf);
}

static void Protocol_Process_Line(char *line)
{
		int len = strlen(line);

    while (len > 0 &&
           (line[len - 1] == ' ' ||
            line[len - 1] == '\r' ||
            line[len - 1] == '\n' ||
            line[len - 1] == '\t'))
    {
        line[len - 1] = '\0';
        len--;
    }
		
    int yaw;
    int pitch;
    int dyaw;
    int dpitch;

    /*
     * #SET,yaw,pitch
     * 例如：#SET,90,90
     */
    if (sscanf(line, "#SET,%d,%d", &yaw, &pitch) == 2)
    {
        yaw_angle = (int16_t)yaw;
        pitch_angle = (int16_t)pitch;

        Gimbal_Apply_Angle();
        Send_OK();
        return;
    }

    /*
     * #MOVE,dyaw,dpitch
     * 例如：#MOVE,-2,0
     */
    if (sscanf(line, "#MOVE,%d,%d", &dyaw, &dpitch) == 2)
    {
        yaw_angle += (int16_t)dyaw;
        pitch_angle += (int16_t)dpitch;

        Gimbal_Apply_Angle();
        Send_OK();
        return;
    }

    /*
     * #CENTER
     */
    if (strcmp(line, "#CENTER") == 0)
    {
        yaw_angle = 90;
        pitch_angle = 90;

        Gimbal_Apply_Angle();
        Send_OK();
        return;
    }

    /*
     * #STOP
     * 对 SG90 来说，就是保持当前角度，不再更新
     */
    if (strcmp(line, "#STOP") == 0)
    {
        Send_OK();
        return;
    }

    /*
     * #GET
     */
    if (strcmp(line, "#GET") == 0)
    {
        Send_State();
        return;
    }

    Send_ERR();
}

static void Protocol_Receive_Byte(uint8_t data)
{
    /*
     * 只从 # 开始接收一帧
     * 这样可以丢弃噪声、空格、残留字符
     */
    if (data == '#')
    {
        rx_index = 0;
        rx_buf[rx_index++] = '#';
        return;
    }

    /*
     * 如果还没收到 #，忽略所有字符
     */
    if (rx_index == 0)
    {
        return;
    }

    /*
     * 收到换行，说明一帧结束
     */
    if (data == '\n' || data == '\r')
    {
        if (rx_index > 1)
        {
            rx_buf[rx_index] = '\0';
            Protocol_Process_Line(rx_buf);
        }

        rx_index = 0;
        return;
    }

    /*
     * 普通字符入缓冲区
     */
    if (rx_index < RX_BUF_SIZE - 1)
    {
        rx_buf[rx_index++] = (char)data;
    }
    else
    {
        rx_index = 0;
        Send_ERR();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

yaw_angle = 90;
pitch_angle = 90;
Gimbal_Apply_Angle();

UART_Send_String("#BOOT,OK\r\n");


	/* ��������Ȼ��� */
	//Servo_Set_UpDown(90);
	//Servo_Set_LeftRight(90);

	//HAL_Delay(1000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		if (HAL_UART_Receive(&huart1, &rx_data, 1, 10) == HAL_OK)
		{
				Protocol_Receive_Byte(rx_data);
		}


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 1000;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
#ifdef USE_FULL_ASSERT
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
