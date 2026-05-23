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



#include "sensor_module.h"
#include "utils.h"
#include "bluetooth.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

Sensor sensors[NUM_SENSORS];
Sensor_Array sensor_array;
PID_Controller pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int sensor_weights[NUM_SENSORS] = { 0, -3, -2, -1,1, 2, 3, 0 };

volatile int line;
volatile int line2;
volatile int correction;
volatile int left_speed;
volatile int right_speed;
int left;
int right;
int start = 0;
volatile float dt ;

uint8_t rx_data;
char rx_buffer[30];
int rx_index = 0;
float b ;
JunctionType j;

volatile uint8_t junction_handling_enabled = 1;

#define CURVE_ENTRY_THRESHOLD  600.0f
#define CURVE_EXIT_THRESHOLD   300.0f
#define CURVE_KP_SCALE          1.5f
#define CURVE_KD_SCALE          1.8f
#define CURVE_SPEED_SCALE       0.60f

static uint8_t in_curve = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == GPIO_PIN_14)
    {
    	if(start == 1){
    		start = 0;
    	}
    	else if (start == 0){
    	start = 1;
    	}


    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rx_data == '\n' || rx_data == '\r') {

			rx_buffer[rx_index] = '\0';
			rx_index = 0;


			char *cmd = rx_buffer;


			if (strncmp(cmd, "PID:", 4) == 0) {
				float p, i, d;
				if (sscanf(cmd + 4, "%f,%f,%f", &p, &i, &d) == 3) {
					pid.Kp = p;
					pid.Ki = i;
					pid.Kd = d;
					pid.integral = 0;
					pid.last_error = 0;
					pid.d_filtered = 0;
				}
			}

			else if (strncmp(cmd, "PARAM:", 6) == 0) {
			    char *bs = strstr(cmd, "BS");
			    char *pl = strstr(cmd, "PL");
			    char *jh = strstr(cmd, "JH");
			    if (bs) sensor_array.base_speed = atoi(bs + 2);
			    if (pl) pid.limit = atoi(pl + 2);
			    if (jh) junction_handling_enabled = atoi(jh + 2);
			}


			else if (strcmp(cmd, "START") == 0) {
				start = 1;
				in_curve = 0;        // always start fresh in straight mode
				pid.integral = 0;
				pid.last_error = 0;
				pid.d_filtered = 0;

			} else if (strcmp(cmd, "STOP") == 0) {
				start = 0;
				in_curve = 0;        // clear so next start is clean
				pid.integral = 0;
			}

			else if (strlen(cmd) > 1) {
				char type = cmd[0];
				float val = atof(&cmd[1]);
				if (type == 'P')
					pid.Kp = val;
				else if (type == 'I')
					pid.Ki = val;
				else if (type == 'D')
					pid.Kd = val;
				else if (type == 'B')
					sensor_array.base_speed = (int) val;
				else if (type == 'X')
					start = 0;
				else if (type == 'O')
					start = 1;
			}


		} else {
			if (rx_index < sizeof(rx_buffer) - 1){
			rx_buffer[rx_index++] = rx_data;}
		}

		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}




void Send_Telemetry() {
    static char buf[160];
    char ir_part[80] = "IR:";
    char tmp[8];



    for (int i = 0; i < sensor_array.number_of_sensors; i++) {
        snprintf(tmp, sizeof(tmp), "%u", (uint16_t)sensor_array.array[i].adc_raw);
        strcat(ir_part, tmp);
        if (i < sensor_array.number_of_sensors - 1) strcat(ir_part, ",");
    }




    snprintf(buf, sizeof(buf),
        "%s;PL:%d;PR:%d;BV:%.1f;PE:%.1f;PO:%.1f;JC:%d\n",
        ir_part,
        (int)(sensor_array.base_speed + correction),
        (int)(sensor_array.base_speed - correction),
        b, (float)line, (float)correction
    );

    HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, strlen(buf));
}

/* USER CODE BEGIN 4 */
float get_line_error_continuous(Sensor_Array *array) {
    float sum_weighted = 0.0f;
    float sum_values   = 0.0f;
    float max_val      = 0.0f;
    int   active_count = 0;

    for (int i = 0; i < array->number_of_sensors; i++) {
        if (array->weights[i] == 0) continue;  // skip non-working sensors

        float val = (float)array->array[i].adc_raw;
        sum_weighted += (float)array->weights[i] * val;
        sum_values   += val;
        if (val > max_val) max_val = val;
        if (val > 1500.0f) active_count++;
    }

    if (max_val < 1500.0f) {
        return 0.0f;
    }

    if (active_count >= 4) return 0.0f;

    float result = (sum_weighted / sum_values) * 500.0f;

    if (fabsf(result) < 50.0f) result = 0.0f;

    return (result);
}
/* USER CODE END 4 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dma_buffer, NUM_SENSORS + 1);

	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	sensor_array.number_of_sensors = NUM_SENSORS;
	sensor_array.array = sensors;
	sensor_array.weights = sensor_weights;
	sensor_array.base_speed = 500;

	Initialize_Sensor_Array(&sensor_array);

	pid.Kd = 5;
	pid.Ki = 0;
	pid.Kp = 10;
	pid.integral = 0;
	pid.last_error = 0;
	pid.limit = 300;
	pid.d_filtered = 0;
	pid.d_alpha = 0.85f;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	uint32_t last_time = HAL_GetTick();
	static uint32_t last_telem = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


		b = battery_voltage(dma_buffer);

		Sync_Sensors(&sensor_array);

		uint32_t current_time = HAL_GetTick();
		uint32_t time_diff = current_time - last_time;

		if (time_diff == 0) {
			continue;
		}
		else{


		dt = time_diff / 1000.0f;
		last_time = current_time;}

		processSensors(&sensor_array);
		binarizeSensors(&sensor_array);

		if ((HAL_GetTick() - last_telem) >= TELEM_INTERVAL_MS) {
		    last_telem = HAL_GetTick();
		    Send_Telemetry();
		}

		if (start == 1) {


			j = NO_JUNCTION;
						if (junction_handling_enabled) {
							j = detect_junction_digital(&sensor_array);
						}
					    if (j != NO_JUNCTION) {
					        handle_junction(&sensor_array, j, 900);

					        pid.integral   = 0;
					        pid.last_error = 0;
					        in_curve       = 0;
					        pid.d_filtered = 0;
					    }

					    else {
					    	float error_f = get_line_error_continuous(&sensor_array);
					    	line = (int)error_f;

					    	float abs_err = fabsf(error_f);

					    					    	/* Hysteresis: separate entry/exit thresholds prevent
					    					    	 * gain chattering when error hovers near the boundary */
					    	if (!in_curve && abs_err > CURVE_ENTRY_THRESHOLD) {
					    			in_curve = 1;
					    			pid.integral = 0;
					    	}
					    	else if (in_curve && abs_err < CURVE_EXIT_THRESHOLD) {
					    			in_curve = 0;
					    			pid.integral = 0;
					    	}

					    	if (in_curve) {
					    			float saved_kp = pid.Kp;
					    			float saved_kd = pid.Kd;
					    			int   saved_bs = sensor_array.base_speed;

					    			pid.Kp = saved_kp * CURVE_KP_SCALE;
					    			pid.Kd = saved_kd * CURVE_KD_SCALE;
					    			sensor_array.base_speed = (int)(saved_bs * CURVE_SPEED_SCALE);

					    			pid.d_alpha = 0.50f;
					    			correction = calculate_pid(&pid, line, dt);
					    			pid.d_alpha = 0.85f;

					    			follow_line(correction, &sensor_array);

					    			pid.Kp = saved_kp;
					    			pid.Kd = saved_kd;
					    			sensor_array.base_speed = saved_bs;
					    	} else {
					    			correction = calculate_pid(&pid, line, dt);
					    			follow_line(correction, &sensor_array);
					    					    	}
					    					    }
					    					}
		else {
		    set_motor_speed(0, 0, b);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
	while (1) {
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
