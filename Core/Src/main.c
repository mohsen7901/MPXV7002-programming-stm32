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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_OF_ADC_DATA 200
#define True 1
#define False 0
#define SAMPLING_FRQ 10     // 1 to 10
#define TERMINATE_TIME 1    // 1 to 60
#define MAX_SAMPLING_FRQ 10
#define MAX_PRESSURE_DATA 1000
//#define MEAN_PRESSURE_STEP_SIZE 1000/1
//#define MAX_NUM_MEAN 1  
#define TOTAL_NUM_DATA (MAX_PRESSURE_DATA/SAMPLING_FRQ*NUM_OF_ADC_DATA)
#define AVDD 3.3
#define ADC_RESISTANCE_RATIO 1.5
#define MAX_ADC_DATA 4095
#define ZERO_PRESSURE_VOLTAGE 2.5
#define SUPPLY_VOLTAGE 5.254


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint32_t adc_data[NUM_OF_ADC_DATA];
volatile float data[MAX_SAMPLING_FRQ];  // number of pressure data depends on sampling frequency
uint32_t adc_counter = 0;
uint32_t timer_counter = 0;
volatile uint8_t adc_ready = False;
uint32_t sum_of_adc_data = 0;  // this var stores sum of the sum of the recieved data from dma
//volatile uint16_t d_counter = 0;
//volatile float mean_pressure = 0;
volatile uint8_t data_ready = False;
volatile uint8_t p_counter = 0;  // An index for pressure
volatile uint16_t sampling_frq = SAMPLING_FRQ; //1 to 10
volatile uint16_t sampling_frq_backup;
volatile uint32_t total_num_data = MAX_PRESSURE_DATA * NUM_OF_ADC_DATA / SAMPLING_FRQ;
volatile uint16_t terminate_time = TERMINATE_TIME;  // the maximum time need to wait for measuring statistics data (1 - 60 second)
volatile uint16_t terminate_time_backup;
volatile uint16_t second_counter;
uint16_t num_of_means;
volatile float sum_of_means = 0;
volatile float means[600]; //an array with the max number of mean data required
uint16_t means_counter = 0;
//float previous_data = 0;
volatile float min_pressure;
volatile float max_pressure;
volatile float mean_pressure;
volatile float std_pressure;
volatile float min_adc_data;
volatile float max_adc_data;
volatile float mean_adc_data;
volatile float std_adc_data;
//volatile uint8_t timer_status = False;
//volatile uint8_t loop_counter = 0;
volatile uint8_t reset_data = False;
volatile uint8_t set_zero_pressure = False;
volatile float adc_zero_pressure = 2048;
volatile uint8_t is_first_time = True;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint32_t get_sum_of_data(void);
float measure_pressure(float data);
float meassure_mean_p(void);
float std_measure_pressure(float data);

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
  MX_ADC1_Init();
  MX_TIM1_Init();
//  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	num_of_means = terminate_time * sampling_frq;
	sampling_frq_backup = sampling_frq;
	terminate_time_backup = terminate_time;
	second_counter = terminate_time;
	
	HAL_ADC_Start_DMA(&hadc1, adc_data, NUM_OF_ADC_DATA);
	
	__HAL_TIM_SetAutoreload(&htim1, 64000/sampling_frq) ;
	HAL_TIM_Base_Start_IT(&htim1);
	//HAL_TIM_Base_Start_IT(&htim3);
	
	for(int i=0; i<600; i++)
	{
		means[i]=0;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(sampling_frq != sampling_frq_backup)
		{
			HAL_TIM_Base_Stop_IT(&htim1);
			sampling_frq_backup = sampling_frq;
			num_of_means = terminate_time * sampling_frq;
			means_counter = 0;
			p_counter =0;
			sum_of_means = 0;
			sum_of_adc_data = 0;
			total_num_data = NUM_OF_ADC_DATA *  MAX_PRESSURE_DATA / sampling_frq;
			for(int i=0; i<600; i++)
			{
				means[i]=0;
			}
			reset_data = True;
			set_zero_pressure = True;
			is_first_time = True;
			__HAL_TIM_SetAutoreload(&htim1, 64000/sampling_frq);
			__HAL_TIM_SetCounter(&htim1, 0);
			HAL_TIM_Base_Start_IT(&htim1);
			
		}
		
		if(terminate_time != terminate_time_backup)
		{
			terminate_time_backup = terminate_time;
			second_counter = terminate_time;
			num_of_means = terminate_time * sampling_frq;
			means_counter = 0;
			p_counter =0;
			sum_of_means = 0;
			sum_of_adc_data = 0;
			for(int i=0; i<600; i++)
			{
				means[i]=0;
			}
			reset_data = True;
			set_zero_pressure = True;
			is_first_time = True;
		}
//		if(timer_status == True)
//		{
//			timer_status = False;
//			//means_counter = 0;
//			//sum_of_means = sum_of_means - means[means_counter] + data[p_counter];
//			
//		}
		
		if(data_ready == True)
		{
			data_ready = False;
			
			if(p_counter >= sampling_frq)
			{
				p_counter =0;
			}
			data[p_counter] = meassure_mean_p();
			sum_of_adc_data = 0;
			
			
			sum_of_means = sum_of_means - means[means_counter] + data[p_counter];

			means[means_counter] = data[p_counter];
			//mean_adc_data = sum_of_means / (means_counter+1);
			
			if(num_of_means == 1)
			{
				mean_adc_data = means[means_counter];
				min_adc_data = means[means_counter];
				max_adc_data = means[means_counter];
				std_adc_data = 0;
			}
			else
			{
				if(is_first_time == True)
				{
					if(means_counter == (num_of_means-1))
						is_first_time = False;
					mean_adc_data = sum_of_means / (means_counter+1);
					// measuring STD
					std_adc_data = 0;
					for(uint16_t i = 0; i < num_of_means; i++)
					{
						std_adc_data += powf(means[i]-mean_adc_data , 2);				
					}
					std_adc_data = sqrtf(std_adc_data/(means_counter+1));
				}
				else
				{
					mean_adc_data = sum_of_means / num_of_means;
					// measuring STD
					std_adc_data = 0;
					for(uint16_t i = 0; i < num_of_means; i++)
					{
						std_adc_data += powf(means[i]-mean_adc_data , 2);				
					}
					std_adc_data = sqrtf(std_adc_data/num_of_means);
					//std_adc_data += adc_zero_pressure;
				}
			
			if (min_adc_data > means[means_counter])
			{
				min_adc_data = means[means_counter];
			}
			if (max_adc_data < means[means_counter])
			{
				max_adc_data = means[means_counter];
			}
			}
			
//			if(means[0] == 0  && means_counter == 1)
//			{
//				means[0] = means[1];
//				min_adc_data = means[1];
//			}
			
			means_counter++;
			p_counter++;
			
			mean_pressure = measure_pressure(mean_adc_data);
			std_pressure = std_measure_pressure(std_adc_data);
			std_pressure = std_pressure / mean_pressure * 100;
			min_pressure = measure_pressure(min_adc_data);
			max_pressure = measure_pressure(max_adc_data);

		}
		
		
		
		if(adc_ready == True)
		{
			adc_ready=False;
			sum_of_adc_data += get_sum_of_data();
			
		}
		
		if(reset_data == True)
		{
			reset_data = False;
			min_adc_data = 4095;
			max_adc_data = 0;
		}
		
		if(set_zero_pressure == True)
		{
			set_zero_pressure = False;
			adc_zero_pressure = mean_adc_data;
		}
		
		//loop_counter++;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 64000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
		adc_counter++;
		adc_ready = True;
	
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		timer_counter++;
		adc_counter = 0;
		data_ready = True;
		adc_ready = False;
		if (timer_counter == sampling_frq)  //timer reached 1 second!
		{
			timer_counter = 0;
			second_counter--;
			if(second_counter == 0)
			{
				second_counter = terminate_time;
				//timer_status = True;
				means_counter = 0;
				//data_ready = False;
				//adc_ready = False;
			}
		}
		//timer_status = False;
	}
//	if(htim == &htim3)
//	{
//		terminate_time--;
//		if(terminate_time == 0)
//		{
//			terminate_time = TERMINATE_TIME;
//			timer_status = True;
//			//data_ready = False;
//		}
//	}
}

uint32_t get_sum_of_data(void)
{
	uint32_t adc_temp_data = 0;
	for(uint8_t i = 0;i < NUM_OF_ADC_DATA;i++)
	{
		adc_temp_data += adc_data[i];
	}

	//adc_temp_data /= NUM_OF_ADC_DATA;
	return adc_temp_data;
}

float measure_pressure(float data)
{
	float temp_pressure = 5 / SUPPLY_VOLTAGE * (data * AVDD * ADC_RESISTANCE_RATIO / MAX_ADC_DATA) - ZERO_PRESSURE_VOLTAGE;
	return temp_pressure;
}

float std_measure_pressure(float data)
{
	float temp_pressure = 5 / SUPPLY_VOLTAGE * data * AVDD * ADC_RESISTANCE_RATIO / MAX_ADC_DATA;
	return temp_pressure;
}

float meassure_mean_p(void)
{
	float mean = sum_of_adc_data / (float)(total_num_data);
	return mean;
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
