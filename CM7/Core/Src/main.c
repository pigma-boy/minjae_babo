/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double FastArcTan(double x);
void InitLUT(void);
void SensorlessLUT(void);
float LowPassFilter(float x,float x_prev);
void generate_L_sin();
float calculate_average(float *values, int count);
int motor_start=0;
int duty=30, duty1=20, duty_pre=0;
char MSG[50];

float Kp= 0.1, Ki=0.1, Kp_par1=0.01, Ki_par1= 0.001;

int p1_LUT[1024], p2_LUT[1024], p3_LUT[1024];
int p1_ssLUT[3000]={0,}, p2_ssLUT[3000]={0,}, p3_ssLUT[3000]={0,};
int enc_c = 0;
int angle_on = 512;
int angle_off = 853;
uint16_t adc_val[4];
uint16_t uvw_val[1];
uint16_t current_offset[4]={0};
uint16_t uvw_offset[1]={0};
int hfp_1=0,hfp_2=0,hfp_3=0;

int current =0, current_error =0, current_error_sum=0, current_ref=2000;

float L_a =0, L_b=0,L_c=0,Lma=0 ,Lna=0, Lmb=0,Lnb=0,Lnc=0,Lmc=0;

int tim8_count,tim8_start,tim8_finish, tim8_count2, tim1_count3,tim8_start2,tim8_finish2,tim1_start3,tim1_finish3;

int chai_a, chai_b,chai_c;
int example1=0,example2=0,example3=0,example4=0,example5=0,example6=0;
int chai_plus_a=0,chai_minus_a=0,chai_plus_b=0,chai_minus_b=0,chai_plus_c=0,chai_minus_c=0;
int example1_plus_a=0,example1_minus_a=0,example2_plus_a=0,example2_minus_a=0,example3_plus_b=0,example3_minus_b=0;
int example4_plus_b=0,example4_minus_b=0,example5_plus_c=0,example5_minus_c=0,example6_plus_c=0,example6_minus_c=0;
int stop_timer=0, sensorless_start=0;
int lut_angle=0,prev_lut_angle;
float plus_di_dt_a=0,minus_di_dt_a=0,plus_di_dt_b=0,minus_di_dt_b=0,plus_di_dt_c=0,minus_di_dt_c=0;

uint16_t enc_c_count=0;

int enc_z_phase=0;

float theta_a=0, angle_a=0, theta_b=0, angle_b=0, theta_c=0, angle_c=0;
int sicid_start=0;
int A_switch=0 ,B_switch=1 ,C_switch=0;
float past_L_a=0,past_L_b=0,past_L_c=0;

int lut_angle_prev =0,dif_angle=0;
unsigned int gain_L_start =0, gain_L_wait =0,gain_L_stop=0, one_time =0, seq=0;

float max_L_a =0, min_L_a =5000,max_L_b =0, min_L_b =5000,max_L_c =0, min_L_c =5000;

float L_sin[4000];

float start_time=0, elapsed_time=0, final_time=0;
int trigger_count=0, measurement_complete=0;
int previous_elapsed_time =0, new_elapsed_time=0;

int L_a_sin_pre =0, L_b_sin_pre =0, L_c_sin_pre=0;
float tim8_nums=0;
int seq_a=0,seq_b=0,seq_c=0;
float inc=0,new_inc=0;
float L_a_sin=0,L_b_sin=0,L_c_sin=0;

float past2_L_a=0,past2_L_b=0,past2_L_c=0, avg_L_a=0,avg_L_b=0,avg_L_c=0;

int encoder =1;
uint16_t a_ready =0,b_ready =0,c_ready =0;
int check =0;
int sin_seq_a=0,sin_seq_b=0,sin_seq_c=0;

float used_L_a=0,used_L_b=0,used_L_c=0;

int p1=0,p2=0,p3=0;
int reconstruction=0;

float L_a_filtered=1;
float L_a_filtered_values[20] = {0};
int filtered_index_a=0;
float L_a_filtered_average=0;

int temp_chai_a1=0,current_num_a1=0;
int temp_chai_a2=0,current_num_a2=0;
float prev_L_a=0;

float L_b_filtered=1;
float L_b_filtered_values[20] = {0};
int filtered_index_b=0;
float L_b_filtered_average=0;

int temp_chai_b1=0,current_num_b1=0;
int temp_chai_b2=0,current_num_b2=0;
float prev_L_b=0;

float L_c_filtered=1;
float L_c_filtered_values[20] = {0};
int filtered_index_c=0;
float L_c_filtered_average=0;

int temp_chai_c1=0,current_num_c1=0;
int temp_chai_c2=0,current_num_c2=0;
float prev_L_c=0;

int index_num=4;

int tim4_cnt=0;
int tim8_ccr[3]={0,};
float prev2_L_a=0,prev2_L_b=0,prev2_L_c=0;
int temp=6000000;
int counttt=750,countt=8500, countttt=10000, count5=100;
int sicid_val=150;
uint16_t first_triggered = 0;  // 첫 번째 트리거 여부
uint16_t min_recon=600;

uint16_t converted_L_a=0, converted_L_a_filtered_average=0;
uint16_t converted_L_b=0, converted_L_b_filtered_average=0;
uint16_t converted_L_c=0, converted_L_c_filtered_average=0;
uint16_t converted_a_ready=0,converted_b_ready=0,converted_c_ready=0;
uint16_t converted_lut_angle=0;
uint16_t current_value=0;

int measure_minmax=0, reset_minmax=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));9
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */







  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  InitLUT();
  SensorlessLUT();
  generate_L_sin();
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  HAL_TIM_Base_Start_IT(&htim2); // Timer 2 Start
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_ADC_Start_DMA(&hadc3, &adc_val[0], 4);
  HAL_ADC_Start_DMA(&hadc1, &uvw_val[0], 1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  //HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
 // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &converted_L_a_filtered_average, 1, DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &converted_L_a_filtered_average, 1, DAC_ALIGN_12B_R);
 // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, &converted_L_c_filtered_average, 1, DAC_ALIGN_12B_R);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // Start Motor
  HAL_Delay(1000);

  TIM8->CCR1 = 30;
  TIM8->CCR2 = 0;
  TIM8->CCR3 = 0;
  HAL_Delay(2000);


  // Intialize the Encoder 2

	TIM4->CNT = 0;
	TIM1->CNT = 0;

	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;
	TIM8->CCR3 = 0;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1000);


	for(int i=0;i<10;i++){
		uvw_offset[0]+=uvw_val[0]/10;
	  for(int j=1;j<4;j++) current_offset[j]+=adc_val[j]/10;
	}
	motor_start = encoder;
	sicid_start=1-encoder;

  /* USER CODE END 2 */

  /* Initialize leds */


  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,L_a);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//   HAL_Delay(500);          // 1초 대기


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T1_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 50-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14&& p1_LUT[enc_c]) {
        if (!first_triggered) {
            // 첫 번째 EXTI 발생 시: CNT 값을 enc_z_phase에 저장
            enc_z_phase = TIM4->CNT;
            first_triggered = 1;
        }
        else {
            // 첫 번째 트리거 이후에만 카운트
        	trigger_count++;

            // 5번에 한 번씩만 TIM4->CNT를 enc_z_phase로 복원
            if (trigger_count >= 5) {
                TIM4->CNT = enc_z_phase;
                trigger_count = 0; // 카운터 초기화
            }
        }
    }
}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_14&& p1_LUT[enc_c]) {
	  TIM4->CNT = 3708;
	  enc_z_phase = TIM4->CNT;
  } else {
	  __NOP();
  }
} */
// Timer 1,8 interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == htim8.Instance)
	{
		tim8_finish=TIM6->CNT;
		tim8_count=tim8_finish-tim8_start;
		tim8_start= TIM6->CNT;
		tim8_start2= TIM6->CNT;


		chai_plus_a = (current_num_a1 != 0) ? (temp_chai_a1 / current_num_a1) : 0;
		chai_minus_a = (current_num_a2 != 0) ? (temp_chai_a2 / current_num_a2) : 0;
		chai_plus_b = (current_num_b1 != 0) ? (temp_chai_b1 / current_num_b1) : 0;
		chai_minus_b = (current_num_b2 != 0) ? (temp_chai_b2 / current_num_b2) : 0;
		chai_plus_c = (current_num_c1 != 0) ? (temp_chai_c1 / current_num_c1) : 0;
		chai_minus_c = (current_num_c2 != 0) ? (temp_chai_c2 / current_num_c2) : 0;

		temp_chai_a1=0;
		current_num_a1=0;
		temp_chai_a2=0;
		current_num_a2=0;

		temp_chai_b1=0;
		current_num_b1=0;
		temp_chai_b2=0;
		current_num_b2=0;

		temp_chai_c1=0;
		current_num_c1=0;
		temp_chai_c2=0;
		current_num_c2=0;

		if(chai_plus_a==0||chai_minus_a==0) L_a= prev_L_a;
		else
			{
		plus_di_dt_a=chai_plus_a;
		minus_di_dt_a=chai_minus_a;
				L_a = temp/(plus_di_dt_a-minus_di_dt_a);
				if(L_a>=4096) L_a=4096;
			}
				L_a_filtered = 0.8*L_a + 0.2 * L_a_filtered;
				//if((L_a-L_a_filtered))
				L_a_filtered_values[filtered_index_a] = L_a_filtered;
				if(filtered_index_a>=index_num)
					{
					L_a_filtered_average = calculate_average(L_a_filtered_values, index_num);
					filtered_index_a =1;
					}
				else filtered_index_a++;


		if(chai_plus_b==0||chai_minus_b==0) L_b= prev_L_b;
		else
			{
		plus_di_dt_b=chai_plus_b;
		minus_di_dt_b=chai_minus_b;
				L_b = temp/(plus_di_dt_b-minus_di_dt_b);
				if(L_b>=4096) L_b=4096;
			}
				L_b_filtered = 0.8*L_b + 0.2 * L_b_filtered;

				L_b_filtered_values[filtered_index_b] = L_b_filtered;
				if(filtered_index_b>=index_num)
					{
					L_b_filtered_average = calculate_average(L_b_filtered_values, index_num);
					filtered_index_b =1;
					}
				else filtered_index_b++;


		if(chai_plus_c==0||chai_minus_c==0) L_c= prev_L_c;
		else
			{
		plus_di_dt_c=chai_plus_c;
		minus_di_dt_c=chai_minus_c;
				L_c = temp/(plus_di_dt_c-minus_di_dt_c);
				if(L_c>=4096) L_c=4096;
			}
				L_c_filtered = 0.8* L_c + 0.2 * L_c_filtered;

				L_c_filtered_values[filtered_index_c] = L_c_filtered;
				if(filtered_index_c>=index_num)
					{
					L_c_filtered_average = calculate_average(L_c_filtered_values, index_num);
					filtered_index_c =1;
					}
				else filtered_index_c++;




				used_L_a=L_a_filtered_average;
				used_L_b=L_b_filtered_average;
				used_L_c=L_c_filtered_average;

				if(adc_val[1] > 35000) L_a_filtered_average = 1500;
				if(adc_val[2] > 35000) L_b_filtered_average = 1500;
				if(adc_val[3] > 35000) L_c_filtered_average = 1500;


				//sicid의 안정성을 높이기 위해 2번 기울기 비교 후, 진입

				prev_L_a = L_a_filtered_average;
				prev_L_b = L_b_filtered_average;
				prev_L_c = L_c_filtered_average;

			    prev2_L_a = prev_L_a;
			    prev2_L_b = prev_L_b;
			    prev2_L_c = prev_L_c;


			if(L_a_filtered_average>L_c_filtered_average&&L_a_filtered_average>L_b_filtered_average) //a ??????????????????  ? ??  ?
				{
					Lma=L_a_filtered_average-((L_b_filtered_average+L_c_filtered_average)/2);
					Lna=0.866*(L_b_filtered_average-L_c_filtered_average);
					theta_a=FastArcTan(Lna/Lma)+6.28;
					 angle_a=45*(theta_a/3.14)*15;

				}

				if(L_b_filtered_average>L_c_filtered_average&&L_b_filtered_average>L_a_filtered_average) //b ??????????????????  ? ??  ?
								{
					Lmb=L_b_filtered_average-((L_a_filtered_average+L_c_filtered_average)/2);
					Lnb=0.866*(L_c_filtered_average-L_a_filtered_average);
					theta_b=FastArcTan(Lnb/Lmb)+2.09;
					angle_a=45*(theta_b/3.14)*15;
								}

				if(L_c_filtered_average>L_a_filtered_average&&L_c_filtered_average>L_b_filtered_average) //c ??????????????????  ? ??  ?
								{
					Lmc=L_c_filtered_average-((L_b_filtered_average+L_a_filtered_average)/2);
					Lnc=0.866*(L_a_filtered_average-L_b_filtered_average);
					theta_c=FastArcTan(Lnc/Lmc)+4.18;
					angle_a=45*(theta_c/3.14)*15;
								}

			/*	if(angle_a>max_angle_a)
					max_angle_a=angle_a;
				if(angle_a<min_angle_a)
					min_angle_a=angle_a;  */



				if(measure_minmax)
				{	stop_timer++;
				if(L_a_filtered_average>max_L_a)
						max_L_a=L_a_filtered_average;
				if(L_a_filtered_average<min_L_a&& L_a_filtered_average>50)
						min_L_a=L_a_filtered_average;

				if(L_b_filtered_average>max_L_b)
						max_L_b=L_b_filtered_average;
				if(L_b_filtered_average<min_L_b&& L_b_filtered_average >50)
						min_L_b=L_b_filtered_average;

				if(L_c_filtered_average>max_L_c)
						max_L_c=L_c_filtered_average;
				if(L_c_filtered_average < min_L_c&& L_c_filtered_average>50)
						min_L_c=L_c_filtered_average;
				}
				if(reset_minmax)
				{
					max_L_a=0;min_L_a=5000;max_L_b=0;min_L_b=0;max_L_c=0;min_L_c=0;
				}
				if(stop_timer <=30000)
				{
					stop_timer++;
				}
				if(stop_timer>29999)
					{
						//sicid_start=0;
						//sensorless_start=1-encoder;
					motor_start = encoder;
					sicid_start=1-encoder;
					}

			/*	if(stop_timer>64999)
				gain_L_start=1;  */


				current = uvw_val[0]- uvw_offset[0];
				if(current<=0) current=0;
				current_error = current_ref - current;
				current_error_sum += current_error/10;
				if(current_error_sum>20000) current_error_sum=20000;
				if(current_error_sum<-20000) current_error_sum=-20000;
				Kp = Kp_par1 * current_error;
				Ki = Ki_par1 * current_error_sum;
				duty_pre = (15 + Kp + Ki);

				if (duty_pre >= 40) duty = 35;
				else if (duty_pre<=1) duty = 1;
				else duty = duty_pre;

				lut_angle = (int)angle_a;
				converted_lut_angle = (uint16_t)lut_angle;


		/*		dif_angle=lut_angle-lut_angle_prev;
				lut_angle_prev = lut_angle;



				if (dif_angle < -1200) {
										final_time=TIM6->CNT;
										new_elapsed_time = final_time - start_time; // ?   번째 조건 충족 ?   경과 ?   ? 계산
										start_time = final_time;
										}
										if (new_elapsed_time >= 5000)
										{
											previous_elapsed_time = elapsed_time; // ?  ?   값을 ???
											elapsed_time = new_elapsed_time;     // ?  로운  ???? ?  ?  ?  ?
										}
*/
						tim8_finish2=TIM6->CNT;
						tim8_count2=tim8_finish2-tim8_start2;
						 converted_L_a = (uint16_t)L_a;
						 converted_L_a_filtered_average = (uint16_t)L_a_filtered_average;
						 converted_L_b = (uint16_t)L_b;
						 converted_L_b_filtered_average = (uint16_t)L_b_filtered_average;
						 converted_L_c = (uint16_t)L_c;
						 converted_L_c_filtered_average = (uint16_t)L_c_filtered_average;
						 converted_a_ready = a_ready*3000;
						 converted_b_ready = b_ready*3000;
						 converted_c_ready = c_ready*3000;

	}



	if(htim->Instance == htim1.Instance)
	{

		tim1_start3= TIM6->CNT;
		current_value = uvw_val[0]/16;



		if(motor_start){
			if(example1<current_offset[1]+count5) hfp_1 =duty1;
			else hfp_1 =0;
			if(adc_val[2]<current_offset[2]+count5) hfp_2 =duty1;
			else hfp_2 =0;
			if(adc_val[3]<current_offset[3]+count5) hfp_3 =duty1;
			else hfp_3 =0;

					enc_c=TIM4->CNT%1024;
					tim4_cnt=TIM4->CNT;
					TIM8->CCR1 =p1_LUT[enc_c]*duty+hfp_1;
					TIM8->CCR2 =p2_LUT[enc_c]*duty+hfp_2;
					TIM8->CCR3 =p3_LUT[enc_c]*duty+hfp_3;
					tim8_ccr[0]=TIM8->CCR1;
					tim8_ccr[1]=TIM8->CCR2;
					tim8_ccr[2]=TIM8->CCR3;
					/*TIM8->CCR1=tim8_ccr[0];
					TIM8->CCR2=tim8_ccr[1];
					TIM8->CCR3=tim8_ccr[2];  */
					enc_c_count = (uint16_t)enc_c*1500/1024;

				}


		  if (A_switch) {
		        if ((L_b_filtered_average >= prev_L_b+sicid_val)) {
		            B_switch = 1;
		            A_switch = 0;
		            C_switch = 0;


		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
		        }
		    }

		    // B_switch 상태에서 L_c의 연속 증가 조건 확인
		    if (B_switch) {
		        if ((L_c_filtered_average>= prev_L_c+sicid_val)) {
		            B_switch = 0;
		            A_switch = 0;
		            C_switch = 1;
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
		        }
		    }

		    // C_switch 상태에서 L_a의 연속 증가 조건 확인
		    if (C_switch) {
		        if ((L_a_filtered_average>= prev_L_a+sicid_val)) {
		            B_switch = 0;
		            A_switch = 1;
		            C_switch = 0;
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
		        }
		    }

		if(sicid_start){ //?   ? sicid

					if(adc_val[1]<current_offset[1]+count5) hfp_1 =duty1;
					else hfp_1 =0;
					if(adc_val[2]<current_offset[2]+count5) hfp_2 =duty1;
					else hfp_2 =0;
					if(adc_val[3]<current_offset[3]+count5) hfp_3 =duty1;
					else hfp_3 =0;

					lut_angle = (int)angle_a;

					  if (A_switch) {
						  if ((L_b_filtered_average >= prev_L_b+sicid_val)) {
					            B_switch = 1;
					            A_switch = 0;
					            C_switch = 0;
					            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
					        }
					    }

					    // B_switch 상태에서 L_c의 연속 증가 조건 확인
					    if (B_switch) {
					    	 if ((L_c_filtered_average>= prev_L_c+sicid_val)) {
					            B_switch = 0;
					            A_switch = 0;
					            C_switch = 1;
					            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
					        }
					    }

					    // C_switch 상태에서 L_a의 연속 증가 조건 확인
					    if (C_switch) {
					    	if ((L_a_filtered_average>= prev_L_a+sicid_val)) {
					            B_switch = 0;
					            A_switch = 1;
					            C_switch = 0;
					            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
					        }
					    }

					TIM8->CCR1 =A_switch*duty+hfp_1;
					TIM8->CCR2 =B_switch*duty+hfp_2;
					TIM8->CCR3 =C_switch*duty+hfp_3;

				}

		//example1=60*(example1/100) + 40*(adc_val[1]/100);
		//example3=60*(example3/100) + 40*(adc_val[2]/100);
		//example5=60*(example5/100) + 40*(adc_val[3]/100);

		example1 = adc_val[1];
		example3 = adc_val[2];
		example5 = adc_val[3];

		chai_a=example1-example2;
		chai_b=example3-example4;
		chai_c=example5-example6;

/*		if(example1>0){
				if(chai_a>8)
								{
									current_num_a1++;
									temp_chai_a1+=chai_a;
								}
				if(chai_a<-8)
								{
									current_num_a2++;
									temp_chai_a2+=chai_a;
								}

		}  */
		if(adc_val[1]>countttt)
			{
			if(chai_a>counttt)
							{
								current_num_a1++;
								temp_chai_a1+=chai_a;
							}
			if(chai_a<-counttt)
							{
								current_num_a2++;
								temp_chai_a2+=chai_a;
							}
			}
		else
		{
			if(chai_a>counttt&&chai_a<countt)
							{
								current_num_a1++;
								temp_chai_a1+=chai_a;
							}
			if(chai_a<-counttt&&chai_a>-countt)
							{
								current_num_a2++;
								temp_chai_a2+=chai_a;
							}
			}

		if(adc_val[2]>countttt){
				if(chai_b>counttt)
								{
									current_num_b1++;
									temp_chai_b1+=chai_b;
								}
				if(chai_b<-counttt)
								{
									current_num_b2++;
									temp_chai_b2+=chai_b;
								}

		}
		else{
			if(chai_b>counttt&&chai_b<countt)
							{
								current_num_b1++;
								temp_chai_b1+=chai_b;
							}
			if(chai_b<-counttt&&chai_b>-countt)
							{
								current_num_b2++;
								temp_chai_b2+=chai_b;
							}

					}
	if(adc_val[3]>countttt){
			if(chai_c>counttt)
							{
								current_num_c1++;
								temp_chai_c1+=chai_c;
							}
			if(chai_c<-counttt)
							{
								current_num_c2++;
								temp_chai_c2+=chai_c;
							}
		}
	else{
		if(chai_c>counttt&&chai_c<countt)
						{
							current_num_c1++;
							temp_chai_c1+=chai_c;
						}
		if(chai_c<-counttt&&chai_c>-countt)
						{
							current_num_c2++;
							temp_chai_c2+=chai_c;
						}
				}


		example2=example1;
		example4=example3;
		example6=example5;


		tim1_finish3=TIM6->CNT;
		tim1_count3=tim1_finish3-tim1_start3;
				}
	}

float LowPassFilter(float x,float x_prev)
{
	float L_a_filtered;
	// Low Pass Filter 계산
	L_a_filtered = 0.6 * x + 0.4 * x_prev;
	return L_a_filtered;
}


void InitLUT(void){

	for(unsigned int i=0;i<1024;i++){
		p1_LUT[i]= (i % 1024)>=angle_on && (i % 1024)<=angle_off;  //512~853
		p2_LUT[i]=((i-340)%1024)>=angle_on && ((i-340)%1024)<=angle_off;  //852~1023, 0~169
		p3_LUT[i]=((i+340)%1024)>=angle_on && ((i+340)%1024)<=angle_off; //172~513
	}
}
void SensorlessLUT(void)
{
	for(unsigned int i=200;i<1600;i++){
			p2_ssLUT[i]= (i%1600)>=1140 && (i % 1600)<=1590;

			p1_ssLUT[i]=(i%1600)>=670 && (i%1600)<=1130;

			p3_ssLUT[i]=(i%1600)>=200 && (i%1600)<=660;

		}
}

/* void SensorlessLUT(void)
{
	for(unsigned int i=1500;i<3000;i++){
			p1_ssLUT[i]= (i%3000)>=2020 && (i % 3000)<=2500;

			p2_ssLUT[i]=(i%3000)>=2500 && (i%3000)<=3000;

			p3_ssLUT[i]=(i%3000)>=1500 && (i%3000)<=2020;

		}
}
*/

/*double FastArcTan(double x){
	return 0.7853981633974483 * x - x * (fabs(x) - 1.0) * (0.2447 + 0.0663 * fabs(x));
} */
double FastArcTan(double x) {
	const double k1 = 0.28125;  // 미리 계산?   ?  ?
	return x / (1.0 + k1 * fabs(x));
}

void generate_L_sin(){
	float angle_increment = (2*3.1416)/7200;
	int max_value = 2650;
	int min_value = 600;
/*	for (int j=0; j< 400; j++)
	{
		L_sin[j] = 300;
	}  */
	for (int i=0; i<= 3600; i++)
	{
		float angle = i * angle_increment-3.1416/2;
		float sin_value = sin(angle);
		L_sin[i] = (max_value-min_value)/2*sin_value+(max_value+min_value)/2;
	}
}

float calculate_average(float *values, int count) {
	float sum = 0;
	for (int i = 1; i <= count; i++) {
		sum += values[i];
	}
	return sum / count;
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
