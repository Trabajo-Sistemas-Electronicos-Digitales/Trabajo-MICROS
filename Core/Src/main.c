/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
//#include "i2c-lcd.h"
//#include "dht11.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
//---------------------------------------------VARIABLES VENTILADOR
volatile int16_t ADC_val;
//--------------------------------------------VARIABLES GARAGE
#define RANGE  11.1   // 2000/180
uint32_t timegarage, waitgarage;
int anglegarage=90 , aperture=0 , sense=0;
float servoinput=0;
volatile int but3=0,but4=0;
volatile int but1=0,but2=0;
int state_light=0;
int mode_light=0;
volatile uint16_t ADC_vali=100;
//--------------------------------------------VARIABLES CAMARA_SEGURIDAD
volatile int flag;
int timer;
int angulo1;
int angulo2;
int t1=0;
int t2=0;
int tiempo_comienzo_interrupcion=0;
int tiempo_final_interrupcion=0;
int tiempo_total_interrupcion=0;
int n=0;
int bit_trabajo=1;
int desactivar_alarma=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------INTERRUPCIONES ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)  //FUNCIONAMIENTO POTENCIOMETRO
{
	if(hadc->Instance == ADC1)
	{
		ADC_val =HAL_ADC_GetValue(&hadc1);
	    //HAL_ADC_Stop_IT(&hadc1);
	}
	if (hadc->Instance == ADC2)
	{
		 ADC_vali = HAL_ADC_GetValue(&hadc2);
	}
}

void Ventilador()
{
	  HAL_ADC_Start_IT(&hadc1);
	  htim9.Instance->CCR2 = (ADC_val*100)/255;
}

//-------------------------------------------------------INTERRUPCIONES BOTONES
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_0)
	 {
		 but3=1;
	 }
	 if(GPIO_Pin == GPIO_PIN_1)
	 {
		 but4=1;
	 }
	 if(GPIO_Pin == GPIO_PIN_15)
	 {
	 	but1=1;
	 }
	 if(GPIO_Pin == GPIO_PIN_14)
	 {
	 	but2=1;
	 }
	 if(GPIO_Pin==GPIO_PIN_3)
	 {
		if   (flag == 1) {flag = 0;}
		else             {flag = 1;}
	 }
	 if(GPIO_Pin==GPIO_PIN_4)
	 {
		 desactivar_alarma = 1;
	 }
}

int debouncerbut3(volatile int* button_int, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_number)
{
	static uint8_t button_count = 0;
	static int counter = 0;

	if (*button_int == 1)
	{
		if (button_count == 0)
		{
			counter = HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick() - counter >= 20)
		{
			counter = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_number) != 1)
			{
				button_count = 1;
			}
			else
				button_count++;
			if (button_count == 4)
			{ //Periodo Antirrebotes
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}

int debouncerbut4(volatile int* button_int, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_number)
{
	static uint8_t button_count = 0;
	static int counter = 0;

	if (*button_int == 1)
	{
		if (button_count == 0)
		{
			counter = HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick() - counter >= 20)
		{
			counter = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_number) != 1)
			{
				button_count = 1;
			}
			else
				button_count++;
			if (button_count == 4)
			{ //Periodo Antirrebotes
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}

int debouncerbut6(volatile int* button_int, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_number)
{
	static uint8_t button_count = 0;
	static int counter = 0;

	if (*button_int == 1)
	{
		if (button_count == 0)
		{
			counter = HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick() - counter >= 20)
		{
			counter = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_number) != 1)
			{
				button_count = 1;
			}
			else
				button_count++;
			if (button_count == 4)
			{ //Periodo Antirrebotes
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}

int debouncerbut5(volatile int* button_int, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_number)
{
	static uint8_t button_count = 0;
	static int counter = 0;

	if (*button_int == 1)
	{
		if (button_count == 0)
		{
			counter = HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick() - counter >= 20)
		{
			counter = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_number) != 1)
			{
				button_count = 1;
			}
			else
				button_count++;
			if (button_count == 4)
			{ //Periodo Antirrebotes
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}

void lightOnOFF(int t)
{
	if(t==1)
	{
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
	}
	else
	{
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
	}

}

void LucesExteriores(void)  //FUNCIONAMIENTO LUCES EXTERIORES
{
	 if((debouncerbut6(&but2,GPIOE,GPIO_PIN_14))==1)
	 {
		  if(mode_light==1)
		  {
			  mode_light=0;
			  state_light=0;
	  	  }
		  else
		  {
	  		mode_light=1;
			  lightOnOFF(0); //Aquiiii
		  }
	 }
	 if ((debouncerbut5(&but1,GPIOE,GPIO_PIN_15))==1 && mode_light==0  )
	 {
		  if(state_light==1 && mode_light ==0)
		  { ///AQui
			  state_light=0;
	  	  }
		  else if(state_light==0 && mode_light==0)
		  { //Aqui
	  		state_light=1;
		  }
	  }
	  if(mode_light==1)
	  {
		  HAL_ADC_Start_IT(&hadc2);
		  if (ADC_vali < 20)
		  {
			  lightOnOFF(1);
		  }
		  else
		  {
			  lightOnOFF(0);
		  }
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1);
	  }
	  else
	  {
		  HAL_ADC_Stop_IT(&hadc2);
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
		  if(state_light==1)
			  lightOnOFF(1);
		  else
			  lightOnOFF(0);
	  }
}

void garagecontrol(void)  //FUNCIONAMIENTO GARAGE
{
	 if((debouncerbut3(&but3,GPIOA,GPIO_PIN_0))==1)
	 {
		  if(sense==1 && aperture==1)
		  {
			  sense=0;
	  	  }
		  else if (sense==0 && aperture==1)
		  {
	  		sense=1;
		  }
	 }
	 if ((debouncerbut4(&but4,GPIOA,GPIO_PIN_1))==1)
	 {
		  if(aperture==1)
		  {
			  aperture=0;
			waitgarage=0;//---
	  	  }
		  else
		  {
	  		aperture=1;
		  }
	  }
	  if(anglegarage==0 && sense==0)
	  {
		  if(waitgarage==0 && aperture==1 )//_-
			  waitgarage = HAL_GetTick();
		  else if(HAL_GetTick()-waitgarage >= 10000  &&  aperture==1)
		  {
			sense=1;
			waitgarage=0;
		  }
	  }
	  if((HAL_GetTick()-timegarage )>=20 && aperture==1)
	  {
		  if(anglegarage<90 && sense==1)
		  {
			  anglegarage++;
			  waitgarage=0;
			  timegarage=HAL_GetTick();
		  }
		  else if(anglegarage> 0 && sense==0)
		  {
			  anglegarage--;
			  waitgarage=0;
			  timegarage=HAL_GetTick();
		  }
	  }
	  servoinput = (RANGE*anglegarage)+500;
 	  htim2.Instance->CCR4 = servoinput;
 	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !aperture);
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 100;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  //FUNCION QUE MIDE DISTANCIAS ULTRASONIDOS
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read (void)  //FUNCION EMISION ONDA ULTRASONIDO
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_Delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_number)
{
  static uint8_t button_count = 0;
  static int counter = 0;
  	  if (*button_int == 1)
  	  {
  		  if (button_count == 0)
  		  {
  			  counter = HAL_GetTick();
  			  button_count++;
  		  }
  		  if (HAL_GetTick() - counter >= 20)
  		  {
  			  counter = HAL_GetTick();
  			  if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_number) != 1)
  			  {
  				  button_count = 1;
  			  }
  			  else
  				  button_count++;
  			  if (button_count == 4)
  			  { //Periodo Antirrebotes
  				  button_count = 0;
  				  *button_int = 0;
  				  return 1;
  			  }
  		  }
  	  }
return 0;
}

void CAMARA_SEGURIDAD(void)  //FUNCIONAMIENTO CAMARA SEGURIDAD
{
	//HCSR04_Read();

	if(n<=4000000)
	{
	  if( (HAL_GetTick() - tiempo_total_interrupcion) > n  && (HAL_GetTick() - tiempo_total_interrupcion) < (20000+n) && flag==0 && Distance >=20 )
	  {
		  angulo1=(((HAL_GetTick() - tiempo_total_interrupcion -n) / 10 ) + 500);
		  htim4.Instance->CCR1=angulo1; //0
		  timer=(HAL_GetTick() - tiempo_total_interrupcion);
		  HCSR04_Read();
	  }

	 if(flag==0 && Distance >=20  &&  (HAL_GetTick() - tiempo_total_interrupcion) >= (20000+n) && (HAL_GetTick() - tiempo_total_interrupcion) <= (25000+n) )
	 {
		 HCSR04_Read();
	 }

	 if ( (25000+n) < (HAL_GetTick() - tiempo_total_interrupcion)  && (HAL_GetTick() - tiempo_total_interrupcion) < (45000+n) && Distance >=20  && flag==0)
	 {
		  angulo2=( (  -(HAL_GetTick()) + tiempo_total_interrupcion + n + 50000  ) / 10 );
		  htim4.Instance->CCR1=angulo2;
		  timer=(HAL_GetTick() - tiempo_total_interrupcion);
		  HCSR04_Read();
	 }

	 if(flag==0 && Distance >=20  &&  (HAL_GetTick() - tiempo_total_interrupcion)  >= (45000+n)  &&  (HAL_GetTick() - tiempo_total_interrupcion) <= (50000+n))
	 {
		HCSR04_Read();
		if(flag==0 && Distance >=20  &&    (HAL_GetTick() - tiempo_total_interrupcion)  >= (49900+n)  &&  (HAL_GetTick() - tiempo_total_interrupcion) <= (50000+n))
		{	bit_trabajo=0; }
	 }

	 if (bit_trabajo == 0)
 	 {
 		  n=n+50000;
 		  bit_trabajo=1;
 		  angulo1=0;angulo2=0;
 	 }

	 if(desactivar_alarma==1)
	 {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 , 0);
		  desactivar_alarma=0;
	 }

	 if(Distance < 20 || flag==1 )  //INTERRUPCION POR OBSTACULO Y/O CAMARA QUIETA
	 {
	  	  tiempo_comienzo_interrupcion=HAL_GetTick();
	  	  HCSR04_Read();
	  	  if (Distance < 20 )
	  	  {
	  	    	HCSR04_Read();
	  	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 , 1);
	  	  }
	  	  tiempo_final_interrupcion= HAL_GetTick();
	  	  tiempo_total_interrupcion=  (tiempo_final_interrupcion - tiempo_comienzo_interrupcion) + tiempo_total_interrupcion;
	 }

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
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_PWM_Start (&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4);
  timegarage = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	LucesExteriores();
	Ventilador();
	garagecontrol();
	CAMARA_SEGURIDAD();

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
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 50;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 36-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 TRIG_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
