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
#include "bmp180_for_stm32_hal.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	IDLE,
	BOOST,
	BURNOUT,
	DROGUE_DESCENT,
	MAIN_DESCENT,
	LANDED
} rocket_status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


float CalculateAltitude(const float* temp, const uint32_t* press, const uint32_t* ground_press) //irtifa hesaplayan fonksiyon
{
	if(*press == 0) return 0; // Sıfıra bölmemek için koruma.

	float alt = (powf((*ground_press / (float)*press), 0.190263f) - 1.0f) * (*temp + 273.15f) / 0.0065f;
	return alt;
}

void DeployDrogueParachute()
{
	//İlk paraşütü açan fonksiyon
}

void DeployMainParachute()
{
	//İkinci paraşütü açan fonksiyon
}

void DetermineGroundPressure(uint32_t* ground_press, uint32_t counter) //Referans basıncını almak için fonksiyon
{
	uint32_t press = 0;
	uint32_t sum = 0;
	float temp = 0.0f; //temp değerini kullanmıyoruz ama UpdateSensorData() fonksiyonunu nullptr ile çalışacak şekilde güncellemeye üşendim.

	for(int i=0; i<counter; i++)
	{
		UpdateSensorData(&temp, &press);
		sum += press;
		HAL_Delay(15); //Sensör verisi 13.5 ms'de bir güncelleniyor ve bunun için bekliyoruz.
	}

	*ground_press = (uint32_t)sum / counter;
}

float GetDerivative(const float* current_value, const uint32_t* current_time_ms, const float* prev_value, const uint32_t* prev_time_ms) //İRtifadan hıza, hızdan ivmeye geçmek için türev hesaplama fonksiyonu
{
	float delta_time = (*current_time_ms - *prev_time_ms) / 1000.0f; //iki sensör verisinin alındığı zaman arasındaki fark
	float delta_value = *current_value - *prev_value; //iki sensör verisi arasındaki fark

	if (delta_time <= 0.0) return 0.0f; // Sıfıra bölmemek veya delta_time'dan kaynaklanan bir hatayla negatif hız değeri almamak için koruma..

	return delta_value/delta_time;
}

float LowPassFilter(const float* raw, const float* prev, float lpf_coef) //Filtre fonksiyonu
{
	return (lpf_coef*(*raw) + (1-lpf_coef)*(*prev));
}

void UpdateSensorData(float* temp, uint32_t* press) //sensör verilerine göre değişkenleri güncelleme fonksiyonu
{
	*temp = BMP180_GetTemperature();
	*press = BMP180_GetPressure();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


char msg[128];

float acceleration = 0.0f;
float altitude = 0.0f;
float raw_altitude = 0.0f;
float temperature = 0.0f;
float velocity = 0.0f;
float previous_altitude = 0.0f;
float previous_velocity = 0.0f;

uint32_t current_time_ms = 0;
uint32_t ground_pressure = 0;
uint32_t pressure = 0;
uint32_t previous_ms = 0;

uint16_t counter = 0;

uint8_t len = 0;

rocket_status current_status = IDLE;


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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BMP180_Init(&hi2c1);
  BMP180_SetOversampling(BMP180_HIGH);
  BMP180_UpdateCalibrationData();

  HAL_Delay(100); //Sensörün kalibre olması için bekliyoruz.
  DetermineGroundPressure(&ground_pressure, 20); //Referans basıncını hesaplamak için 20 ölçümün ortalamasını alıyoruz.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UpdateSensorData(&temperature, &pressure); //sensör verisi ile değişkenleri güncelliyoruz.
	  current_time_ms = HAL_GetTick(); //değişkenlerin güncellendiği anı alıyoruz. Direkt UpdateSensorData() fonksiyonunda belirlemeyi düşündüm ama fonksiyonun esnek kalmasını istedim.
	  raw_altitude = CalculateAltitude(&temperature, &pressure, &ground_pressure); //ham irtifa verisi hesaplama.
	  altitude = LowPassFilter(&raw_altitude, &previous_altitude, 0.2); //irtifa verisini filtreden geçiriyoruz.
	  velocity = GetDerivative(&altitude, &current_time_ms, &previous_altitude, &previous_ms); //irtifanın türeviyle hız hesaplama.
	  acceleration = GetDerivative(&velocity, &current_time_ms, &previous_velocity, &previous_ms); //hızın türeviyle ivme hesaplama.

	  switch (current_status) //mevcut aşamaya bağlı istenilen satıra atlar.
	  {
	  case IDLE: //roketin rampada olduğu an.
		  if(altitude > 10.0f && velocity > 1.0f) //10 metre yükselmiş ve hız 1 m/s olmuşsa sonraki aşamaya geç.
			  current_status = BOOST;
		  else break;

	  case BOOST:
		  if(acceleration < -9.6f) // İvme yerçekimi ivmesine eşitlenirse roketin yakıtı bitmiş demektir (0.21f hata payı)
		  {
			  counter += 1;
			  if (counter < 10) break;
			  else
			  {
				  counter = 0;
				  current_status = BURNOUT;
			  }
		  }

		  else
		  {
			  counter = 0;
			  break;
		  }

	  case BURNOUT:
		  if(velocity < 0.0f) // Hız aşağı yönlüyse roket düşüyordur. Sürüklenme paraşütünü aç.
		  {
			  counter += 1;
			  if (counter < 10) break;
			  else
			  {
				  counter = 0;
				  DeployDrogueParachute();
				  current_status = DROGUE_DESCENT;
			  }
		  }

		  else
		  {
			  counter = 0;
			  break;
		  }

	  case DROGUE_DESCENT:
		  if (altitude < MAIN_DEPLOY_ALTITUDE) // Roket ana paraşüt ateşlenmesi için belirlenen irtifanın altındaysa ana paraşütü aç.
		  {
			  DeployMainParachute();
			  current_status = MAIN_DESCENT;
		  } else break;

	  case MAIN_DESCENT:
		  if (!(velocity < -0.1f || velocity > 0.1f) && altitude < 10.0f) // Hız 0 ve irtifa 10'un altındaysa roket yere düşmüştür.
		  {
			  counter += 1;

			  if (counter < 10) break;
			  else
			  {
				  counter = 0;
				  current_status = LANDED;
			  }
		  }

		  break;

	  case LANDED:
		  //buzzer togglepin

	  default:

	  }

	  //Kullanılan değerlerin daha sonra türev almak için kaydedilmesi.
	  previous_altitude = altitude;
	  previous_ms = current_time_ms;
	  previous_velocity = velocity;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 16 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
