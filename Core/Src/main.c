#include "main.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az;

float vectorPre;
float vector;
float vectorSum;
int steps = 0;
uint32_t Count = 0;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();

	SystemClock_Config();

	  MX_GPIO_Init();
	  MX_I2C1_Init();
	  MX_I2C2_Init();
	  MX_TIM2_Init();

	MPU6050_Init();
	lcd_init();
	lcd_clear();

	char data[4];
	uint8_t Rec_Data[6] = {0,0,0,0,0,0};
	
  while (1)
  {
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, 0, 0, 1000);
		HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		/*
			De chuyen doi gia tri RAW --> gia toc voi don vi g.
	  ==> phai chia theo gia tri Full scale: 16384
		*/
	  
		Ax = Accel_X_RAW/16384.0;
		Ay = Accel_Y_RAW/16384.0;
		Az = Accel_Z_RAW/16384.0;
		
		if(0u == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)) {
			lcd_clear();																						// Clear display LCD
			HAL_TIM_Base_Stop_IT(&htim2);															// Disable TIM2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
			steps = 0;																							// Reset counter step
			lcd_put_cur(0, 1);																			
			lcd_send_string ("Steps: ");																								 																													
			lcd_put_cur(1, 3);																												// Print string
			sprintf(data, "%d", steps);															// Convert Steps from int to char
			lcd_send_string(data);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
			if(0u == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)) {
				HAL_Delay(1);																						// Wait for some time to stabilize the system
				if(0u == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)) {															// Check pressed SS Pin Switch again
				while(0u == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3));													// Wait to Unpressed SS Pin Switch
				Count++;																						// Increase the counter by 1
				}
			}
			if(Count % 2 !=0) {
				HAL_TIM_Base_Start_IT(&htim2);
				vector = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az)); // Calculate the Space Vector length
				vectorSum = vector - vectorPre;								// Calculate the difference from the previous value
				
				if ((vectorSum*100) > 3){
					steps++;																						// Increase the Steps value by 1
				}
			}
			
			else {
				HAL_TIM_Base_Stop_IT(&htim2);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
			}
		}
		lcd_put_cur(0, 1);																			
		lcd_send_string ("TA - ND");																							 																													
		lcd_put_cur(1, 3);																			
		lcd_send_string("Steps= ");																
		sprintf(data, "%d", steps);															
		lcd_send_string(data);
    vectorPre = vector;
		HAL_Delay(800);
  }
  /* USER CODE END 3 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{


  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


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
