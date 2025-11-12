/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include "string.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAME_TIMEOUT 10000    // in ms, adjust to your baudrate
#define CMD_SHOW_TIME 5000

#define RX_BUFFER_SIZE 103
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t a;
char buf[25];
//char buffer[17];
//char buffer1[16];
//char buffer2[16];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
//char rx_buffer[17];
char msg[20];
uint32_t adc_value=0;
uint32_t size = 0;
uint8_t display_command = 0;
uint32_t command_display_tick = 0;

typedef enum {
	WAIT_START,
	WAIT_LENGTH,
	RECEIVE_DATA,
	WAIT_STOP
} RX_State;

volatile RX_State rx_state = WAIT_START;
volatile uint32_t last_rx_time;
volatile uint8_t rx_byte;
volatile uint8_t rx_index = 0;
volatile uint8_t expected_length = 0;
volatile uint8_t flagframecompleted = 0;
volatile uint32_t frame_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADC_Read()
{
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
	   adc_value = HAL_ADC_GetValue(&hadc1);
	   snprintf(msg, 16, "ADC:%lu", adc_value);
	   LCD_Set_Cursor(0, 0);
	   LCD_Send_String(msg);


	   // Send ADC value to PC
	   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	   HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
	}
	HAL_ADC_Stop(&hadc1);
}
void UART_Tx(char *test)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)test, strlen(test),HAL_MAX_DELAY);
}
void Led1_brightness_control(int val)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val);
}
void Led2_control_ON_OFF(char value)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	if(value==1)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 4095);
	}
	else if(value==0)
	{
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	}
}
void LCD_Display(char *msg)
{
	LCD_Clear();
	LCD_Set_Cursor(0, 0);
    LCD_Send_String(msg);
    LCD_Set_Cursor(1, 0);
    LCD_Send_String("     ");
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint32_t now = HAL_GetTick();

        if ((now - last_rx_time) > FRAME_TIMEOUT)
        {
            rx_state = WAIT_START;
            rx_index = 0;
            expected_length = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }

        last_rx_time = now;
        uint8_t b = rx_byte;
        switch (rx_state)
        {
            case WAIT_START:
                if (b == 0xAA)
                {
                    rx_buffer[0] = b;  // Store start byte
                    rx_state = WAIT_LENGTH;
                    rx_index = 1;
                }
                break;

            case WAIT_LENGTH:
                rx_buffer[rx_index++] = b;
                expected_length = b;
                if (expected_length > 0 && expected_length <=(RX_BUFFER_SIZE - 3))
                {
                    rx_state = RECEIVE_DATA;
                }
                else
                {
                    rx_state = WAIT_START;
                    rx_index = 0;
                }
                break;

            case RECEIVE_DATA:
                rx_buffer[rx_index++] = b;
                if (rx_index >= (expected_length+2))
                {
                    rx_state = WAIT_STOP;
                }
                break;

            case WAIT_STOP:
                if (b == 0x55)
                {
                    rx_buffer[rx_index++] = b;
                    flagframecompleted = 1;
                    frame_tick = HAL_GetTick();
                }
                rx_state = WAIT_START;
                rx_index = 0;
                expected_length = 0;
                break;
        }

		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && (huart->ErrorCode & HAL_UART_ERROR_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_FLUSH_DRREGISTER(huart);

        rx_state = WAIT_START;
        rx_index = 0;
        expected_length = 0;
        flagframecompleted = 0;

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
uint16_t decimal_value;

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  //HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  /* USER CODE END 2 */
  //uint32_t last_adc_tick;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	 uint32_t now = HAL_GetTick();
	 UART_Tx("Hello\r\n");

//	 uint8_t test_data[2] = {0x04, 0x05};
//	 uint16_t decimal_test_value = ((uint16_t)test_data[0] << 8) | test_data[1];
//	 snprintf(buf, sizeof(buf), "Decimal: %u", decimal_test_value);
//	 UART_Tx(buf);
//	 LCD_Set_Cursor(1, 0);
//	 LCD_Send_String(buf);

	 if (flagframecompleted==1)
	 {
//		 snprintf(buffer1, sizeof(buffer), "%02X %02X %02X %02X", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
//		 LCD_Set_Cursor(1, 0);
//		 LCD_Send_String(buffer1);
//		if(expected_length >= 4)
//		 {
		    decimal_value = (rx_buffer[2] << 8) | rx_buffer[3];

		    snprintf(buf, sizeof(buf), "%u", decimal_value);
		    UART_Tx(buf);

		    LCD_Clear();
		 	LCD_Set_Cursor(1, 0);
		 	LCD_Send_String(buf);
		 	HAL_Delay(3000);


		 	frame_tick = now;

		 	switch(rx_buffer[2])
		 	{
		 	    case 0x01:
		 	    	Led2_control_ON_OFF(1);
		 	    	LCD_Clear();
		 	    	LCD_Set_Cursor(1, 0);
		 	    	LCD_Send_String("LED ON");
		 	    	break;
		 	    case 0x02:
		 	    	Led2_control_ON_OFF(0);
		 	    	LCD_Clear();
		 	    	LCD_Set_Cursor(1, 0);
		 	    	LCD_Send_String("LED OFF");
		 	    	break;
		 	   case 0x00:
		 	        Led1_brightness_control(0);
		 	        LCD_Clear();
		 	        LCD_Set_Cursor(1, 0);
		 	        LCD_Send_String("Brightness: 0%");
		 	        break;

		 	   case 0x40:
		 	        Led1_brightness_control((0x40 * 4095) / 255);
		 	        LCD_Clear();
		 	        LCD_Set_Cursor(1, 0);
		 	        LCD_Send_String("Brightness: 25%");
		 	        break;

		 	    case 0x80:
		 	         Led1_brightness_control((0x80 * 4095) / 255);
		 	         LCD_Clear();
		 	         LCD_Set_Cursor(1, 0);
		 	         LCD_Send_String("Brightness: 50%");
		 	         break;

		 	    case 0xC0:
		 	         Led1_brightness_control((0xC0 * 4095) / 255);
		 	         LCD_Clear();
		 	         LCD_Set_Cursor(1, 0);
		 	         LCD_Send_String("Brightness: 75%");
		 	         break;

		 	    case 0xFF:
		 	         Led1_brightness_control(4095);
		 	         LCD_Clear();
		 	         LCD_Set_Cursor(1, 0);
		 	         LCD_Send_String("Brightness: 100%");
		 	         break;

		 	   default:
		 	          LCD_Clear();
		 	          LCD_Set_Cursor(1, 0);
		 	          //LCD_Send_String("Unknown Cmd");
		 	          break;
		 	}

		 	flagframecompleted = 0;
		 	frame_tick = now;
	    }
	 //flagframecompleted = 0;

	  ADC_Read();
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 8400-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4096-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
