/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  Julio de la Rosa          2018-6732
  Practica 1          Calculadora
  Microcontroladores C2-2023
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c-lcd.h"
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include"max_matrix_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PORT GPIOA
#define R1_PIN GPIO_PIN_4

#define R2_PORT GPIOA
#define R2_PIN GPIO_PIN_5

#define R3_PORT GPIOA
#define R3_PIN GPIO_PIN_6

#define R4_PORT GPIOA
#define R4_PIN GPIO_PIN_7

#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_0

#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_1

#define C3_PORT GPIOB
#define C3_PIN GPIO_PIN_2

#define C4_PORT GPIOB
#define C4_PIN GPIO_PIN_10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*char num1[10],num2[10];
int resultado;
int co=0;
int numero=0, numero2=0;
int Counter=0;
char array[50];*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


char read_keypad (void)
{
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '1';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '2';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '3';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'A';
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '4';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '5';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '6';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'B';
	}



	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '7';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '8';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '9';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'C';
	}



	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
		return '*';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
		return '0';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
		return '#';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));
		return 'D';
	}

return '\0';
}


uint8_t key;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init ();
  max_init(0x03);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
	char operation = 0;

	lcd_send_cmd(0x01); // Limpiar la pantalla

	lcd_send_cmd(0x80); // Establecer cursor en la primera línea
	lcd_send_string("  A)suma  B)resta");

	lcd_send_cmd(0xC0); // Establecer cursor en la segunda línea
	lcd_send_string("C)multi D)div");

	while (operation == 0) {
	    key = read_keypad();

	    if (key != 0x01) {
	        if (key >= 'A' && key <= 'D') {
	            operation = key;
	        } else {
	            lcd_send_cmd(0x01); // Limpiar la pantalla
	            lcd_send_cmd(0x80);
	            lcd_send_string("  invalido");
	            lcd_send_cmd(0xC0);
	            lcd_send_string("Intente de nuevo");
	            write_char(88, 1);
	            HAL_Delay(2000);
	            max_clear();
	            lcd_send_cmd(0x01);
	            lcd_send_cmd(0x80); // Establecer cursor en la primera línea
	            lcd_send_string("  A)suma  B)resta");
	            lcd_send_cmd(0xC0); // Establecer cursor en la segunda línea
	            lcd_send_string("C)multi  D)div");
	        }
	    }
	}

	max_clear();

	lcd_send_cmd(0x01); // Limpiar la pantalla

	switch (operation) {
	    case 'A':
	        write_char(43, 1);
	        break;
	    case 'B':
	        write_char(45, 1);
	        break;
	    case 'C':
	        write_char(42, 1);
	        break;
	    case 'D':
	        write_char(47, 1);
	        break;
	}

	int num1 = 0;
	int num2 = 0;
	int digits1 = 0;
	int digits2 = 0;

	lcd_send_cmd(0x80);
	lcd_send_string("1ro num:");

	while (1) {
	    key = read_keypad();

	    if (key != 0x01) {
	        if (key >= '0' && key <= '9' && digits1 < 4) {
	            int digit = key - '0';
	            num1 = (num1 * 10) + digit;
	            lcd_send_data(key);
	            digits1++;
	        } else if (key == '#') {
	            break;
	        } else if (key == '*') {
	            num1 /= 10;
	            lcd_send_cmd(0x10); // Mover cursor hacia la izquierda
	            lcd_send_data(' ');
	            lcd_send_cmd(0x10);
	            digits1--;
	        }
	    }
	}

	lcd_send_cmd(0x01); // Limpiar la pantalla
	lcd_send_cmd(0x80); // Establecer cursor en la primera línea
	lcd_send_string("  2do num:");

	while (1) {
	    key = read_keypad();

	    if (key != 0x01) {
	        if (key >= '0' && key <= '9' && digits2 < 4) {
	            int digit = key - '0';
	            num2 = (num2 * 10) + digit;
	            lcd_send_data(key);
	            digits2++;
	        } else if (key == '#') {
	            break;
	        } else if (key == '*') {
	            num2 /= 10;
	            lcd_send_cmd(0x10); // Mover cursor hacia la izquierda
	            lcd_send_data(' ');
	            lcd_send_cmd(0x10);
	            digits2--;
	        }
	    }
	}

	int result = 0;

	switch (operation) {
	    case 'A':
	        write_char(43, 1);
	        result = num1 + num2;
	        break;
	    case 'B':
	        write_char(45, 1);
	        result = num1 - num2;
	        break;
	    case 'C':
	        write_char(42, 1);
	        result = num1 * num2;
	        break;
	    case 'D':
	            write_char(47, 1);
	            if (num2 != 0) {
	                result = num1 / num2;
	            } else {
	                lcd_send_cmd(0x01);
	                lcd_send_cmd(0x80);
	                lcd_send_string("  Division por 0");
	                lcd_send_cmd(0xC0);
	                lcd_send_string("Resultado: Inf");
	                while (1) {
	                    key = read_keypad();
	                    if (key == '#') {
	                        max_clear();
	                        break;
	                    }
	                }

	            }
	            break;
	    }

	lcd_send_cmd(0x01);
	lcd_send_cmd(0x80);
	if(num2!=0){
	lcd_send_string("  Resultado:");
	}
	if (result > 9999) {
	    lcd_send_cmd(0xC0);
	    lcd_send_string("Error Muy Largo");
	    write_char(1, 1);
	} else if (num2 != 0) {
	    lcd_send_cmd(0xC0);
	   if (result < 0) {
	      lcd_send_data('-');
	      result = -result;
	    }
	    lcd_send_data((result / 1000) + '0');
	    lcd_send_data(((result / 100) % 10) + '0');
	    lcd_send_data(((result / 10) % 10) + '0');
	    lcd_send_data((result % 10) + '0');
	}

	// hasta que se presione #
	while (1) {
	    key = read_keypad();

	    if (key == '#') {
	        max_clear();
	        break;
	    }
	}
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
