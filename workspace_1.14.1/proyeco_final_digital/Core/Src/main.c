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
#include "stdio.h"
#include "lcd.h"
#include "keypadmodify.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Redefine los modos del sistema para mayor claridad
#define CLOCK_MODE 0       // Modo reloj
#define AUTO_MODE 1        // Modo automático
#define MANUAL_WATER_MODE 2  // Modo para riego manual
#define MENU_MODE 3        // Modo menú principal

#define BOMBA_PIN GPIO_PIN_13  // PB13 para la bomba
#define BOMBA_PORT GPIOB

#define MAX_RIEGO_TIEMPO 300000  // Tiempo máximo de riego en ms (5 minutos)

// Umbrales de humedad (ajustar según calibración del sensor)
#define HUMEDAD_SECO 3000     // Lectura ADC cuando el suelo está seco (mayor valor)
#define HUMEDAD_MOJADO 1000   // Lectura ADC cuando el suelo está mojado (menor valor)
#define UMBRAL_RIEGO 2500     // Umbral para activar el riego automático

// Reducir tiempo de debounce para mejor respuesta
#define KEY_DEBOUNCE_TIME 300  // 300ms para evitar rebotes pero permitir respuesta rápida
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Variables para el sistema de menú */
uint8_t currentMode = MENU_MODE;
uint8_t autoModeEnabled = 0;
uint8_t autoOnHour = 18;  // Hora de encendido por defecto (6 PM)
uint8_t autoOffHour = 6;  // Hora de apagado por defecto (6 AM)

/* Variables para el riego manual */
uint8_t bombaActiva = 0;
uint32_t tiempoInicioRiego = 0;
uint16_t tiempoRiegoRestante = 0;  // en segundos

/* Variables para estabilidad del menú */
uint32_t lastKeyPressTime = 0;

/* Variable para la lectura del ADC */
uint32_t medida_adc;
uint8_t porcentaje_humedad = 0;

/* Variable para controlar tiempo entre riegos automáticos */
uint32_t ultimoRiegoAutomatico = 0;
#define TIEMPO_ENTRE_RIEGOS 3600000  // 1 hora entre riegos automáticos (ms)
char timeData[15];
char dateData[15];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void displayMenu(void);
void handleClockMode(void);
void handleAutoMode(void);
void handleManualWaterMode(void);
void processKeyInput(char key);
void activarBomba(void);
void desactivarBomba(void);
void set_time(uint8_t hr, uint8_t min, uint8_t sec);
void set_date(uint8_t year, uint8_t month, uint8_t date, uint8_t day);
void get_time_date(char *time, char *date);
void set_alarm(uint8_t hr, uint8_t min, uint8_t sec, uint8_t date);
uint8_t calcularPorcentajeHumedad(uint32_t lecturaAdc);
void verificarHumedadYRegar(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t calcularPorcentajeHumedad(uint32_t lecturaAdc) {
    // Limitar lecturas a los rangos definidos
    if (lecturaAdc > HUMEDAD_SECO) {
        lecturaAdc = HUMEDAD_SECO;
    } else if (lecturaAdc < HUMEDAD_MOJADO) {
        lecturaAdc = HUMEDAD_MOJADO;
    }

    // Calcular porcentaje (invertido porque mayor ADC = menor humedad)
    // 0% = HUMEDAD_SECO, 100% = HUMEDAD_MOJADO
    uint8_t porcentaje = (uint8_t)(100 - ((lecturaAdc - HUMEDAD_MOJADO) * 100) / (HUMEDAD_SECO - HUMEDAD_MOJADO));

    return porcentaje;
}


/* Función para verificar la humedad y activar el riego si es necesario */
void verificarHumedadYRegar(void) {
    // Actualizar porcentaje de humedad
    porcentaje_humedad = calcularPorcentajeHumedad(medida_adc);

    // Verificar si se necesita regar basado en la humedad y tiempo transcurrido desde el último riego
    if (medida_adc > UMBRAL_RIEGO &&
        !bombaActiva &&
        (HAL_GetTick() - ultimoRiegoAutomatico > TIEMPO_ENTRE_RIEGOS)) {

        activarBomba();
        ultimoRiegoAutomatico = HAL_GetTick();
    }
}


void set_time(uint8_t hr, uint8_t min, uint8_t sec)
{
    RTC_TimeTypeDef sTime = {0};

    sTime.Hours = hr;
    sTime.Minutes = min;
    sTime.Seconds = sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

void set_date(uint8_t year, uint8_t month, uint8_t date, uint8_t day)  // monday = 1
{
    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = day;
    sDate.Month = month;
    sDate.Date = date;
    sDate.Year = year;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
}

void set_alarm(uint8_t hr, uint8_t min, uint8_t sec, uint8_t date)
{
    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.AlarmTime.Hours = hr;
    sAlarm.AlarmTime.Minutes = min;
    sAlarm.AlarmTime.Seconds = sec;
    sAlarm.AlarmTime.SubSeconds = 0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = date;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

void get_time_date(char *time, char *date)
{
    RTC_DateTypeDef gDate;
    RTC_TimeTypeDef gTime;

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

    /* Display time Format: hh:mm:ss */
    sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);

    /* Display date Format: dd-mm-yyyy */
    sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);  // turn the LED ON
}

/* Función para activar la bomba */
void activarBomba(void) {
    HAL_GPIO_WritePin(BOMBA_PORT, BOMBA_PIN, GPIO_PIN_SET);
    bombaActiva = 1;
    tiempoInicioRiego = HAL_GetTick();
    tiempoRiegoRestante = MAX_RIEGO_TIEMPO / 1000;  // Convertir a segundos
}

/* Función para desactivar la bomba */
void desactivarBomba(void) {
    HAL_GPIO_WritePin(BOMBA_PORT, BOMBA_PIN, GPIO_PIN_RESET);
    bombaActiva = 0;
}

/* Función para mostrar el menú principal actualizado */
void displayMenu(void) {
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("1.Reloj 2.Auto");
    lcd_put_cur(1, 0);
    lcd_send_string("3.Riego Manual");
}

/* Función para manejar el modo reloj */
void handleClockMode(void) {
    get_time_date(timeData, dateData);
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string(timeData);
    lcd_put_cur(1, 0);
    lcd_send_string(dateData);
    lcd_put_cur(1, 14);
    lcd_send_string("<*");
}

/* Función para manejar el modo automático */
void handleAutoMode(void) {
    lcd_clear();
    lcd_put_cur(0, 0);

    char statusMsg[16];
    // Actualizar porcentaje de humedad
    porcentaje_humedad = calcularPorcentajeHumedad(medida_adc);

    if (autoModeEnabled) {
        sprintf(statusMsg, "Auto: ON  Hum:%3d%%", porcentaje_humedad);

        // Verificar si es hora de riego y la humedad es baja
        RTC_TimeTypeDef currentTime;
        HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);

        if ((currentTime.Hours >= autoOnHour || currentTime.Hours < autoOffHour) &&
            porcentaje_humedad < (UMBRAL_RIEGO * 100 / HUMEDAD_SECO)) {

            // Verificar tiempo desde último riego
            if (!bombaActiva && (HAL_GetTick() - ultimoRiegoAutomatico > TIEMPO_ENTRE_RIEGOS)) {
                activarBomba();
                ultimoRiegoAutomatico = HAL_GetTick();
            }
        } else if (bombaActiva && (HAL_GetTick() - tiempoInicioRiego > MAX_RIEGO_TIEMPO)) {
            // Apagar bomba si excede tiempo máximo
            desactivarBomba();
        }
    } else {
        sprintf(statusMsg, "Auto: OFF Hum:%3d%%", porcentaje_humedad);
        // Asegurarse de que la bomba esté apagada cuando el modo automático está desactivado
        if (bombaActiva) {
            desactivarBomba();
        }
    }

    lcd_send_string(statusMsg);

    lcd_put_cur(1, 0);
    char timeMsg[16];
    if (bombaActiva) {
        uint16_t tiempoRestante = (MAX_RIEGO_TIEMPO - (HAL_GetTick() - tiempoInicioRiego)) / 1000;
        sprintf(timeMsg, "Riego: %03ds  <*", tiempoRestante);
    } else {
        sprintf(timeMsg, "#=ON/OFF A=Conf <*");
    }
    lcd_send_string(timeMsg);
}

void handleManualWaterMode(void) {
    lcd_clear();
    lcd_put_cur(0, 0);

    // Mostrar humedad actual
    porcentaje_humedad = calcularPorcentajeHumedad(medida_adc);
    char humedad[16];
    sprintf(humedad, "Humedad: %3d%%", porcentaje_humedad);
    lcd_send_string(humedad);

    lcd_put_cur(1, 0);
    if (bombaActiva) {
        // Calcular tiempo restante
        uint32_t tiempoTranscurrido = (HAL_GetTick() - tiempoInicioRiego) / 1000;  // en segundos
        if (tiempoTranscurrido < (MAX_RIEGO_TIEMPO / 1000)) {
            tiempoRiegoRestante = (MAX_RIEGO_TIEMPO / 1000) - tiempoTranscurrido;
        } else {
            // Si se ha excedido el tiempo máximo, desactivar la bomba
            desactivarBomba();
        }

        char tiempo[16];
        sprintf(tiempo, "Riego: %03ds   <*", tiempoRiegoRestante);
        lcd_send_string(tiempo);
    } else {
        lcd_send_string("# ON/OFF     <*");
    }
}

// Función mejorada para procesar las teclas presionadas
void processKeyInput(char key) {
    // Evitar cambios demasiado rápidos de pantalla
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastKeyPressTime < KEY_DEBOUNCE_TIME) {
        return;  // Ignorar pulsaciones muy rápidas
    }
    lastKeyPressTime = currentTime;

    // Para depuración - enviar la tecla presionada por UART si es necesario
    char debugMsg[20];
    sprintf(debugMsg, "Tecla: %c Modo: %d\r\n", key, currentMode);
    HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);

    switch (currentMode) {
        case MENU_MODE:
            if (key == '1') {
                currentMode = CLOCK_MODE;
                handleClockMode();
            } else if (key == '2') {
                currentMode = AUTO_MODE;
                handleAutoMode();
            } else if (key == '3') {
                currentMode = MANUAL_WATER_MODE;
                handleManualWaterMode();
            }
            break;

        case CLOCK_MODE:
            if (key == '*') {
                currentMode = MENU_MODE;
                displayMenu();
            }
            break;

        case AUTO_MODE:
            if (key == '*') {
                currentMode = MENU_MODE;
                displayMenu();
            } else if (key == '#') {
                // Alternar el estado del modo automático
                autoModeEnabled = !autoModeEnabled;
                if (!autoModeEnabled && bombaActiva) {
                    desactivarBomba();
                }
                handleAutoMode();  // Actualizar pantalla inmediatamente
            } else if (key == 'A') {
                // Incrementar la hora de encendido
                autoOnHour = (autoOnHour + 1) % 24;
                handleAutoMode();  // Actualizar pantalla inmediatamente
            } else if (key == 'B') {
                // Decrementar la hora de encendido
                autoOnHour = (autoOnHour + 23) % 24;
                handleAutoMode();  // Actualizar pantalla inmediatamente
            } else if (key == 'C') {
                // Incrementar la hora de apagado
                autoOffHour = (autoOffHour + 1) % 24;
                handleAutoMode();  // Actualizar pantalla inmediatamente
            } else if (key == 'D') {
                // Decrementar la hora de apagado
                autoOffHour = (autoOffHour + 23) % 24;
                handleAutoMode();  // Actualizar pantalla inmediatamente
            }
            break;

        case MANUAL_WATER_MODE:
            if (key == '*') {
                // Asegurarse de que la bomba esté apagada al salir
                desactivarBomba();
                currentMode = MENU_MODE;
                displayMenu();
            } else if (key == '#') {
                // Alternar el estado de la bomba
                if (bombaActiva) {
                    desactivarBomba();
                } else {
                    activarBomba();
                }
                handleManualWaterMode();  // Actualizar pantalla inmediatamente
            }
            break;
    }
}

uint32_t medida_adc;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
   HAL_ADC_Start_DMA(&hadc1, &medida_adc, 1);
   lcd_init();

   if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x2345)
   {
       // Establecer la hora actual de República Dominicana (UTC-4)
       set_time(22, 01, 00);
       set_date(25, 3, 23, 7); // 23 de marzo de 2025, domingo (7)
   }

   // Inicializar el pin de la bomba como salida y asegurarse de que esté apagada
   HAL_GPIO_WritePin(BOMBA_PORT, BOMBA_PIN, GPIO_PIN_RESET);

   // Iniciar en modo menú y mostrar el menú principal
   currentMode = MENU_MODE;
   displayMenu();

   // Enviar mensaje de inicio por UART para depuración
   char startMsg[] = "Sistema de riego iniciado\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)startMsg, strlen(startMsg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Obtener tecla presionada
    char key = Keypad_Get_Char();
    HAL_Delay(100);

    // Procesar la entrada del teclado si se presionó una tecla
    if (key != 0) {
        processKeyInput(key);
    }

    // Actualizar la pantalla periódicamente solo para modos que necesitan actualización constante
    static uint32_t lastRefresh = 0;
    if (HAL_GetTick() - lastRefresh > 1000) { // Actualizar cada segundo
        switch (currentMode) {
            case CLOCK_MODE:
                handleClockMode();
                break;
            case AUTO_MODE:
                if (autoModeEnabled || bombaActiva) {
                    handleAutoMode();
                }
                break;
            case MANUAL_WATER_MODE:
                if (bombaActiva) {
                    handleManualWaterMode();
                }
                break;
        }

        // Verificar si el modo automático está activado para controlar el riego
        if (autoModeEnabled && currentMode == AUTO_MODE) {
            verificarHumedadYRegar();
        }

        lastRefresh = HAL_GetTick();
    }

    // Verificar si se ha excedido el tiempo máximo de riego
    if (bombaActiva && (HAL_GetTick() - tiempoInicioRiego > MAX_RIEGO_TIEMPO)) {
        desactivarBomba();
        // Actualizar pantalla inmediatamente si estamos en una vista relevante
        if (currentMode == MANUAL_WATER_MODE || (currentMode == AUTO_MODE && autoModeEnabled)) {
            if (currentMode == MANUAL_WATER_MODE) {
                handleManualWaterMode();
            } else {
                handleAutoMode();
            }
        }
    }

    // Pequeña pausa para reducir el consumo de CPU
    HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 20;
  sTime.Minutes = 54;
  sTime.Seconds = 30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 23;
  sDate.Year = 25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, relay_Pin|R2_Pin|R4_Pin|R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : relay_Pin R2_Pin R4_Pin R3_Pin */
  GPIO_InitStruct.Pin = relay_Pin|R2_Pin|R4_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : C4_Pin */
  GPIO_InitStruct.Pin = C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C3_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

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
