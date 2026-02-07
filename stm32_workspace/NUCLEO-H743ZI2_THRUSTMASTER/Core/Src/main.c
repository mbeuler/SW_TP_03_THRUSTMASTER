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
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_hid.h"
#include "usbh_hid_throttle.h"

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

/* USER CODE BEGIN PV */
// Global buffer for processed throttle data
//volatile JoystickData_t g_joystickData;
volatile ThrottleData_t g_throttleData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void uint16_to_padded_str(uint16_t value, char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost) {
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;
    uint8_t *current = HID_Handle->pData;

    // Only process new reports (compare first 16 bytes)
    static uint8_t last_report[THROTTLE_REPORT_EFFECTIVE_SIZE];

    if (memcmp(current, last_report, THROTTLE_REPORT_EFFECTIVE_SIZE) != 0) {
        memcpy(last_report, current, THROTTLE_REPORT_EFFECTIVE_SIZE);

        ThrottleData_t data = USBH_HID_GetThrottleData(current);

        // Atomically update the global buffer
        __disable_irq();
        g_throttleData = data;
        __enable_irq();
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
  uint32_t lastPrint = 0;

  //JoystickData_t localCopyJoystick;
  ThrottleData_t localCopyThrottle;

  char xStr[6], yStr[6], msg[40];
  char zStr[6], sStr[6];
  int k;

  // Initialize global buffer
  g_throttleData.x = 0;
  g_throttleData.y = 0;
  g_throttleData.slider = 0;
  g_throttleData.z = 0;
  g_throttleData.rz = 0;

  for (int i = 0; i < 32; i++) {
	  g_throttleData.buttons[i] = 0;
  }
  g_throttleData.hat_switch = 8;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // Print every 250ms
    if (HAL_GetTick() - lastPrint >= 250) {
    	lastPrint = HAL_GetTick();

        // Copy throttle data atomically
        __disable_irq();
        //localCopyJoystick = g_joystickData;
        localCopyThrottle = g_throttleData;
        __enable_irq();

        uint16_to_padded_str(localCopyThrottle.z, zStr);
        uint16_to_padded_str(localCopyThrottle.x, xStr);
        uint16_to_padded_str(localCopyThrottle.y, yStr);
        uint16_to_padded_str(localCopyThrottle.slider, sStr);

        // Put message together: "Z:xxxxx X:yyyyy Y:yyyyy S:yyyyy\r\n"
        k = 0;
        msg[k++] = 'Z'; msg[k++] = ':';
        for (int i = 0; i < 5; i++) msg[k++] = zStr[i];
        msg[k++] = ' ';
        msg[k++] = 'X'; msg[k++] = ':';
        for (int i = 0; i < 5; i++) msg[k++] = xStr[i];
        msg[k++] = ' ';
        msg[k++] = 'Y'; msg[k++] = ':';
        for (int i = 0; i < 5; i++) msg[k++] = yStr[i];
        msg[k++] = ' ';
        msg[k++] = 'S'; msg[k++] = ':';
        for (int i = 0; i < 5; i++) msg[k++] = sStr[i];
        msg[k++] = '\r'; msg[k++] = '\n';
        msg[k] = '\0';

        HAL_UART_Transmit(&huart3, (uint8_t*)msg, k, 100);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void uint16_to_padded_str(uint16_t value, char *str) {
    // Temporary buffer for the numeric characters (max. 5 digits + null terminator)
    char temp[6];
    int i = 0;

    // Convert the number into the temp buffer (stored in reverse order)
    do {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0 && i < 5);

    // Pad with spaces if the number has fewer than 5 digits
    while (i < 5) {
        temp[i++] = ' ';
    }

    // Reverse the content of temp into the output string (fixed width: 5 chars)
    for (int j = 0; j < 5; j++) {
        str[j] = temp[4 - j];
    }

    // Null‑terminate the result
    str[5] = '\0';
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
