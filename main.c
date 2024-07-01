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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  //the MPU code below is copied from application note AN4838 page 13,
  //I only changed RAM region size for my MCU

  MPU_Region_InitTypeDef MPU_InitStruct;
  /* Disable MPU */
  HAL_MPU_Disable();
  /* Configure RAM region as Region N°0, 8kB of size and R/W region */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Configure FLASH region as REGION N°1, 1MB of size and R/W region */
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Configure FMC region as REGION N°2, 0.5GB of size, R/W region */
  MPU_InitStruct.BaseAddress = 0x40000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;

  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);



	volatile uint32_t timestamp_start = 0x00FFFFFF;
	volatile uint32_t timestamp_after_100nops = 0;
	volatile uint32_t timestamp_after_100movs = 0;
	volatile uint32_t duration_100nops = 0;
	volatile uint32_t duration_100movs = 0;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //disable ticker
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; //disable interrupt
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //source = CPU
	SysTick->LOAD = 0x00FFFFFF; //large enough reload value
	SysTick->VAL = 0x00FFFFFF; //it ticks down
	asm volatile ("cpsid f");
	//disable all handlers
	asm volatile ("dmb");
	asm volatile ("isb");
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable ticker
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");

	timestamp_after_100nops = SysTick->VAL;

	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");
	asm volatile ("MOV R0, R0");

	timestamp_after_100movs = SysTick->VAL;

	asm volatile ("cpsie f");
	//enable all handlers

	duration_100nops = timestamp_start - timestamp_after_100nops;
	duration_100movs = timestamp_after_100nops - timestamp_after_100movs;



	//Data cache test
	//D-Cache is actually for reading data from the flash, so there is no performance win here
	volatile uint32_t test_data_cache[1024] = { 0 }; //make it uint8_t for more reads
	volatile uint32_t timestamp_datacache_test_start = 0x00FFFFFF;
	volatile uint32_t timestamp_datacache_end = 0;
	volatile uint32_t duration_64_data_reads = 0;
	//FLASH->ACR &= ~FLASH_ACR_DCEN_Msk; //disabled data cache
	//FLASH->ACR |= FLASH_ACR_DCRST_Msk; //invalidate data cache
	//(volatile uint32_t) FLASH->ACR; //dummy read to waste a couple cycles
	//FLASH->ACR &= ~FLASH_ACR_DCRST_Msk; //release data cache invalidation
	//FLASH->ACR |= FLASH_ACR_DCEN_Msk; //re-enable data cache

	//starting the test
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //disable ticker
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; //disable interrupt
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //source = CPU
	SysTick->LOAD = 0x00FFFFFF; //large enough reload value
	SysTick->VAL = 0x00FFFFFF; //it ticks down
	asm volatile ("cpsid f");
	//disable all handlers
	asm volatile ("dmb");
	asm volatile ("isb");
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable ticker
	//64 reads
	//16 reads for 32-bit data reads, 64 reads for 8-bit data reads
	//check the disassembled instructions to estimate how many (and what exactly) instructions every line takes
  //3 instructions, 1 of them read, for byte access
  //2 instructions, 1 of them read, for word access
	test_data_cache[0];
	test_data_cache[1];
	test_data_cache[2];
	test_data_cache[3];
	test_data_cache[4];
	test_data_cache[5];
	test_data_cache[6];
	test_data_cache[7];
	test_data_cache[8];
	test_data_cache[9];
	test_data_cache[10];
	test_data_cache[11];
	test_data_cache[12];
	test_data_cache[13];
	test_data_cache[14];
	test_data_cache[15];
	test_data_cache[16];
//
//	test_data_cache[17];
//	test_data_cache[18];
//	test_data_cache[19];
//	test_data_cache[20];
//	test_data_cache[21];
//	test_data_cache[22];
//	test_data_cache[23];
//	test_data_cache[24];
//	test_data_cache[25];
//	test_data_cache[26];
//	test_data_cache[27];
//	test_data_cache[28];
//	test_data_cache[29];
//	test_data_cache[30];
//	test_data_cache[31];
//	test_data_cache[32];
//	test_data_cache[33];
//	test_data_cache[34];
//	test_data_cache[35];
//	test_data_cache[36];
//	test_data_cache[37];
//	test_data_cache[38];
//	test_data_cache[39];
//	test_data_cache[40];
//	test_data_cache[41];
//	test_data_cache[42];
//	test_data_cache[43];
//	test_data_cache[44];
//	test_data_cache[45];
//	test_data_cache[46];
//	test_data_cache[47];
//	test_data_cache[48];
//	test_data_cache[49];
//	test_data_cache[50];
//	test_data_cache[51];
//	test_data_cache[52];
//	test_data_cache[53];
//	test_data_cache[54];
//	test_data_cache[55];
//	test_data_cache[56];
//	test_data_cache[57];
//	test_data_cache[58];
//	test_data_cache[59];
//	test_data_cache[60];
//	test_data_cache[61];
//	test_data_cache[62];
//	test_data_cache[63];
	timestamp_datacache_end = SysTick->VAL;
	asm volatile ("cpsie f");
	duration_64_data_reads = timestamp_datacache_test_start - timestamp_datacache_end;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
