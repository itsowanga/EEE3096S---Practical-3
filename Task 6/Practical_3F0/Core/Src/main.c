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
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Max_iter 100
#define CPU_freq 48000000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
volatile uint64_t CheckSum;
uint32_t StartTime;
static uint32_t EndTime;
uint32_t total_cycles;
volatile uint32_t ExecutionTime;
volatile float Throughput;
uint32_t image_Width[] = {128, 160, 192, 224, 256};
uint32_t image_Height[] = {128, 160, 192, 224, 256};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
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
	StartTime = HAL_GetTick();

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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //TODO: Visual indicator: Turn on LED0 to signal processing start
	  HAL_GPIO_WritePin(GPIOB,1, GPIO_PIN_SET);

	  //TODO: Benchmark and Profile Performance
	  uint16_t x = sizeof(image_Width)/sizeof(image_Width[0]);

	  //for (uint32_t i = 0; i< x ;i++){
		  volatile uint16_t width = 256;                                 //image_Width[i];
		  volatile uint16_t height = 256;                                //image_Height[i];
		  //StartTime = HAL_GetTick();
		  CheckSum = calculate_mandelbrot_fixed_point_arithmetic(width, height , Max_iter);
		  //EndTime = HAL_GetTick();
		  //ExecutionTime = EndTime - StartTime;
		  //total_cycles  = (uint64_t)ExecutionTime * (CPU_freq / 1000);
		  //uint32_t pixels = image_Width[i]* image_Height[i];
		  //Throughput = (float)pixels /((float)ExecutionTime/1000.0f);
		  //TODO: Visual indicator: Turn on LED1 to signal processing start
			HAL_GPIO_WritePin(GPIOB,2, GPIO_PIN_SET);

		  //TODO: Keep the LEDs ON for 2s
			HAL_Delay(2000);
		  //TODO: Turn OFF LEDs
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
			EndTime = HAL_GetTick();
			ExecutionTime = EndTime - StartTime;
	  //}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here
// Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
	#define S (1<<28)
    //TODO: Complete the function implementation
  	  for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {

        	  int64_t x_scaled = ((int64_t)x * S + width / 2) / width;
        	              int64_t y_scaled = ((int64_t)y * S + height / 2) / height;

        	              int64_t x0 = ((x_scaled * (int64_t)(3.5 * S) + (S >> 1)) >> 28) - (int64_t)(2.5 * S);
        	              int64_t y0 = ((y_scaled * (int64_t)(2.0 * S) + (S >> 1)) >> 28) - (1 * S);

        	              int64_t xi = 0;
        	              int64_t yi = 0;
        	              int iteration = 0;

        	              while (iteration < max_iterations &&
        	                     ((xi * xi + yi * yi) <= (4LL * S * S))) {


        	                  int64_t x_sq = (xi * xi + (S >> 1)) >> 28;
        	                  int64_t y_sq = (yi * yi + (S >> 1)) >> 28;
        	                  int64_t xy   = (xi * yi + (S >> 1)) >> 28;

        	                  int64_t temp_xi = x_sq - y_sq + x0;
        	                  yi = (xy << 1) + y0;
        	                  xi = temp_xi;

        	                  iteration++;
        	              }

        	              mandelbrot_sum += iteration;
        	          }
        	      }

        	      return mandelbrot_sum;

}

//Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;

    double inv_width  = 3.5 / width;
    double inv_height = 2.0 / height;

    for (int y = 0; y < height; y++) {
        double y0 = y * inv_height - 1.0;

        for (int x = 0; x < width; x++) {
            double x0 = x * inv_width - 2.5;

            double xi = 0.0, yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations && (xi*xi + yi*yi) <= 4.0) {
                double temp = xi*xi - yi*yi + x0;
                yi = 2.0 * xi * yi + y0;
                xi = temp;
                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }
    return mandelbrot_sum;
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
