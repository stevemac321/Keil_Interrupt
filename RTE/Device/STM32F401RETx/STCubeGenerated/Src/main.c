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
#include "stm32f4xx.h" // Include the appropriate CMSIS and device headers
#include <stdatomic.h>
#include <time.h>  // For timestamp conversion
#include <string.h>
#include <stdio.h>
#include "graph.h"
 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Check if the TIM2 interrupt flag is set
static inline int check_interrupt_flag(void) 
{
    // Check the update interrupt flag (UIF) in the TIM2 status register
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the interrupt flag by writing 0 to the UIF bit
        TIM2->SR &= ~TIM_SR_UIF;
        return 1;  // Interrupt occurred
    } else {
        return 0;  // No interrupt
    }
}

static inline uint32_t get_timestamp() {
    extern volatile uint32_t system_tick;
    return system_tick;
}
// Function to check if the TIMER_EVENT flag should be set
static inline int check_timer_event() {
    // Check the timer interrupt flag (UIF bit in the status register for TIM2)
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag (UIF)
        TIM2->SR &= ~TIM_SR_UIF;
        return TIMER_EVENT;
    }
    return 0;
}

// Function to check if the GPIO_PIN_CHANGE flag should be set
static inline int check_gpio_pin_change() {
    // Check GPIO pin state, for example
    if (GPIOA->IDR & (1 << 0)) {  // Check if GPIO pin 0 is high
        return GPIO_PIN_CHANGE;
    }
    return 0;
}

// Function to check if the INTERRUPT_EVENT flag should be set
static inline int check_interrupt_event() {
    if (check_interrupt_flag()) {
        return INTERRUPT_EVENT;
    }
    return 0;
}

// Function to check if the critical section events should be set
static inline int check_critical_section_event() {
    // Example: Check if a critical section has been entered using an arbitrary flag (application-specific)
    if (__get_PRIMASK()) { // Check if interrupts are disabled (used for critical section entry)
        return ENTER_CRITICAL_SECTION;
    } else if (!__get_PRIMASK()) { // Check if interrupts are enabled (used for critical section exit)
        return EXIT_CRITICAL_SECTION;
    }
    return 0;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
atomic_flag graph_lock = ATOMIC_FLAG_INIT;
volatile uint32_t system_tick = 0;  // Define the global system tick counter

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//system init
void SystemClock_Config(void);
void init_systick();
static void MX_TIM2_Init(void);


// event callbacks
void SysTick_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//state
int aggregate_event_flags();
void capture_gp_registers(int* gp_registers);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Graph graph;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	create_graph(&graph, ARRAYSZ);
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
  MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);  // Enable the TIM2 interrupt

  /* USER CODE BEGIN 2 */
	//HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
// Check if write_index is lock-free and within array bounds
	//	printf("hello from loop\n");
	  save_to_vertex();
	 // HAL_Delay(100);  // Adjust this delay as needed to balance execution
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// gather state, create and add new vertex
void save_to_vertex() {
    // Try to acquire the lock using atomic_flag
    if (!atomic_flag_test_and_set(&graph_lock)) {
        // If the graph is full or unavailable, inspect and free the graph
        if (graph.numVertices >= graph.graph_capacity) {
						inspect(&graph);
            atomic_flag_clear(&graph_lock);  // Release the lock
            return;
        }

        // Get the current timestamp from SysTick
        int timestamp = (int)get_timestamp();

        // Aggregate event flags from the system state
        int event = aggregate_event_flags();

        // General-purpose registers (R4-R7)
        int gp_registers[8];
        capture_gp_registers(gp_registers);  // Capture the GP registers

        // Hardware-specific registers (e.g., GPIOA registers)
        int hw_registers[4] = {
            (int)GPIOA->IDR,   // Input data register
            (int)GPIOA->ODR,   // Output data register
            (int)GPIOA->MODER, // Mode register
            (int)GPIOA->PUPDR  // Pull-up/pull-down register
        };

        // Set the color of the event (for example, let's assume the color is GRAY)
        setcolor(&event, GRAY);

        // Add the vertex to the graph with the aggregated event and register values
        size_t new_vertex_index = graph.numVertices; // Capture the index of the new vertex
        graph.add_vertex(&graph, timestamp, event, gp_registers, hw_registers);

        // Now check existing vertices and add edges based on common flags and timestamp window criteria
        for (size_t i = 0; i < new_vertex_index; i++) {
            // Check for matching event flags between the current and new vertex
            if ((graph.vertices[i].event & event) != 0) {
                // Check if the timestamp difference is within the allowed window
                if (abs(graph.vertices[i].timestamp - timestamp) <= TIMESTAMP_WINDOW) {
                    // Add an edge between the vertices (i.e., connect them)
                    graph.add_edge(&graph, i, new_vertex_index);
                }
            }
        }

        // Release the lock after writing to the graph
        atomic_flag_clear(&graph_lock);
    }
}

// event callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	// Interrupt writes 1 if within bounds
    	// Only write to one element at a time in the interrupt
			save_to_vertex();
    }
}
void init_systick() {
    // Set up SysTick to generate a tick every 1 ms (assuming a system clock of 16 MHz)
    SysTick_Config(SystemCoreClock / 1000);
}


// Function to capture general-purpose registers
void capture_gp_registers(int* gp_registers) 
	{
    // Inline assembly to read R4-R7 and store in the gp_registers array
    __asm volatile (
        "mov %[r4], r4\n\t"
        "mov %[r5], r5\n\t"
        "mov %[r6], r6\n\t"
        "mov %[r7], r7\n\t"
        : [r4] "=r" (gp_registers[0]),
          [r5] "=r" (gp_registers[1]),
          [r6] "=r" (gp_registers[2]),
          [r7] "=r" (gp_registers[3])
        :
        : "r4", "r5", "r6", "r7"
    );
}
	// Function to aggregate flags
int aggregate_event_flags() {
    int event_flags = 0;
    
    event_flags |= check_timer_event();
    event_flags |= check_gpio_pin_change();
    event_flags |= check_interrupt_event();
    event_flags |= check_critical_section_event();
    // Call other functions here as needed

    return event_flags;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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


