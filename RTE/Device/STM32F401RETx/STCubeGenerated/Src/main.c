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
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum EventField {
    // Regular events
    TIMER_EVENT = 0x00000001,       // TIM2 interrupt or any other timer
    GPIO_PIN_CHANGE = 0x00000002,   // Change in GPIO pin state
    INTERRUPT_EVENT = 0x00000004,   // General interrupt event
    ENTER_CRITICAL_SECTION = 0x00000008,  // Enter critical section
    EXIT_CRITICAL_SECTION = 0x00000010,   // Exit critical section
    POWER_STATE_CHANGE = 0x00000020,  // Power state transition
    ERROR_EVENT = 0x00000040,         // Error or fault condition
    WATCHDOG_RESET = 0x00000080,      // Watchdog reset event
    SENSOR_READ_EVENT = 0x00000100,   // Sensor read event
    COMMUNICATION_EVENT = 0x00000200, // Communication event

    // Colors using the most significant bits (MSBs) in the 32-bit field
    WHITE = 0x00000000,  // 00 in the LSBs
    GRAY  = 0x40000000,  // 01 in the MSBs (6th bit)
    BLACK = 0x80000000   // 11 in the MSBs (7th bit)
};
// Define the Vertex structure
struct Vertex {
    int timestamp;  // Placeholder for the timestamp
    int event;      // Placeholder for event description or enum
    int gp_registers[8];  // General-purpose registers
    int hw_registers[4];  // Hardware-specific registers
};

// Function to initialize a Vertex (constructor equivalent in C)
struct Vertex init_vertex(int ts, int ev, const int gp_regs[8], const int hw_regs[4]) {
    struct Vertex v;
    v.timestamp = ts;
    v.event = ev;

    // Copy the passed values into the general-purpose registers
    for (int i = 0; i < 8; i++) {
        v.gp_registers[i] = gp_regs[i];
    }

    // Copy the passed values into the hardware-specific registers
    for (int i = 0; i < 4; i++) {
        v.hw_registers[i] = hw_regs[i];
    }

    return v;
}

// Set color in the event field
void set_color(struct Vertex* v, enum EventField color) {
    // Clear current color bits (MSB)
    v->event &= 0x3FFFFFFF;  // Keep the lower 30 bits, clear the top 2 bits

    // Set the new color
    v->event |= color;
}

// Get the current color from the event field
enum EventField get_color(const struct Vertex* v) {
    // Extract the top 2 bits
    uint32_t colorBits = v->event & 0xC0000000;
    return (enum EventField)colorBits;
}

// Set the event field, preserving the color bits
void set_event(struct Vertex* v, uint32_t newEvent) {
    // Mask out the color bits from the new event
    newEvent &= 0x3FFFFFFF;  // Ensure newEvent doesn't affect the color bits

    // Clear the existing event bits, preserve the color bits
    v->event &= 0xC0000000;  // Preserve the color bits (MSB)

    // Set the new event value (lower 30 bits)
    v->event |= newEvent;
}

// Get the event value, ignoring the color bits
uint32_t get_event(const struct Vertex* v) {
    // Mask out the color bits to return only the event-related bits
    return v->event & 0x3FFFFFFF;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_SIZE 128
volatile struct Vertex shared_array[ARRAY_SIZE];   // Shared array
volatile atomic_int write_index = 0;     // Atomic index to control access
extern volatile uint32_t system_tick; // Declare the system tick variable
volatile uint32_t system_tick = 0;  // Define the global system tick counter

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
void capture_gp_registers(int* gp_registers);
void add_vertex();
void init_systick();
void SysTick_Handler(void);
uint32_t get_timestamp();
int check_interrupt_flag(void);
 
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
	  add_vertex();
	  HAL_Delay(100);  // Adjust this delay as needed to balance execution
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void init_systick() {
    // Set up SysTick to generate a tick every 1 ms (assuming a system clock of 16 MHz)
    SysTick_Config(SystemCoreClock / 1000);
}
#if 0
// SysTick interrupt handler
void SysTick_Handler(void) {
    // Increment the system tick (this could be a global variable)
    // This will act as your millisecond counter
    extern volatile uint32_t system_tick;
    system_tick++;
}
#endif
uint32_t get_timestamp() {
    extern volatile uint32_t system_tick;
    return system_tick;
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
// Check if the TIM2 interrupt flag is set
int check_interrupt_flag(void) 
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

void add_vertex() {
    if (atomic_is_lock_free(&write_index)) {
        int local_index = atomic_load(&write_index);  // Load the index atomically

        if (local_index < ARRAY_SIZE) {
            // Get the current timestamp from SysTick
            int timestamp = (int)get_timestamp();

            // Set an example event based on system state (e.g., interrupt or pin change)
            int event = 0;  // Default event

            // Example: Check if an interrupt flag is set, update event accordingly
            if (check_interrupt_flag()) {
                event = INTERRUPT_EVENT;
            } else if (GPIOA->IDR & (1 << 0)) {  // Check if GPIO pin 0 is high
                event = GPIO_PIN_CHANGE;
            } else {
                event = TIMER_EVENT;  // Default to a timer event
            }

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

            // Initialize the vertex with real values
            shared_array[local_index] = init_vertex(timestamp, event, gp_registers, hw_registers);

            // Increment the index atomically
            atomic_fetch_add(&write_index, 1);
        }

        // Reset the index if it reaches the end (circular buffer)
        if (local_index >= ARRAY_SIZE - 1) {
            atomic_store(&write_index, 0);  // Reset to 0 atomically
        }
    }
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	// Interrupt writes 1 if within bounds
    	// Only write to one element at a time in the interrupt
			add_vertex();

    }
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
