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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE         6
#define SAMPLES_PER_BUFFER  512
#define BUFFER_SIZE         (SAMPLE_SIZE * SAMPLES_PER_BUFFER)

#define ADXL_RANGE_2G   0x00
#define ADXL_RANGE_4G   0x01
#define ADXL_RANGE_8G   0x02
#define ADXL_RANGE_16G  0x03

#define ADXL_FULL_RES	(1 << 3)

#define ADXL_3200_HZ	0x0F
#define ADXL_1600_HZ	0x0E
#define ADXL_800_HZ		0x0D
#define ADXL_400_HZ 0x0C
// 3072 / 6 bytes per ADXL345 sample = 512 samples

typedef enum
{
    BUF_FREE = 0,
    BUF_FILLING,
    BUF_READY,
    BUF_TX
} BufferState;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint8_t 		start_stop = 0;
volatile uint32_t 		sample_count = 0; // testing fs
volatile uint32_t 		start_time = 0; // testing fs
volatile uint32_t 		stop_time = 0; // testing fs
volatile uint32_t dropped_buffer0 = 0;
volatile uint32_t dropped_buffer1 = 0;
volatile uint32_t tx0_complete = 0;
volatile uint32_t tx1_complete = 0;
uint8_t bw_rate_readback = 0;
volatile uint8_t exti_monitor;

uint8_t buffers[2][BUFFER_SIZE];

volatile BufferState 	buffer_state[2] = {BUF_FREE, BUF_FREE};

volatile uint8_t 		fill_buffer_id = 0;
volatile uint16_t 		fill_index = 0;

volatile uint8_t 		tx_buffer_id = 0;
volatile uint8_t 		tx_busy = 0;

volatile uint8_t 		acquiring = 0;
volatile uint8_t 		stopping = 0;

volatile uint8_t 		partial_buffer_id = 0;
volatile uint16_t 		partial_length = 0;
volatile uint8_t 		partial_pending = 0;

volatile uint32_t 		dropped_samples = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write (uint8_t Reg, uint8_t data)
{
	uint8_t writeBuf[2];
	writeBuf[0] = Reg | 0x40;	// multibyte write enabled
	writeBuf[1] = data;
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 	// pull the cs pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, writeBuf, 2, 100);		// transmit the address and data
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);		// pull the cs pin high to disable slave
}

void adxl_read (uint8_t Reg, uint8_t *Buffer, size_t len)
{
	Reg |= 0x80;	// read operation
	Reg |= 0x40;	// multibyte read
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 	// pull the cs pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, &Reg, 1, 100);		// send the address from where you want to read data
	HAL_SPI_Receive(&hspi1, Buffer, len, 100);	// read 6 bytes of data
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);		// pull the cs pin high to disable slave
}

void adxl_init(uint8_t range, uint8_t fs)
{
    uint8_t chipID = 0;
    adxl_read(0x00, &chipID, 1);

    if (chipID == 0xE5)
    {
        adxl_write(0x2D, 0x00);                 // standby
        adxl_write(0x2C, fs);
        adxl_write(0x31, ADXL_FULL_RES | range);

        adxl_write(0x2F, 0x00);                 // DATA_READY -> INT1
        adxl_write(0x2E, 0x00);                 // interrupt OFF for now

        adxl_write(0x2D, 0x08);                 // measurement mode
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
    	if (tx_buffer_id == 0)
    	            tx0_complete++;
    	        else
    	            tx1_complete++;

        buffer_state[tx_buffer_id] = BUF_FREE;
        tx_busy = 0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_5)
        return;

    exti_monitor ^= 1;

    uint8_t dummy[6];

    if (!acquiring)
    {
        // Clear DATA_READY if an interrupt slips through
        adxl_read(0x32, dummy, 6);
        return;
    }

    /*
     * A buffer may just have become free after UART transmission.
     */
    if (buffer_state[fill_buffer_id] == BUF_FREE)
    {
        buffer_state[fill_buffer_id] = BUF_FILLING;
    }

    /*
     * If the next buffer is still being transmitted / waiting for TX,
     * UART has fallen behind.
     */
    if (buffer_state[fill_buffer_id] != BUF_FILLING)
    {
        adxl_read(0x32, dummy, 6);   // still clear DATA_READY
        dropped_samples++;
        if (fill_buffer_id == 0)
                dropped_buffer0++;
            else
                dropped_buffer1++;
        return;
    }

    /*
     * THIS is the one and only normal ADXL data read.
     */
    adxl_read(
        0x32,
        &buffers[fill_buffer_id][fill_index],
        SAMPLE_SIZE
    );
    sample_count++;	// testing fs
    fill_index += SAMPLE_SIZE;

    if (fill_index >= BUFFER_SIZE)
    {
        buffer_state[fill_buffer_id] = BUF_READY;

        fill_buffer_id ^= 1;     // 0 -> 1, 1 -> 0
        fill_index = 0;

        if (buffer_state[fill_buffer_id] == BUF_FREE)
        {
            buffer_state[fill_buffer_id] = BUF_FILLING;
        }
//        else
//        {
//            dropped_samples++;
//        }
    }
}

void service_uart(void)
{
    if (tx_busy)
        return;

    /* First priority: transmit complete READY buffers */
    for (uint8_t i = 0; i < 2; i++)
    {
        if (buffer_state[i] == BUF_READY)
        {
            tx_buffer_id = i;
            tx_busy = 1;
            buffer_state[i] = BUF_TX;

            if (HAL_UART_Transmit_DMA(&huart2,
                                      buffers[i],
                                      BUFFER_SIZE) != HAL_OK)
            {
                buffer_state[i] = BUF_READY;
                tx_busy = 0;
            }

            return;
        }
    }

    /* Then transmit the final partial buffer */
    if (stopping && partial_pending)
    {
        tx_buffer_id = partial_buffer_id;
        tx_busy = 1;
        partial_pending = 0;

        buffer_state[tx_buffer_id] = BUF_TX;

        if (HAL_UART_Transmit_DMA(&huart2,
                                  buffers[tx_buffer_id],
                                  partial_length) != HAL_OK)
        {
            buffer_state[tx_buffer_id] = BUF_FILLING;
            partial_pending = 1;
            tx_busy = 0;
        }

        return;
    }
}

void service_stop(void)
{
    if (!stopping)
        return;

    if (tx_busy)
        return;

    if (buffer_state[0] == BUF_READY ||
        buffer_state[1] == BUF_READY)
        return;

    if (partial_pending)
        return;

    uint8_t stop_msg[] = {
        0xAA, 0x55, 0xAA, 0x55,
        0xAA, 0x55, 0xAA, 0x55
    };

    HAL_UART_Transmit(&huart2,
                      stop_msg,
                      sizeof(stop_msg),
                      HAL_MAX_DELAY);

    buffer_state[0] = BUF_FREE;
    buffer_state[1] = BUF_FREE;

    stopping = 0;
}

void start_program(void)
{
    uint8_t dummy[6];
    sample_count = 0;
    dropped_buffer0 = 0;
    dropped_buffer1 = 0;
    tx0_complete = 0;
    tx1_complete = 0;


    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

    // Clear stale DATA_READY
    adxl_read(0x32, dummy, 6);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);

    buffer_state[0] = BUF_FILLING;
    buffer_state[1] = BUF_FREE;

    fill_buffer_id = 0;
    fill_index = 0;

    tx_busy = 0;
    dropped_samples = 0;

    acquiring = 1;

    adxl_write(0x2E, 0x80);        // enable DATA_READY

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    start_time = HAL_GetTick();
}

void stop_program(void)
{
    uint8_t dummy[6];

    stop_time = HAL_GetTick();

    /* Stop NEW samples first */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    acquiring = 0;

    adxl_write(0x2E, 0x00);      // disable DATA_READY
    adxl_read(0x32, dummy, 6);   // clear DATA_READY
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);

    /* Remember valid part of current filling buffer */
    partial_buffer_id = fill_buffer_id;
    partial_length = fill_index;

    if (partial_length > 0)
        partial_pending = 1;
    else
        partial_pending = 0;

    /*
     * DO NOT:
     * HAL_UART_DMAStop()
     * reset buffer states
     * send STOP marker yet
     */

    stopping = 1;
}

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  adxl_init(ADXL_RANGE_4G, ADXL_1600_HZ);
  adxl_read(0x2C, &bw_rate_readback, 1);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	  {
	      start_stop ^= 1;

	      if (start_stop == 1)
	          start_program();
	      else
	          stop_program();

	      HAL_Delay(20);  // simple debounce

	      while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	      {
	          // wait until button is released
	      }

	      HAL_Delay(20);  // debounce release
	  }

	  if (acquiring || stopping)
	      service_uart();

	  if (stopping)
	      service_stop();

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 230400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
