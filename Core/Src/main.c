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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "st7735.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{

} ELRS_Packet_Typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t    rx_buffer[8];
uint32_t   rx_size;
uint8_t    rx_packet;
uint8_t    rx_types[256];

uint8_t  terminal_str[256];
uint8_t  terminal_cnt;
uint8_t  terminal_char;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void debug_printf(char *format, ...)
{
    va_list args;
    va_start(args, format);

    char text[128];
    char msg[168];

    memset(text,0,128);
    vsprintf(text, format, args);
    memset((char *)msg,0,168);
    sprintf(msg,"Debug > %s", text);

    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

    va_end(args);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crc8(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

#define TICKS_TO_US(x)  ((x - 992) * 5 / 8 + 1500)
#define US_TO_TICKS(x)  ((x - 1500) * 8 / 5 + 992)


crsfPayloadRcChannelsPacked_t rx_channels;
crsfLinkStatistics_t rx_statistics;

crsfFrameDef_t rx_frame;
volatile uint8_t rx_frame_status=0;

crsfFrameDef_t irq_frame;
volatile uint8_t irq_received=0;




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1)
	{
		if (huart == &huart1)
		{
			terminal_cnt++;
			HAL_UART_Transmit_IT(&huart1, &terminal_char, 1);
			HAL_UART_Receive_IT(&huart1, &terminal_char, 1);
		}
	}
	//================================================================
	if (huart->Instance == USART2)
	{
    	switch(irq_received)
    	{
    	case 0:
    		if(rx_buffer[0]== CRSF_SYNC_BYTE)
    		{
    			irq_frame.bytes[irq_received]=CRSF_SYNC_BYTE;
    			irq_frame.syncbyte=irq_frame.bytes[0];
    			irq_received++;
    		}
    		break;
    	case 1:
    		if(rx_buffer[0]<=CRSF_FRAME_SIZE_MAX)
    		{
    			irq_frame.bytes[irq_received]=rx_buffer[0];
    			irq_frame.length=irq_frame.bytes[1];
    			irq_received++;
    		}
    		break;
    	case 2:
    			irq_frame.bytes[irq_received]=rx_buffer[0];
    			irq_frame.type=irq_frame.bytes[2];
    			irq_received++;
    		break;
    	default:
    			irq_frame.bytes[irq_received]=rx_buffer[0];
    			irq_received++;
    			if(irq_received==(irq_frame.length+2))
    			{
    				irq_frame.crc8=rx_buffer[0];
    				memcpy(&rx_frame, &irq_frame, sizeof(crsfFrameDef_t));
    				rx_frame_status=1;
    				irq_received=0;

    			}
    	}
    	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}

void print_channels()
{
	debug_printf("CH00=%d \n\r", rx_channels.chan0 );
	debug_printf("CH01=%d \n\r", rx_channels.chan1 );
	debug_printf("CH02=%d \n\r", rx_channels.chan2 );
	debug_printf("CH03=%d \n\r", rx_channels.chan3 );
	debug_printf("CH04=%d \n\r", rx_channels.chan4 );
	debug_printf("CH05=%d \n\r", rx_channels.chan5 );
	debug_printf("CH06=%d \n\r", rx_channels.chan6 );
	debug_printf("CH07=%d \n\r", rx_channels.chan7 );
	debug_printf("CH08=%d \n\r", rx_channels.chan8 );
	debug_printf("CH09=%d \n\r", rx_channels.chan9 );
	debug_printf("CH10=%d \n\r", rx_channels.chan10 );
	debug_printf("CH11=%d \n\r", rx_channels.chan11 );
	debug_printf("CH12=%d \n\r", rx_channels.chan12 );
	debug_printf("CH13=%d \n\r", rx_channels.chan13 );
	debug_printf("CH14=%d \n\r", rx_channels.chan14 );
	debug_printf("CH15=%d \n\r", rx_channels.chan15 );
}

void print_frames()
{
	for(uint8_t f=0; f<255; f++)
	 if(rx_types[f]!=0)
		 debug_printf("FT=%X %d \n\r", f, rx_types[f] );

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();


  HAL_UART_Receive_IT(&huart1, &terminal_char, 1);
  HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
  rx_packet=1;
  int16_t rx_ch1;
  memset(rx_types,0,256);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ST7735_Init();

  // Check border
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_WriteString(0,  0, "X: ", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(0, 18, "Y: ", Font_11x18, ST7735_WHITE, ST7735_BLACK);

  ST7735_WriteString(0, 18*2+5, "Move to 0", Font_11x18, ST7735_GREEN, ST7735_BLACK);
  ST7735_WriteString(0, 18*3+5, "Select ...", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(0, 18*4+5, "Save ...", Font_11x18, ST7735_WHITE, ST7735_BLACK);

  ST7735_WriteString(0, 18*6+5, "Setup ...", Font_11x18, ST7735_WHITE, ST7735_BLACK);


  HAL_Delay(2000);

  uint32_t time_now = HAL_GetTick();
  uint32_t time_last= time_now;
  while (1)
  {
	  //===========================================================================
	  time_now = HAL_GetTick();
	  if( (time_now-time_last)> 500 )
	  {
		  char msg[32];
		  sprintf(msg,"X: %ld", time_last);
		  ST7735_WriteString(0,  0, msg, Font_11x18, ST7735_WHITE, ST7735_BLACK);

		  sprintf(msg,"Y: %ld", time_now);
		  ST7735_WriteString(0, 18, msg, Font_11x18, ST7735_WHITE, ST7735_BLACK);

		  time_last= HAL_GetTick();
	  }
	  //===========================================================================
	  if(terminal_cnt>0)
      {
    	  switch(terminal_char)
    	  {
    	  	  case 'w' : debug_printf("Move up\n\r"); break;
    	  	  case 's' : debug_printf("Move down\n\r"); break;
    	  	  case 'a' : debug_printf("Move left\n\r"); break;
    	  	  case 'd' : debug_printf("Move right\n\r"); break;
    	  	  case 'i' : debug_printf("Show info\n\r"); break;
    	  	  case 'z' : debug_printf("Move to zero\n\r"); break;
    	      case '0' : debug_printf("Store as zero\n\r"); break;
    	      case 'c' : print_channels(); break;
    	      case 'f' : print_frames(); break;
    	  }
    	  terminal_cnt--;
      }
      //===========================================================================
	 if(rx_frame_status==1)
	 {
		 rx_frame_status=0;
		 uint8_t frame_crc = crc8(&(rx_frame.bytes[2]),rx_frame.length-1);
		 if(frame_crc==rx_frame.crc8)
		 {
			 if(rx_frame.type==0x16)
			 {
				 memcpy((uint8_t *)&rx_channels, (uint8_t *)&(rx_frame.bytes[3]), 22);
				 rx_ch1=rx_channels.chan0;
				 rx_ch1=TICKS_TO_US(rx_ch1);
				// debug_printf("CH1=%d %d  \n\r", rx_channels.chan0, rx_ch1 );
			 }
			 else
			 {
				 rx_types[rx_frame.type]++;
			   //debug_printf("FT=%X - %d\n\r", rx_frame.type, rx_types[rx_frame.type] );
			 }
		 }
		 else
		 {
			 //debug_printf("Frame = %d CRC Error\n\r", rx_frame.type );
		 }

	 }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 420000;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_CS_Pin|LED_RESET_Pin|LED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_CS_Pin LED_RESET_Pin LED_DC_Pin */
  GPIO_InitStruct.Pin = LED_CS_Pin|LED_RESET_Pin|LED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
