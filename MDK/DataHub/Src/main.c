/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

uint16_t Modbus_Slave_ID = 0x01;
uint16_t Modbus_Hold_Reg[16];

#define UART1_RX_LEN 1024
uint8_t UART1_RX_BUF[UART1_RX_LEN];
__IO uint16_t UART1_RX_STA = 0;

void USART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)  //若空闲中断标记被置位
	{
	    __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // 清楚中断标记
	    HAL_UART_DMAStop(&huart1);           // 停止DMA接收
			UART1_RX_STA = UART1_RX_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);  // 总数据量减去未接收到的数据量为已经接收到的数据量
	    UART1_RX_BUF[UART1_RX_STA] = 0;  // 添加结束符
	    UART1_RX_STA |= 0X8000;         // 标记接收结束
	    HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, UART1_RX_LEN);  // 重启DMA接收
	}
}

//Modbus 03 读取保持寄存器16位
void Read_Reg_Hold(void)
{
	if(UART1_RX_BUF[2]==0x00 && UART1_RX_BUF[4]==0x00)
	{
		uint16_t start_reg = UART1_RX_BUF[3]; //起始寄存器地址
		uint16_t bit_num = UART1_RX_BUF[5]; //读取寄存器数量
		for(uint16_t i = start_reg; i<start_reg+bit_num; i++)
		{
			printf("i");
		}
	}
}

void Modbus_Handle(void)
{
	if(UART1_RX_BUF[0]!=Modbus_Slave_ID)
	{
		UART1_RX_STA = 0; //清空接收缓存
		return;
	}
	switch(UART1_RX_BUF[1])
	{
		case 0x01:{}break; //读取输出线圈状态
		case 0x02:{}break; //读取输入线圈状态
		case 0x03: //读取保持寄存器
		{
			Read_Reg_Hold();
		}break; 
		case 0x04:{}break; //读取输入寄存器
		case 0x05:{}break; //设置单个线圈
		case 0x06:{}break; //设置单个寄存器
		case 0x0f:{}break; //设置多个线圈
		case 0x10:{}break; //设置多个寄存器
			default:{}break; //错误
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


	//USART1 Modbus串口初始化
	HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, UART1_RX_LEN);  //启动DMA接收
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);              //使能空闲中断

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//若USART1 Modbus串口接收到数据
		if(UART1_RX_STA & 0X8000)
		{
			Modbus_Handle();
			HAL_UART_Transmit(&huart1, UART1_RX_BUF, UART1_RX_STA & 0X7FFF, 100);    // ???????????
			UART1_RX_STA = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
