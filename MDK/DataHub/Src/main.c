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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "oled.h"
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


#define UART3_RX_LEN 1024
uint8_t UART3_RX_BUF[UART3_RX_LEN];
__IO uint16_t UART3_RX_STA = 0;

uint16_t AT_command = 0;
uint16_t text8y = 0;

void OLED_CLR(){
	OLED_Clear();
	text8y = 0;
	OLED_Refresh();
}


void TrimSpace(char* str)
{
  char *start = str - 1;
    char *end = str;
    char *p = str;
    while(*p)
    {
        switch(*p)
        {
        case ' ':
					{
                if(start + 1==p)
                    start = p;
            }
            break;
        case '\r':
					{
                if(start + 1==p)
                    start = p;
            }
            break;
        case '\n':
            {
                if(start + 1==p)
                    start = p;
            }
            break;
        default:
            break;
        }
        ++p;
    }
    //??????????? ????
    --p;
    ++start;
    if(*start == 0)
    {
        //??????????
        *str = 0 ;
        return;
    }
    end = p + 1;
    while(p > start)
    {
        switch(*p)
        {
        case ' ':
					{
                if(end - 1 == p)
                    end = p;
            }break;
        case '\r':
					{
                if(end - 1 == p)
                    end = p;
            }break;
        case '\n':
            {
                if(end - 1 == p)
                    end = p;
            }
            break;
        default:
            break;
        }
        --p;
    }
    memmove(str,start,end-start);
    *(str + (int)end - (int)start) = 0;
}

void LED_Bleep(int ms, int times)
{
	for(int i = 0; i<times*2; i++)
	{			
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);//LED亮灭状�?�翻�?
		HAL_Delay(ms);//延时1000毫秒=1秒钟

	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void USART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE )!= RESET)
	{
		//printf("ORE\r\n");
		__HAL_UART_CLEAR_OREFLAG(&huart1);		
	}else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE )!= RESET){
		//printf("NE \r\n");
		__HAL_UART_CLEAR_NEFLAG(&huart1);
	}else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE )!= RESET){
		//printf("FE \r\n");
		__HAL_UART_CLEAR_FEFLAG(&huart1);
	}else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE )!= RESET){
		//printf("PE \r\n");
		__HAL_UART_CLEAR_PEFLAG(&huart1);
	}else{
		//printf("\r\n Normal, ");
	}
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)  //若空闲中断标记被置位
	{
	    __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // 清楚中断标记
	    HAL_UART_DMAStop(&huart1);           // 停止DMA接收
			UART1_RX_STA = UART1_RX_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);  // 总数据量减去未接收到的数据量为已经接收到的数据量
	    UART1_RX_BUF[UART1_RX_STA] = 0;  // 添加结束�?
	    UART1_RX_STA |= 0X8000;         // 标记接收结束
	    HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, UART1_RX_LEN);  // 重启DMA接收
	}
}

void USART3_IRQHandler(void)
{

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE )!= RESET)
	{
		//printf("ORE\r\n");
		__HAL_UART_CLEAR_OREFLAG(&huart3);		
	}else if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE )!= RESET){
		//printf("NE \r\n");
		__HAL_UART_CLEAR_NEFLAG(&huart3);
	}else if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE )!= RESET){
		//printf("FE \r\n");
		__HAL_UART_CLEAR_FEFLAG(&huart3);
	}else if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE )!= RESET){
		//printf("PE \r\n");
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}else{
		//printf("\r\n Normal, ");
	}
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)  //若空闲中断标记被置位
	{
		//printf("Interrupt reset\r\n");
	    __HAL_UART_CLEAR_IDLEFLAG(&huart3);  // 清楚中断标记
	    HAL_UART_DMAStop(&huart3);           // 停止DMA接收
			UART3_RX_STA = UART3_RX_LEN - __HAL_DMA_GET_COUNTER(huart3.hdmarx);  // 总数据量减去未接收到的数据量为已经接收到的数据量
		  //printf("PC: UART3_RX_STA: %u\r\n", UART3_RX_STA);
	    UART3_RX_BUF[UART3_RX_STA] = 0;  // 添加结束�?
	    UART3_RX_STA |= 0X8000;         // 标记接收结束
	    HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_LEN);  // 重启DMA接收
	}
}



//Modbus 03 读取保持寄存�?16�?
void Read_Reg_Hold(void)
{
	//printf("reg\r\n");
	if((UART1_RX_STA & 0X7FFF) > 0x06)
	{
		if(UART1_RX_BUF[2]==0x00 && UART1_RX_BUF[4]==0x00)
		{
			uint16_t start_reg = UART1_RX_BUF[3]; //起始寄存器地�?
			uint16_t bit_num = UART1_RX_BUF[5]; //读取寄存器数�?
			printf("PC: START_REG_NUM: %u, BIT_NUM %u \r\n",start_reg,bit_num);
		}
	}
}

void Modbus_Handle(void)
{
	//HAL_UART_Transmit(&huart1, UART1_RX_BUF, UART1_RX_STA & 0X7FFF, 100);    // ???????????
	printf("PC: MODBUS HEX, LENGTH: %u \r\n", UART1_RX_STA & 0X7FFF);

	if(UART1_RX_BUF[0]!=Modbus_Slave_ID)
	{		
		printf("PC: ID is not correct.\r\n");
	}
	else{
		printf("PC: ID is correct.\r\n");
		switch(UART1_RX_BUF[1])
		{
			case 0x01:{}break; //读取输出线圈状�??
			case 0x02:{}break; //读取输入线圈状�??
			case 0x03: //读取保持寄存�?
			{
				Read_Reg_Hold();
			}break; 
			case 0x04:{}break; //读取输入寄存�?
			case 0x05:{}break; //设置单个线圈
			case 0x06:{}break; //设置单个寄存�?
			case 0x0f:{}break; //设置多个线圈
			case 0x10:{}break; //设置多个寄存�?
				default:{}break; //错误
		}
	}
}

void AT_Command_Set(int step)
{

	switch(step)
	{
		case 1:{
			OLED_CLR();
			printf("\r\nPC: AT+CGSN=1------\r\n");
			AT_command = 1;
			uint8_t * net_test = (uint8_t*)"AT+CGSN=1\r\n";
			HAL_UART_Transmit(&huart3, net_test, 16, 100);    // ???????????       
			
		}break;
		case 2:{
			printf("\r\nPC: AT+CGATT?---------\r\n");
			AT_command = 2;	
	  	uint8_t * net_test = (uint8_t*)"AT+CGATT?\r\n";
			HAL_UART_Transmit(&huart3, net_test, 16, 100);    // ???????????       
						
		}break;
		case 3:{
			printf("\r\nPC: AT+CSQ---------\r\n");
			AT_command = 3;		
	  	uint8_t * net_test = (uint8_t*)"AT+CSQ\r\n";
			HAL_UART_Transmit(&huart3, net_test, 16, 100);    // ???????????    
				
		}break;
		case 4:{					
			printf("\r\nPC: AT+NMGS---------\r\n");
			AT_command = 4;	
			uint8_t * net_test = (uint8_t*)"AT+NMGS=17,0019992f4b4ccc66667fff7289b332ffff\r\n";
			HAL_UART_Transmit(&huart3, net_test, 64, 100);    // ???????????   
			
		}
		default:{}break;
	}

	//HAL_Delay(200);
	//uint8_t * net_quality = (uint8_t*)"AT+CSQ<CR>\r\n";
	//HAL_UART_Transmit(&huart3, net_quality, 16, 100);    // ???????????       
	//AT_command = 2;
	//LED_Bleep(50,4);
	
}

void AT_RX_Handle(void)
{
	TrimSpace((char*)UART3_RX_BUF);
	HAL_UART_Transmit(&huart1, UART3_RX_BUF, UART3_RX_STA & 0X7FFF, 100);    // ???????????
	UART3_RX_STA = 0;
	
	OLED_ShowString(0,text8y,UART3_RX_BUF,8,1);
	text8y+=8;
	OLED_Refresh();
	switch(AT_command)
	{
		case 1:{
			AT_Command_Set(2);
		}break;
		case 2:{
			AT_Command_Set(3);
		}break;
		case 3:{
			AT_Command_Set(4);
		}break;
			default:{}break;
	
	}
	OLED_Refresh();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		OLED_CLR();
		AT_Command_Set(1);	
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	OLED_Init();
	LED_ON;
	OLED_Refresh();
	
	HAL_TIM_Base_Start_IT(&htim2);


	//LED_Bleep(200,2);
	
	//USART1 Modbus串口初始�?
	HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, UART1_RX_LEN);  //启动DMA接收
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);              //使能空闲中断
	
	//__HAL_UART_DISABLE_IT(&huart3, UART_IT_PE);
	//HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_LEN);  //启动DMA接收
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);              //使能空闲中断
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//若USART1 Modbus串口接收到数�?
		if(UART1_RX_STA & 0X8000)
		{
			printf("PC:  Modbus handler \r\n");
			Modbus_Handle();
			UART1_RX_STA = 0;
		
		}
		
		//若UART3 Modbus串口接收到数�?
		if(UART3_RX_STA & 0X8000)
		{
			//printf("PC:  AT handler \r\n");
			AT_RX_Handle();
			
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==KEY1_Pin)
	{
		OLED_CLR();
		AT_Command_Set(1);
	}
	if(GPIO_Pin==KEY2_Pin)
	{
		OLED_CLR();
		AT_Command_Set(3);	
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
