/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
#include "lcd.h"
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
osThreadId defaultTaskHandle;
osThreadId ReadMatricialHandle;
osThreadId ReadADCHandle;
osThreadId PrintLCDHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_GPIO_Init_UART(void);
void USER_GPIO_Init_Matricial(void);
void USER_GPIO_Init_ADC(void);


void USER_USART1_Init(void);
void USER_USART1_Transmit(uint8_t *pData, uint16_t size);

void barrido(void);

void USER_ADC_Init(void);
void USER_ADC_Calibration(void);
uint16_t USER_ADC_Read( void );

void StartDefaultTask(void const * argument);
void readMatricial(void const * argument);
void readADC(void const * argument);
void printLCD(void const * argument);


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
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  USER_RCC_Init();
  USER_GPIO_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(readMatricial, ReadMatricialHandle, osPriorityHigh, 0, 256);
	ReadMatricialHandle = osThreadCreate(osThread(readMatricial), NULL);

	osThreadDef(readADC, ReadADCHandle, osPriorityNormal, 0, 256);
	ReadADCHandle = osThreadCreate(osThread(readADC), NULL);

	osThreadDef(printLCD, PrintLCDHandle, osPriorityAboveNormal, 0, 256);
	PrintLCDHandle = osThreadCreate(osThread(printLCD), NULL);
	/* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* GENERAL FUNCTIONS */
void USER_RCC_Init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // I/O port A clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//		I/O port B clock enable
//  -------UART--------------
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 clock enable
//  --------ADC--------------
  RCC->APB2ENR |=	 RCC_APB2ENR_ADC1EN;//	ADC 1 clock enable
  RCC->CFGR |=	 RCC_CFGR_ADCPRE;  //	ADC prescaler 1:8 for 8 MHz
  //Timer 2 and 3 clock enable
  RCC->APB1ENR	|=	 RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN; //TIM2 AND TIM3 ENABLE
}
void USER_GPIO_Init(void){
	USER_GPIO_Init_UART();
	USER_GPIO_Init_Matricial();
	USER_GPIO_Init_ADC();
}
void USER_GPIO_Init_UART(void){
	// Pin PA9 (USART1_TX) as alternate function output push-pull, max speed 10MHz
	GPIOA->CRH &= ~GPIO_CRH_CNF9;
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;

	// Pin PA10 (RX)
	GPIOA->CRH &= ~GPIO_CRH_CNF10;
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
}

void USER_GPIO_Init_Matricial(void){
	//PA5 -> 0, LD2 OFF
	GPIOA->BSRR = GPIO_BSRR_BR5;
	GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5_0;

	//PA10 as input pull-up -horizontal
	GPIOA->CRH &= ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10_0;
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->ODR |= GPIO_ODR_ODR10;
	//PA6 as input pull-up -horizontal
	GPIOA->CRL &= ~GPIO_CRL_MODE6 & ~GPIO_CRL_CNF6_0;
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->ODR |= GPIO_ODR_ODR6;
	//PA11 as input pull-up -horizontal
	GPIOA->CRH &= ~GPIO_CRH_MODE11 & ~GPIO_CRH_CNF11_0;
	GPIOA->CRH |= GPIO_CRH_CNF11_1;
	GPIOA->ODR |= GPIO_ODR_ODR11;
	//PA7 as input pull-up -horizontal
	GPIOA->CRL &= ~GPIO_CRL_MODE7 & ~GPIO_CRL_CNF7_0;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->ODR |= GPIO_ODR_ODR7;

	//PB3 as output push-pull -vertical
	GPIOB->BSRR = GPIO_BSRR_BS3;
	GPIOB->CRL &= ~GPIO_CRL_MODE3_1 & ~GPIO_CRL_CNF3;
	GPIOB->CRL |= GPIO_CRL_MODE3_0;
	//PB4 as output push-pull -vertical
	GPIOB->BSRR = GPIO_BSRR_BS4;
	GPIOB->CRL &= ~GPIO_CRL_MODE4_1 & ~GPIO_CRL_CNF4;
	GPIOB->CRL |= GPIO_CRL_MODE4_0;
	//PB5 as output push-pull -vertical
	GPIOB->BSRR = GPIO_BSRR_BS5;
	GPIOB->CRL &= ~GPIO_CRL_MODE5_1 & ~GPIO_CRL_CNF5;
	GPIOB->CRL |= GPIO_CRL_MODE5_0;
	//PB10 as output push-pull -vertical
	GPIOB->BSRR = GPIO_BSRR_BR10;
	GPIOB->CRH &= ~GPIO_CRH_MODE10_1 & ~GPIO_CRH_CNF10;
	GPIOB->CRH |= GPIO_CRH_MODE10_0;
}

void USER_GPIO_Init_ADC(){
	//PA0 (ADC12_IN0) as analog
	GPIOA->CRL	&=	~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0;
	//PA1 (TIM2_CH2) as alternate function push-pull, max speed 10MHz
	GPIOA->CRL	&=	~GPIO_CRL_CNF1_0 & ~GPIO_CRL_MODE1_1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1_0;
}

/* UART FUNCTIONS */

void USER_USART1_Init(void){
  USART1->CR1 |= USART_CR1_UE; // USART enabled
  USART1->CR1 &= ~USART_CR1_M; // 1 start bit, 8 data bits
  USART1->CR1 &= ~USART_CR1_PCE; // Parity control disabled
  USART1->CR2 &= ~USART_CR2_STOP; // 1 stop bit
  USART1->BRR = 0x22C; // 115200 bps 34.72
  USART1->CR1 |= USART_CR1_TE; // Transmitter enabled
  USART1->CR1 |= USART_CR1_RE;// receiver enabled
}

void USER_USART1_Transmit(uint8_t *pData, uint16_t size){
  for (int i = 0; i < size; i++){
    while ((USART1->SR & USART_SR_TXE) == 0){} // Wait until transmit register is empty
    USART1->DR = pData[i]; // Transmit data
  }
}

/* MATRICIAL FUNCTIONS */

void barrido(void){
	//ROWS
	//	  	 PA 10,6,11,7
	//	  First Column
	GPIOB->ODR &= ~GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //1
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("1");
		printf("Turn Signal Left\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //4
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("4");
		printf("Left\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //7
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("7");
		//printf("");
		while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //* delete
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Cursor_Left();
		LCD_Put_Str(" ");
		//printf("");
		LCD_Cursor_Left();
		while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		HAL_Delay(10);
	}

	//ROWS
	//	  	 PA 10,6,11,7
	//Second Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR &= ~GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //2
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("2");
		printf("Forward\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //5
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  LCD_Put_Str("5");
	  printf("Braking\n");
	  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
	  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //8
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  LCD_Put_Str("8");
	  printf("Backward\n");
	  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
	  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //0
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  LCD_Put_Str("0");
	  //printf("");
	  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
	  HAL_Delay(10);
	}

	//Third Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR &= ~GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //3
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("3");
		printf("Turn Signal Right\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //6
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("6");
		printf("Right\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //9
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  LCD_Put_Str("9");
		  //printf();
		  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //# space
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  LCD_Put_Str(" ");
		  //printf("");
		  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		  HAL_Delay(10);
	}

	//Fourth Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR &= ~GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //A
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("A");
		printf("Drive Mode\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		HAL_Delay(10);
	  }
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //B
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("B");
		printf("Neutral Mode\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //C
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("C");
		printf("Reverse Mode\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //D
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		LCD_Put_Str("D");
		printf("D1 Mode\n");
		while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		HAL_Delay(10);
	}
}

/* ADC FUNCTIONS */

void USER_ADC_Init(void){
	ADC1->CR1	&=	~ADC_CR1_DUALMOD;//	independent mode
	ADC1->CR2	&=	~ADC_CR2_ALIGN;//	right alignment for the result
	ADC1->CR2	|=	 ADC_CR2_CONT;//	continuous conversion mode
	ADC1->SMPR2	&=	~ADC_SMPR2_SMP0;//	1.5 cycles channel sample time
	ADC1->SQR1	&=	~ADC_SQR1_L;//		1 conversion on regular channels
	ADC1->SQR3 	&=	~ADC_SQR3_SQ1;//	first and only conversion in Ch0
	ADC1->CR2	|=	 ADC_CR2_ADON;//	ADC enabled
	HAL_Delay(1);//					tstab(1us) after ADC enabled, real 1ms
}
void USER_ADC_Calibration(void){
	ADC1->CR2	|=	 ADC_CR2_CAL;//		start calibration
	while( ADC1->CR2 & ADC_CR2_CAL );//		wait until calibration is done
}
uint16_t USER_ADC_Read( void ){
	while( !( ADC1->SR & ADC_SR_EOC ) );//		wait until conversion is done
	return (uint16_t)ADC1->DR;//			return ADC data
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

void readMatricial(void const * argument){
	/* USER CODE BEGIN 5 */
	  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(1);
	  }
	  /* USER CODE END 5 */
}
void readADC(void const * argument){
	/* USER CODE BEGIN 5 */
	  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(1);
	  }
	  /* USER CODE END 5 */
}
void printADC(void const * argument){
	/* USER CODE BEGIN 5 */
	  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(1);
	  }
	  /* USER CODE END 5 */
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
