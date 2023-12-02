//speed, rpm, gear, modo(switch), teclado

/* USER CODE BEGIN Header */
/*
 A00834027 Roberto Ivan Estrada
 A01660619 Jeffry Emanuel Granados Johnson
 A00834003 Dafne Naomi Reyes Pimentel
 A00828736 Rodolfo Alejandro Hernandez Ibarra
 */
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

 #include <stddef.h>
 #include <stdio.h>
 #include "EngTrModel.h" /* Model's header file */
 #include "rtwtypes.h"


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
osThreadId ReadControlHandle;
osThreadId ReadStateHandle;
osThreadId SendLCDHandle;
//osThreadId SendDataHandle;


//speed, rpm, gear, modo(switch), teclado(pendiente)
uint32_t data[6] = {};

osMessageQId msgQueueHandle;
//osMessageQId speedQueueHandle;
//osMessageQId engineQueueHandle;
//osMessageQId gearQueueHandle;



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

void USER_LCD_Init(void);
void barrido(void);

void USER_ADC_Init(void);
void USER_ADC_Calibration(void);
uint16_t USER_ADC_Read( void );

void StartDefaultTask(void const * argument);
void readMatricial(void const * argument);
void readADC(void const * argument);
void readState(void const * argument);
void readControl(void const *argument);

void readAll(void const * argument);
void sendLCD(void const * argument);
//void sendData(void const * argument);


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
//
//  USER_USART1_Init();
//
//  USER_ADC_Init();
//  USER_ADC_Calibration();
//  ADC1->CR2	|=	 ADC_CR2_ADON;//	starts the conversion

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
  osMessageQDef(msgQueue, 4, uint32_t);
  msgQueueHandle = osMessageCreate(osMessageQ(msgQueue), NULL);

//  osMessageQDef(speedQueue, 4, uint32_t);
//  msgQueueHandle = osMessageCreate(osMessageQ(speedQueue), NULL);
//
//  osMessageQDef(engineQueue, 4, uint32_t);
//  msgQueueHandle = osMessageCreate(osMessageQ(engineQueue), NULL);
//
//  osMessageQDef(gearQueue, 4, uint32_t);
//  msgQueueHandle = osMessageCreate(osMessageQ(gearQueue), NULL);

  /* USER CODE END RTOS_QUEUES */
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(MatricialTask, readMatricial, osPriorityNormal, 0, 128*2);
  ReadMatricialHandle = osThreadCreate(osThread(MatricialTask), NULL);
//
  osThreadDef(ADCTask, readADC, osPriorityNormal, 0, 512);
  ReadADCHandle = osThreadCreate(osThread(ADCTask), NULL);
//
  osThreadDef(readStateTask, readState, osPriorityNormal, 0, 256);
  ReadStateHandle = osThreadCreate(osThread(readStateTask), NULL);

  osThreadDef(readControlTask, readControl, osPriorityNormal, 0, 256);
  ReadStateHandle = osThreadCreate(osThread(readControlTask), NULL);

  osThreadDef(LCDTask, sendLCD, osPriorityNormal, 0, 512);
  SendLCDHandle = osThreadCreate(osThread(LCDTask), NULL);

//  osThreadDef(sendDataTask, sendData, osPriorityNormal, 0, 256);
//  SendDataHandle = osThreadCreate(osThread(sendDataTask), NULL);




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
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;//		I/O port C clock enable
//  -------UART--------------
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 clock enable
//  --------ADC--------------
  RCC->APB2ENR |=	 RCC_APB2ENR_ADC1EN;//	ADC 1 clock enable
  RCC->CFGR |=	 RCC_CFGR_ADCPRE;  //	ADC prescaler 1:8 for 8 MHz

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

void USER_USART1_Init(void){
  USART1->CR1 |= USART_CR1_UE; // USART enabled
  USART1->CR1 &= ~USART_CR1_M; // 1 start bit, 8 data bits
  USART1->CR1 &= ~USART_CR1_PCE; // Parity control disabled
  USART1->CR2 &= ~USART_CR2_STOP; // 1 stop bit
  USART1->BRR = 0x22C; // 115200 bps 34.72
  USART1->CR1 |= USART_CR1_TE; // Transmitter enabled
  USART1->CR1 |= USART_CR1_RE;// receiver enabled
}

void USER_GPIO_Init_Matricial(void){
	//PA5 -> 0, LD2 OFF
	GPIOA->BSRR = GPIO_BSRR_BR5;
	GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5_0;

	//PINES MODOS as input pull-up -horizontal
	GPIOC->CRL &= ~GPIO_CRL_MODE1 & ~GPIO_CRL_CNF1_0;
	GPIOC->CRL |= GPIO_CRL_CNF1_1;
	GPIOC->ODR |= GPIO_ODR_ODR1;

	GPIOC->CRL &= ~GPIO_CRL_MODE2 & ~GPIO_CRL_CNF2_0;
	GPIOC->CRL |= GPIO_CRL_CNF2_1;
	GPIOC->ODR |= GPIO_ODR_ODR2;



	//PA12 as input pull-up -horizontal
	GPIOA->CRH &= ~GPIO_CRH_MODE12 & ~GPIO_CRH_CNF12_0;
	GPIOA->CRH |= GPIO_CRH_CNF12_1;
	GPIOA->ODR |= GPIO_ODR_ODR12;
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
	//PA2 (ADC12_IN2) as analog
//	GPIOA->CRL	&	~GPIOIO_CRL_CNF2 & ~GPIO_CRL_MODE2;
}

void USER_USART1_Transmit(uint8_t *pData, uint16_t size){
  for (int i = 0; i < size; i++){
    while ((USART1->SR & USART_SR_TXE) == 0){} // Wait until transmit register is empty
    USART1->DR = pData[i]; // Transmit data
  }
}


void USER_LCD_Init(void){
	LCD_Init( );//				inicializamos la libreria del LCD
	LCD_Cursor_ON( );//			cursor visible activo
	LCD_Clear( );//			borra la pantalla
}




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
  /* Infinite loop */
  for(;;)
  {
	printf("Vehicle Speed: %ld,", data[0]);
	printf("Engine Speed: %ld,", data[1]);
	printf("Gear: %ld,", data[2]);
	printf("Throttle: %ld,", data[3]);
	printf("Mode: %ld,", data[4]);
	printf("Orientation: %ld\n\r", data[5]);
	osDelay(500);
  }
  /* USER CODE END 5 */

}

/*
INPUT FUNCTIONS
 */

void readMatricial(void const * argument){
	/* USER CODE BEGIN 5 */
	uint32_t counter = 0;
	uint32_t temp;
	/* Infinite loop */
	USER_USART1_Init();

	printf("Tt\r\n");
	for(;;)
	{
	  barrido();
	  osDelay(50);
	}
	/* USER CODE END 5 */
}

void readADC(void const * argument){
	/* USER CODE BEGIN 5 */
	/*
	 El dispositivo deberá recibir una señal analógica
	 de un potenciómetro para el acelerador.
	 */

	uint32_t msg;
	float dataADC = 0;
	float converted = 0;

	USER_ADC_Init();
	USER_ADC_Calibration();
	ADC1->CR2	|=	 ADC_CR2_ADON;//	starts the conversion

	  /* Infinite loop */
	for(;;)
	{
	  dataADC = USER_ADC_Read();
	  converted = 100*(dataADC/((pow(2,12)-1)));
	  msg = (uint32_t)floor(converted);
//	  EngTrModel_U.Throttle = msg;	//Actualizamos la velocidad del acelerador
//	  EngTrModel_U.BrakeTorque = 0.0; //Paramos de frenar
//	  osMessagePut(msgQueueHandle, msg, 0);
	  data[3] = msg;
	  osDelay(5);
	}
	  /* USER CODE END 5 */
}

void readState(void const * argument){
	EngTrModel_initialize();
	  /* Infinite loop */

	for(;;)
	{
		EngTrModel_step();
		data[0] = EngTrModel_Y.VehicleSpeed;
		data[1] = EngTrModel_Y.EngineSpeed;
		data[2] = EngTrModel_Y.Gear;

//		printf("Vehicle Speed: %f\r\n", EngTrModel_Y.VehicleSpeed);
//		printf("Engine Speed: %f\r\n", EngTrModel_Y.EngineSpeed);
//		printf("Gear: %f\r\n", EngTrModel_Y.Gear);
		osDelay(40);
	}
}

void readControl(void const * argument){
	osEvent r_event;
	uint32_t value;
	for(;;){
//		r_event = osMessageGet(msgQueueHandle, 10);
//		if( r_event.status == osEventMessage )
//			value = r_event.value.v;

		value = data[3];

		if(!(GPIOC->IDR & GPIO_IDR_IDR13)){
			EngTrModel_U.Throttle = 2.0;
			EngTrModel_U.BrakeTorque = 100.0;
		}
		else{
			EngTrModel_U.Throttle = value;
//			printf("%d\n\r", value);
			EngTrModel_U.BrakeTorque = 0.0;
		}
		osDelay(10);
	}
}


/*	OUTPUT FUNCTIONS	*/

void sendLCD(void const * argument){
	/* USER CODE BEGIN 5 */
	/*
	 El dispositivo deberá mostrar las RPM actuales
	 del motor, la velocidad del vehículo, la marcha y
	 el acelerador en la pantalla principal de 16x2, , utilizando las
	 unidades correctas y con el espacio correcto.
	 Mostrará los valores actuales
	 * */

	uint32_t valorAnterior[6];
	uint32_t value[6], valorCambiado;
//	osEvent r_event;

	LCD_Init( );//				inicializamos la libreria del LCD
	LCD_Cursor_ON( );//			cursor visible activo
	LCD_Clear( );//			borra la pantalla
	LCD_Set_Cursor( 1,0);

	/* Infinite loop */
	for(;;)
	{
//		r_event = osMessagePeek(msgQueueHandle, 100);
//		if( r_event.status == osEventMessage )
//			value = r_event.value.v;
		for(int i = 0; i < 5; i++)
			value[i] = data[i];

		 valorCambiado = 0;
			  // Comprobar si el valor ha cambiado
		 for(int i = 0; i < 5; i++){
			 if (value[i] < (valorAnterior[i] - 3) || value[i] > (valorAnterior[i] + 3)) {
				  valorCambiado = 1;
				  break;
			 }
		 }
		if (valorCambiado) {
				  LCD_Clear();
				  LCD_Set_Cursor(1, 0);
				  LCD_Put_Str("VS:");
				  LCD_Put_Num(value[0]);
				  LCD_Put_Str(" ES:");
				  LCD_Put_Num(value[1]);
				  LCD_Set_Cursor(2, 0);
				  LCD_Put_Str("GR:");;
				  LCD_Put_Num(value[2]);
				  LCD_Put_Str(" TH:");
				  LCD_Put_Num(value[3]);
				  LCD_Put_Str(" MD:");
				  LCD_Put_Num(value[4]);

				  for(int i = 0; i < 5; i++)
					  valorAnterior[i] = value[i];
		}
//		temp = osKernelSysTick() - (100 * counter++);
		osDelay(200);
	}
	/* USER CODE END 5 */
}

//void sendData(void const * argument){
//	////speed, rpm, gear, modo(switch), teclado
//
//	/*
//	  El dispositivo deberá mostrar las RPM del motor, la velocidad del vehículo,
//	   la marcha y el acelerador en la pantalla secundaria en un formato gráfico
//	   con datos históricos.
//	   El gráfico mostrará variables vs tiempo con resolución de 500 ms.
//	   Con datos historicos.
//	 */
//
//	for(;;)
//	{
////		printf("RPM: ,");
////		printf("Vehicle Speed: ,");
////		printf("Gear: ,");
////		printf("Throttle: \n\r");
//		printf("SENDING DATA\n\r");
//		osDelay(500);
//	}
//
//}

/*	PROCESS FUNCTIONS	*/

void barrido(void){
	//PINES DE MODO

//	int switchState1 = HAL_GPIO_ReadPin(SWITCH_1_PORT, SWITCH_1_PIN);
//	int switchState2 = HAL_GPIO_ReadPin(SWITCH_2_PORT, SWITCH_2_PIN);

	int switchState1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	int switchState2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

	if (switchState1 == GPIO_PIN_RESET && switchState2 == GPIO_PIN_SET)
	{
		data[4] = 4;
	}
	else if(switchState1 == GPIO_PIN_RESET && switchState2 == GPIO_PIN_RESET)
	{
		data[4] = 3;
	}
	else if(switchState1 == GPIO_PIN_SET && switchState2 == GPIO_PIN_RESET)
	{
		data[4] = 2;
	}
	else if(switchState1 == GPIO_PIN_SET && switchState2 == GPIO_PIN_SET)
	{
		data[4] = 1;
	}
	else{
		data[4] = 0;
	}





	//ROWS
	//	  	 PA 12,6,11,7
	//	  First Column
	GPIOB->ODR &= ~GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR12)){ //1
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Turn Signal Left\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR12)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //4
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Left\n\r");
		data[5] = 4;
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //7
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		//printf("");
		while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //* delete
		GPIOA->ODR ^= GPIO_ODR_ODR5;
		//printf("");
		while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		HAL_Delay(10);
	}

	//ROWS
	//	  	 PA 12,6,11,7
	//Second Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR &= ~GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR12)){ //2
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Forward\n\r");
		data[5] = 1;
		while(!(GPIOA->IDR & GPIO_IDR_IDR12)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //5
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
//	  printf("Braking\n\r");
	  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
	  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //8
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
//	  printf("Backward\n\r");
	  data[5] = 3;
	  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
	  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //0
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  //printf("");
	  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
	  HAL_Delay(10);
	}

	//Third Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR &= ~GPIO_ODR_ODR5;
	GPIOB->ODR |= GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR12)){ //3
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Turn Signal Right\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR12)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //6
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Right\n\r");
		data[5] = 2;
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //9
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  //printf();
		  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		  HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //# space
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  //printf("");
		  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		  HAL_Delay(10);
	}

	//Fourth Column
	GPIOB->ODR |= GPIO_ODR_ODR3;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->ODR &= ~GPIO_ODR_ODR10;

	if(!(GPIOA->IDR & GPIO_IDR_IDR12)){ //A
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Drive Mode\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR12)){}
		HAL_Delay(10);
	  }
	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //B
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Neutral Mode\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //C
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("Reverse Mode\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
		HAL_Delay(10);
	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //D
		GPIOA->ODR ^= GPIO_ODR_ODR5;
//		printf("D1 Mode\n\r");
		while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		HAL_Delay(10);
	}
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
