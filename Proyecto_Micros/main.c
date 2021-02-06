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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "ESPDataLogger.h"

//------------DEFINE PARA EL ULTRASONIDOS----------------------------------------
#define TRIGGER_PORT GPIOE
#define TRIGGER_PIN GPIO_PIN_8

//------------DEFINE PARA LOS BOTONES----------------------------------------

#define BOTON_1_PORT GPIOA
#define BOTON_1_PIN GPIO_PIN_1

#define BOTON_2_PORT GPIOA
#define BOTON_2_PIN GPIO_PIN_2

#define BOTON_3_PORT GPIOA
#define BOTON_3_PIN GPIO_PIN_3

#define BOTON_4_PORT GPIOA
#define BOTON_4_PIN GPIO_PIN_4

#define BOTON_5_PORT GPIOC
#define BOTON_5_PIN GPIO_PIN_5


//------------DEFINE PARA LOS LUCES----------------------------------------
#define LUZ_INTERIOR_PORT GPIOD
#define LUZ_INTERIOR_PIN GPIO_PIN_1

#define LUZ_GARAJE_PORT GPIOD
#define LUZ_GARAJE_PIN GPIO_PIN_2

#define LUZ_ALARMA1_PORT GPIOD
#define LUZ_ALARMA1_PIN GPIO_PIN_3

#define LUZ_ALARMA2_PORT GPIOD
#define LUZ_ALARMA2_PIN GPIO_PIN_4

#define LUZ_ALARMA3_PORT GPIOD
#define LUZ_ALARMA3_PIN GPIO_PIN_5

#define LUZ_ALARMA4_PORT GPIOD
#define LUZ_ALARMA4_PIN GPIO_PIN_6

#define PARTY_MODE_PORT GPIOD
#define PARTY_MODE_PIN GPIO_PIN_12

#define AUTO_MODE_PORT GPIOD
#define AUTO_MODE_PIN GPIO_PIN_13

#define MANUAL_MODE_PORT GPIOD
#define MANUAL_MODE_PIN GPIO_PIN_14

#define ESP_WORK_PORT GPIOD
#define ESP_WORK_PIN GPIO_PIN_15


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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------VARIABLES-----------------------------------------------

//-------Variables para el sensor de ultrasonido HC-SR04

uint32_t IC_Val1 = 0;			//Tiempo inicial de la espera de la señal
uint32_t IC_Val2 = 0;			//Tiempo final de la espera de la señal
uint32_t Difference = 0; 		//Diferencia entre los dos tiempos anteriormente definidos
uint8_t Is_First_Captured = 0;          //Flag para detectar si es primera o segunda captura dentro de una medicion
uint8_t Distance  = 0;			//Distancia medida (Baja resolucion)
volatile float d = 0;			//Distancia medida con mas resolucion

//-----Variable auxiliar para el ultrasonidos
uint32_t ultrasound_counter;		//Me permetira saber si puedo leer o no
int car_presence = 0;	                // 0->No hay coche -------  1->Hay coche


//-------Variables para los botones--------------------------------------------------------
//------Boton de cambio de estado------
volatile int button_1 = 0;
uint32_t counter_1 = 0;
int button_counter_1 = 0;

//-----Boton de luces de dentro
volatile int button_2 = 0;
uint32_t counter_2 = 0;
int button_counter_2 = 0;

//-----Boton de luces de garaje
volatile int button_3 = 0;
uint32_t counter_3 = 0;
int button_counter_3 = 0;

//-----Boton de luces del fuera
volatile int button_4 = 0;
uint32_t counter_4 = 0;
int button_counter_4 = 0;
volatile int luz_on = 0;        	//0->OFF ----- 1->ON

//-----Boton del servomotor
volatile int button_5 = 0;
uint32_t counter_5 = 0;
int button_counter_5 = 0;
int puerta_abierta = 0;			//0->Puerta abierta --- 1->Puerta cerrada

int flag_button = 0;
uint32_t flag_counter = 0;
int flag_button_counter = 0;

//------Variable para el ESP---------------------------------------------------------------
uint16_t buffer_envio[3];
uint16_t counter_esp;

//------Variable para los pines del analogico digitales------------------------------------
volatile uint8_t adc1;		//Lectura del pin CHANNEL->8   LDR
uint8_t adc2;			//Lectura del pin CHANNEL->9   TEMPERATURA

//------Variable para la lectura del acelerometro------------------------------------------
uint8_t spiTxBuf[2];
uint8_t spiRxBuf[2];
volatile int8_t accel_x;
volatile int8_t accel_y;
volatile int8_t accel_z;
volatile uint32_t accel_counter = 0;
volatile int alarma_terremoto = 0;		//0-> no terremoto ----- 1->Terremoto

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------FUNCIONES-----------------------------------------------


//---------Funcion para la llamada para las interrupciones por botones---------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin == GPIO_PIN_1){
			button_1 = 1;
		}
		else if(GPIO_Pin == GPIO_PIN_2){
			button_2 = 1;
		}
		else if(GPIO_Pin == GPIO_PIN_3){
			button_3 = 1;
		}
		else if(GPIO_Pin == GPIO_PIN_4){
			button_4 = 1;
		}
		else if(GPIO_Pin == GPIO_PIN_5){
			button_5 = 1;
		}
}

//-------Funcion para el tratamiento del input capture------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			d=Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

//-------Funcion para crear las esperas de microsegundos----------------------------------
void delay (uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

//--------Funcion de llamada al sensor para que mida-----------------------------------
void HCSR04_Read (void){
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);    // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

//---------Funcion antirebotes---------------------------------------------------------
int antirebote(GPIO_TypeDef* GPIO,uint16_t GPIO_Pin,int boton,int contador_boton,uint32_t contador){
	if (boton==1){
		if (HAL_GetTick()-contador>=20){
			contador=HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO, GPIO_Pin)!=1){
				contador_boton=0;
			}
			else{
				contador_boton++;
			}
			if (contador_boton==3){        //Periodo antirebotes
				contador_boton=0;
				contador=HAL_GetTick();
				boton=0;

				flag_button_counter=contador_boton;
				flag_counter=contador;
				flag_button=boton;
				return 1;
			}
		}
	}
	flag_counter=contador;
	flag_button_counter=contador_boton;
	return 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //---------Inicializacion del ESP -------------------------------------------
  //ESP_Init("ErBixooo", "chocolate");
  //ESP_Init("HUAWEI-B311-9BB6", "4GY7AQBLT3BDR94Q");
  HAL_GPIO_WritePin(ESP_WORK_PORT, ESP_WORK_PIN,1);

  //---------Inicializacion del timer para el ultrasonidos---------------------
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  //---------Inicializacion del timer para el servomotor
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //---------Inicializacion del timer para las luces regulables
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //---------Inicializacion del timer para las alarmas sonoras
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //---------Inicializacion SPI------------------------------------------------
    spiRxBuf[0]=0x00;
    spiRxBuf[1]=0x00;

  //-----TRANSMITIR--------------------------------------------
  //--Chip Select Low
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
  //--Mandar datos
    spiTxBuf[0]=0x20;		//Direccion del registro que quiero
    spiTxBuf[1]=0x0F;		//El valor que quiero escribir m
    HAL_SPI_Transmit(&hspi1,spiTxBuf,2,HAL_MAX_DELAY);
  //--Chip Select High
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);

  //-----RECIBIR-----------------------------------------------
  //--Chip Select Low
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
  //--Mandar datos
    spiTxBuf[0]=0x20|0x80;		//Enable read mode. Debo mandarle un 1 al primer bit del registro anterior
    HAL_SPI_Transmit(&hspi1,spiTxBuf,1,HAL_MAX_DELAY);
  //--Recibir datos
    HAL_SPI_Receive(&hspi1,spiRxBuf,1,HAL_MAX_DELAY);
  //SS High
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);


  //-----Variable auxiliar para el ultrasonidos
    ultrasound_counter = HAL_GetTick();		//Me permetira saber si puedo leer o no

  //-----Variable para el ESP
    counter_esp = HAL_GetTick();

  //-----Variable para el switch case ---------------------------------------------------------
    volatile int estado = 0;	//Indica en que estado se esta:
				// 0->Modo Manual, 1->Modo Automatico, 2->Modo Terremoto



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //---Boton 1: Cambio de modos dentro del switch case
	  if(antirebote(BOTON_1_PORT,BOTON_1_PIN, button_1, button_counter_1, counter_1) == 1){
		  	 estado++;
		  	 if(estado == 2){
		  		 estado = 0;
		  	 }
		  	 button_1=flag_button;
	  }
	  button_counter_1 = flag_button_counter; counter_1 = flag_counter;

	  //---Boton 2: Control de la luz interior
	  if(antirebote(BOTON_2_PORT,BOTON_2_PIN, button_2, button_counter_2, counter_2) == 1){
		  	 HAL_GPIO_TogglePin(LUZ_INTERIOR_PORT, LUZ_INTERIOR_PIN);
			 button_2=flag_button;
	  }
	  button_counter_2 = flag_button_counter; counter_2 = flag_counter;

	  //---Boton 3: Control de la luz del garaje
	  if(antirebote(BOTON_3_PORT,BOTON_3_PIN, button_3, button_counter_3, counter_3) == 1){
		     HAL_GPIO_TogglePin(LUZ_GARAJE_PORT, LUZ_GARAJE_PIN);
			 button_3 = flag_button;
	  }
	  button_counter_3 = flag_button_counter; counter_3 = flag_counter;


	  switch (estado){
	  case 0:    //Modo Manual
		  HAL_GPIO_WritePin(MANUAL_MODE_PORT, MANUAL_MODE_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(AUTO_MODE_PORT, AUTO_MODE_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(PARTY_MODE_PORT, PARTY_MODE_PIN, GPIO_PIN_RESET);

		  //---Boton 4: Control de la luz exterior
		  if(antirebote(BOTON_4_PORT,BOTON_4_PIN, button_4, button_counter_4, counter_4) == 1){
			  if(luz_on == 0){
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,900);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,900);
				  luz_on = 1;
			  }
			  else if(luz_on == 1){
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
				  luz_on = 0;
			  }


			  button_4 = flag_button;
		  }
		  button_counter_4 = flag_button_counter; counter_4 = flag_counter;

		  break;
	  case 1:    //Modo Automatico
		  HAL_GPIO_WritePin(MANUAL_MODE_PORT, MANUAL_MODE_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(AUTO_MODE_PORT, AUTO_MODE_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(PARTY_MODE_PORT, PARTY_MODE_PIN, GPIO_PIN_RESET);

		  //-------Luces exteriores en funcion de la luz ambiente
		  if(adc1 < 75){
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
		  }
		  else if(adc1 < 150){
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,250);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,250);
		  }
		  else{
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1000);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1000);
		  }

		  break;
	  case 2:    //Modo Terremoto
		  HAL_GPIO_WritePin(MANUAL_MODE_PORT, MANUAL_MODE_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(AUTO_MODE_PORT, AUTO_MODE_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(PARTY_MODE_PORT, PARTY_MODE_PIN, GPIO_PIN_SET);

		  //----------Vuelta al estado por defecto
		  alarma_terremoto = 0;
		  estado = 0;

		  //----------Apagado de las luces-------------------------------------
		  HAL_GPIO_WritePin(LUZ_GARAJE_PORT, LUZ_GARAJE_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_INTERIOR_PORT, LUZ_INTERIOR_PIN, GPIO_PIN_RESET);


		  HAL_GPIO_WritePin(LUZ_ALARMA1_PORT,LUZ_ALARMA1_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_ALARMA2_PORT,LUZ_ALARMA2_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LUZ_ALARMA3_PORT,LUZ_ALARMA3_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_ALARMA4_PORT,LUZ_ALARMA4_PIN, GPIO_PIN_SET);

		  for(int i = 0; i < 15 ; i++){

			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,500);

			  HAL_Delay(200);

			  HAL_GPIO_TogglePin(LUZ_ALARMA1_PORT, LUZ_ALARMA1_PIN);
			  HAL_GPIO_TogglePin(LUZ_ALARMA2_PORT, LUZ_ALARMA2_PIN);
			  HAL_GPIO_TogglePin(LUZ_ALARMA3_PORT, LUZ_ALARMA3_PIN);
			  HAL_GPIO_TogglePin(LUZ_ALARMA4_PORT, LUZ_ALARMA4_PIN);

			  HAL_Delay(200);

			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,800);

		  }

		  HAL_GPIO_WritePin(LUZ_ALARMA1_PORT,LUZ_ALARMA1_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_ALARMA2_PORT,LUZ_ALARMA2_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_ALARMA3_PORT,LUZ_ALARMA3_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LUZ_ALARMA4_PORT,LUZ_ALARMA4_PIN, GPIO_PIN_RESET);



		  break;
	  default:
		  break;
	  }

	 /*__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
	  HAL_Delay(2000);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
	  HAL_Delay(2000);*/

	  //---------------------------LECTURA DE DATOS--------------------------
	  //----Lectura del ultrasonidos------
	  if((HAL_GetTick()-ultrasound_counter) > 250){
		  HCSR04_Read();
		  ultrasound_counter=HAL_GetTick();
	  }
	  if(d < 5){  car_presence = 1;}
	  else{	  	  car_presence = 0;}

	  //----Lectura de los pines de adc---
	  HAL_ADC_Start(&hadc1);

	  HAL_ADC_PollForConversion(&hadc1, 5);
	  adc1=HAL_ADC_GetValue(&hadc1);

	  HAL_ADC_PollForConversion(&hadc1, 5);
	  adc2=HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

	  //----Lectura del acelerometro

	  if(HAL_GetTick() - accel_counter > 250){
	  		//----EJE X-----
			//--Chip Select Low
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
			//--Transmitir datos
			  spiTxBuf[0]=0x29|0x80;		//Enable read mode
			  HAL_SPI_Transmit(&hspi1,spiTxBuf,1,HAL_MAX_DELAY);
			//--Recibir datos
			  HAL_SPI_Receive(&hspi1,&accel_x,1,HAL_MAX_DELAY);
			//--Chip Select High
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);

			//----EJE Y-----
			//--Chip Select Low
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
			//--Transmitir datos
			  spiTxBuf[0]=0x2B|0x80;		//Enable read mode
			  HAL_SPI_Transmit(&hspi1,spiTxBuf,1,HAL_MAX_DELAY);
			//--Recibir datos
			  HAL_SPI_Receive(&hspi1,&accel_y,1,HAL_MAX_DELAY);
			//--Chip Select High
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);

			//----EJE Z-----
			//--Chip Select Low
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
			//--Transmitir datos
			  spiTxBuf[0]=0x2D|0x80;		//Enable read mode
			  HAL_SPI_Transmit(&hspi1,spiTxBuf,1,HAL_MAX_DELAY);
			//--Recibir datos
			  HAL_SPI_Receive(&hspi1,&accel_z,1,HAL_MAX_DELAY);
			//--Chip Select High
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);

			//HAL_Delay(100);

			  accel_counter = HAL_GetTick();


			  if((accel_x < -10) || (accel_x > 10)){ alarma_terremoto = 1;}
			  else if((accel_y < -10) || (accel_y > 10)){	alarma_terremoto = 1;}
			  else if((accel_z < -10) || (accel_z > 10)){	alarma_terremoto = 1;}
			  if(alarma_terremoto == 1){	estado = 2;}
	  }

	  //----------------------------ENVIO DE DATOS----------------------
	  /*for(int i = 0; i < 10;i++){
		  buffer_envio[i]=i;
	  }

	  if(HAL_GetTick()-counter_esp < 15000){
		  ESP_Send_Multi("SF0QKRKC8ZEHKF8D", 10, buffer_envio);
		  buffer_envio[0]++;
	  }*/


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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI_S_R_Pin|TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PARTY_MODE_Pin|AUTOMATIC_MODE_Pin|MANUAL_MODE_Pin|ESP_WORKING_Pin
                          |LUZ_INTERIOR_Pin|LUZ_GARAJE_Pin|LUZ_ALARMA_1_Pin|LUZ_ALARMA_2_Pin
                          |LUZ_ALARMA_3_Pin|LUZ_ALARMA_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_S_R_Pin TRIGGER_Pin */
  GPIO_InitStruct.Pin = SPI_S_R_Pin|TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTON_1_Pin BOTON_2_Pin BOTON_3_Pin BOTON_4_Pin */
  GPIO_InitStruct.Pin = BUTON_1_Pin|BOTON_2_Pin|BOTON_3_Pin|BOTON_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTON_5_Pin */
  GPIO_InitStruct.Pin = BOTON_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOTON_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PARTY_MODE_Pin AUTOMATIC_MODE_Pin MANUAL_MODE_Pin ESP_WORKING_Pin
                           LUZ_INTERIOR_Pin LUZ_GARAJE_Pin LUZ_ALARMA_1_Pin LUZ_ALARMA_2_Pin
                           LUZ_ALARMA_3_Pin LUZ_ALARMA_4_Pin */
  GPIO_InitStruct.Pin = PARTY_MODE_Pin|AUTOMATIC_MODE_Pin|MANUAL_MODE_Pin|ESP_WORKING_Pin
                          |LUZ_INTERIOR_Pin|LUZ_GARAJE_Pin|LUZ_ALARMA_1_Pin|LUZ_ALARMA_2_Pin
                          |LUZ_ALARMA_3_Pin|LUZ_ALARMA_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
