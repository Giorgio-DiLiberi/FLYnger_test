/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

//CODICE USATO PER ESTRARRE DATI DA UN SENSORE I2C (GIROSCOPIO)


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // per riconoscere le funzioni sulle stringhe
#include <stdio.h>
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

I2C_HandleTypeDef hi2c1;

USART_HandleTypeDef husart3;

/* USER CODE BEGIN PV */

//Indirizzo del sensore inerziale 9-axis BMX160
static const uint8_t BMX160Address=0x68<<1; //lo shift a destra serve per lasciare spazio al bit di READ/WRITE
static const uint8_t GyroDataRegister[6]={0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11}; //i sei registri di dati vanno da LSB a MSB da X a Z
static const uint8_t GyroConfRegister=0x42; //registro di configurazione del giroscopio
static const uint8_t AccConfRegister=0x40; //registro di configurazione dell' accelerometro
static const uint8_t CMD=0x7E; //Power modes register
//static const uint8_t GyroRangeRegister=0x43; //registro di configurazione del range del giroscopio

//Constants utilities
static const float k_toLPFprevSamp_Data=0.5333; //constant to LPF previous sample data
static const float k_toRaw_Data=0.2334; //constant to raw data for filter


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
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

	//Inizializzazione di una variabile ret che detenga il valore di ritorno delle funzioni di trasmissione e ricezione per controllo
	HAL_StatusTypeDef ret;

	//Variabil dal giroscopio
	uint8_t BMX_buff[12];//sono 12 byte: da LSB a MSB da X a Z SIA DI GYRO CHE DI ACC

	int16_t Gyr_Data[3]; //variabile con segno dove vengono salvati  dati completi a 16 bit del giroscopio

	float Omega[3];//3 componenti di velocità angolare con segno in virgola mobile
	float Omega_1[3]={0, 0, 0};// dato raw al sample precedente
	float OmegaLPF[3];//dato filtrato sample attuale
	float OmegaLPF_1[3]={0, 0, 0};//dato filtrato sample precedente
	float Gyr_cal[3]={0, 0, 0}; //Three constant evaluated in every start up to calibrate the gyro
	uint16_t i_gyro_cal; //initialize a counter i for cycles

    //Variabili dell'accelerometro
	int16_t Acc_Data[3]; //variabile con segno dove vengono salvati  dati completi a 16 bit del giroscopio

	float Acc[3];//3 componenti di velocità angolare con segno in virgola mobile |||| dato raw al sample attuale
	float Acc_1[3]={0, 0, 0};// dato raw al sample precedente
	float AccLPF[3];//dato filtrato sample attuale
	float AccLPF_1[3]={0, 0, 0};//dato filtrato sample precedente
	float Acc_cal[3]={0, 0, 0}; //Three constant evaluated in every start up to calibrate the gyro

	float BMX_Tx_LPF_buff[6]; //buffer to store Gyro and ACC values to send via usart
	//type casting for this variable will be executed in the Uart TX function only on the pointer, so to maitain the bit sequence iltere by the software


	uint16_t counter = 1; //Counter to toggle the led in functioning
	uint8_t i; //generic counter utility for loops in code


	uint8_t buff[32];



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
  MX_USART3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* LED EXPLANATION
   * # GREEN LED SOLID: initialization and calibration status;
   * # BLUE LED BLINKIING (1 Hz): normal runtime of control loop.
   */


  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //turn on green LED for the initialization

  HAL_Delay(10);


  /*######### BMX160 CONFIGURATION##########*/

    //codice per inizializzare e configurare il giroscopio
	buff[0]=GyroConfRegister;//invio l'indirizzo del configuratore del giroscopio
	buff[1]=0b00011100;//set the Gyro conf register (see the red notebook page 10)
	HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 2, HAL_MAX_DELAY);

	//codice per inizializzare e configurare l'accelerometro
	buff[0]=AccConfRegister;//invio l'indirizzo del configuratore dell' accelerometro
	buff[1]=0b00101100;//set the Acc conf register
	ret = HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 2, HAL_MAX_DELAY);

	//codice per configurare il range dell'accelerometro
	buff[0]=0x41;//invio l'indirizzo del configuratore del range
	buff[1]=0b00001000;//set the Acc range register (see the red notebook appropriate page)
	ret = HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 2, HAL_MAX_DELAY);


    HAL_Delay(800);

	//change the power mode to normal for the gyro
	buff[0]=CMD;
    buff[1]=0x15;//command for normal mode for gyro;
	HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 2, HAL_MAX_DELAY);

	//change the power mode to normal for the Acc see appropriate page of the notebook
	buff[0]=CMD;
	buff[1]=0x11;//command for normal mode for accelerometer;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 2, HAL_MAX_DELAY);


	/*###########us timer enabling (counter)#############*/

	LL_TIM_EnableCounter(TIM1); //func to enable the counter based on TIM1 with microseconds tick
	LL_TIM_SetCounter(TIM1,0);  //func to set set counter to 0value


	HAL_Delay(1000);

	/* ###### BMX CALIBRATION #######
	 *
	 * Calibration is performed polling data for 1000 times
	 *  with 1ms sample time: then an average value is
	 *  calculated and subtracted to the non filtered measures
	 *  taken during functioning
	 *
	 */

	for (i_gyro_cal=0; i_gyro_cal<1000; i_gyro_cal++) {

		LL_TIM_SetCounter(TIM1,0);

		buff[0]=GyroDataRegister[0];
		  HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 1, HAL_MAX_DELAY); //transmitting 1 byte with the register of the LSBx
		  HAL_I2C_Master_Receive(&hi2c1, BMX160Address, BMX_buff, 12, HAL_MAX_DELAY);// looking for the data bytes asked

		  //GYRO CALIBRATION

		  Gyr_Data[0]=((int16_t)BMX_buff[1]) << 8 | (BMX_buff[0]);  //X Raw (signed int)

		  Gyr_Data[1]=((int16_t)BMX_buff[3]) << 8 | (BMX_buff[2]);  //Y Raw

		  Gyr_Data[2]=((int16_t)BMX_buff[5]) << 8 | (BMX_buff[4]);  //Z Raw

		  Omega[0]=(float)Gyr_Data[0] * 0.0610351562;  //obtaining data dividing for MAX signed value and multiplying for the scale
		  Omega[1]=(float)Gyr_Data[1] * 0.0610351562;
		  Omega[2]=(float)Gyr_Data[2] * 0.0610351562;

		  Gyr_cal[0] += Omega[0];
		  Gyr_cal[1] += Omega[1];
		  Gyr_cal[2] += Omega[2];

		  //ACCELEROMETER

		  Acc_Data[0]=((int16_t)BMX_buff[7]) << 8 | (BMX_buff[6]); //same as gyro

		  Acc_Data[1]=((int16_t)BMX_buff[9]) << 8 | (BMX_buff[8]);

		  Acc_Data[2]=((int16_t)BMX_buff[11]) << 8 | (BMX_buff[10]);

		  Acc[0]= (float)Acc_Data[0] * 8 / 32768; //Conversion=8/32768 to obtain data in g
		  Acc[1]= (float)Acc_Data[1] * 8 / 32768;
		  Acc[2]= (float)Acc_Data[2] * 8 / 32768 - 1; //added 1 because the z axis detect the 1 g gravity acceleration but calibration is done on the errors

		  Acc_cal[0] += Acc[0];
		  Acc_cal[1] += Acc[1];
		  Acc_cal[2] += Acc[2];

		  while (LL_TIM_GetCounter(TIM1)<1000)	{

		  }

	}


	Gyr_cal[0] = Gyr_cal[0]/1000;
	Gyr_cal[1] = Gyr_cal[1]/1000;
	Gyr_cal[2] = Gyr_cal[2]/1000;

	Acc_cal[0] = Acc_cal[0]/1000;
    Acc_cal[1] = Acc_cal[1]/1000;
    Acc_cal[2] = Acc_cal[2]/1000;

    //END OF CONFIGURATION AND SETUP


	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //turn off green LED

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);//Toggle pinB7 to see it blink every second blue LED

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  LL_TIM_SetCounter(TIM1,0); //us counter set

	  //blue LED blinking control (toggle every second) ###### counter>1/sampleTime
	  if(counter>100){

		  counter=1;

		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);//Toggle pinB7 to see it blink every second

	  }

	  /*#################################
	   * GYRO DATA ACQUISITION AND ELABORATION
	   * 1) Get raw data and store in Gyro buffer;
	   * 2) Composition of the 2byte data and scaling to obtain deg/s (or g for accelerometer) float (32 bits) number;
	   * 3) Filtering the float data via a discrete LPF (sample time and c.o. freq. specified below);
	   * 4) Data utilization.
	   ##################################*/


	  //first is sent the bmx address with the register of the first data
	  buff[0]=GyroDataRegister[0];
	  HAL_I2C_Master_Transmit(&hi2c1, BMX160Address, buff, 1, HAL_MAX_DELAY); //transmitting 1 byte with the register of the LSBx
	  HAL_I2C_Master_Receive(&hi2c1, BMX160Address, BMX_buff, 12, HAL_MAX_DELAY);// looking for the data bytes asked

	  /* BMX_buff[0]=buff[0]; //X-LSB
	  BMX_buff[1]; //X-MSB
	  BMX_buff[2]; //Y-LSB
	  BMX_buff[3]; //Y-MSB
	  BMX_buff[4]; //Z-LSB
	  BMX_buff[5]; //Z-MSB */

	  /*This function take all the registers of the gyro just asking for the first one
	   * because the gyro automatically switch to next data register and send its content
	   * if it receives an ACK from master in Rx mode. Here the master sends the address and
	   * the first data register and ACK the next 6 bytes so the gyro continues to send data*/

	  //dividing the bytes

	  //now let's do the conversion to get a data in degrees per second

	  Gyr_Data[0]=((int16_t)BMX_buff[1]) << 8 | (BMX_buff[0]);  //X Raw (signed int)

	  Gyr_Data[1]=((int16_t)BMX_buff[3]) << 8 | (BMX_buff[2]);  //Y Raw

	  Gyr_Data[2]=((int16_t)BMX_buff[5]) << 8 | (BMX_buff[4]);  //Z Raw

	  Omega[0]=((float)Gyr_Data[0] * 0.0610351562) - Gyr_cal[0];  //obtaining data dividing for MAX signed value and multiplying for the scale
	  Omega[1]=((float)Gyr_Data[1] * 0.0610351562) - Gyr_cal[1];
	  Omega[2]=((float)Gyr_Data[2] * 0.0610351562) - Gyr_cal[2];

	  /*LPF specs:
	   * 1) Sample time Ts=0.01s (10 ms);
	   * 2) Cut off frequency selected is 10 Hz.
	   */

	  OmegaLPF[0]= k_toLPFprevSamp_Data * OmegaLPF_1[0] + k_toRaw_Data * (Omega[0] + Omega_1[0]); //LPFed data by discrete function see notebook p 11
      OmegaLPF[1]= k_toLPFprevSamp_Data * OmegaLPF_1[1] + k_toRaw_Data * (Omega[1] + Omega_1[1]);
	  OmegaLPF[2]= k_toLPFprevSamp_Data * OmegaLPF_1[2] + k_toRaw_Data * (Omega[2] + Omega_1[2]);

	  //COLLECTING DATA FOR ACCELEROMETER

	  Acc_Data[0]=((int16_t)BMX_buff[7]) << 8 | (BMX_buff[6]); //same as gyro

	  Acc_Data[1]=((int16_t)BMX_buff[9]) << 8 | (BMX_buff[8]);

	  Acc_Data[2]=((int16_t)BMX_buff[11]) << 8 | (BMX_buff[10]);

	  Acc[0]= (float)Acc_Data[0] * 8 / 32768 - Acc_cal[0]; //Conversion=8/32768 and also subtrating calibration data
	  Acc[1]= (float)Acc_Data[1] * 8 / 32768 - Acc_cal[1]; //Conversion is done to obtain data in g unit (1g=9.81 m/s^2)
	  Acc[2]= (float)Acc_Data[2] * 8 / 32768 - Acc_cal[2];

	  //ACCELEROMETER FILTER (SAME AS GYRO)
	  AccLPF[0]= k_toLPFprevSamp_Data * AccLPF_1[0] + k_toRaw_Data * (Acc[0] + Acc_1[0]); //LPFed data by discrete function see notebook p 11
	  AccLPF[1]= k_toLPFprevSamp_Data * AccLPF_1[1] + k_toRaw_Data * (Acc[1] + Acc_1[1]);
	  AccLPF[2]= k_toLPFprevSamp_Data * AccLPF_1[2] + k_toRaw_Data * (Acc[2] + Acc_1[2]);


	  //sending data to check the filtering on matlab: data is sent raw from gyro buff and elaborated on the reciever machine


	  //HAL_USART_Transmit(&husart3, BMX_buff, 12, HAL_MAX_DELAY); //transmit via serial port first 12 bytes in BMX_buff to operate on MATLAB

	  //loop to fill the BMX filtered data
	  for (i=0; i<6; i++) {
		  if (i<3)
			  *(BMX_Tx_LPF_buff+i) = *(OmegaLPF+i);
		  else
			  *(BMX_Tx_LPF_buff+i) = *(AccLPF+i-3);
	  }

	  HAL_USART_Transmit(&husart3, (uint8_t*)BMX_Tx_LPF_buff, 24, HAL_MAX_DELAY); // Tx 12 bytes of GYRO and 12 bytes ACC filtered->float number directly transmitted


	  //Refreshing of Gyro data store (data at preceding sample must be used before this passage)

	  Omega_1[0]=Omega[0];
	  Omega_1[1]=Omega[1];
	  Omega_1[2]=Omega[2];

	  OmegaLPF_1[0]=OmegaLPF[0];
	  OmegaLPF_1[1]=OmegaLPF[1];
	  OmegaLPF_1[2]=OmegaLPF[2];

	  Acc_1[0]=Acc[0];
	  Acc_1[1]=Acc[1];
	  Acc_1[2]=Acc[2];

	  AccLPF_1[0]=AccLPF[0];
	  AccLPF_1[1]=AccLPF[1];
	  AccLPF_1[2]=AccLPF[2];


	  counter=counter+1;//update the counter value

	  //Function to create the 10ms loop

	  while( LL_TIM_GetCounter(TIM1)<10000){

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2010091A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 96;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
