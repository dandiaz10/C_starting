/* USER CODE BEGIN Header */
/*******************************************************************************
  * File Name          : main.c
  * Description        : it is a GPS data processor that depending on the user
  * 					 choice will print the Time, Latitude, Longitude and
  * 					 Altitude through serial port and to a LCD 16*2.
  *
  * Author:              Daniela Diaz Tenorio
  * Date:                April 21st 2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "debounce.h"
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
UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t readPush(int16_t buttonsPin [], int8_t numberOfButtons, char port, int8_t mode);
void printLcdVcn(char printBuffer1[], char printBuffer2[]);


struct gpsData {
   char*  time;
   char*  latitude;
   char* latitudeM;
   char*  longitude;
   char*  longitudeM;
   char*  altitude;
   char*  altitudeM;
};
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
  /* USER CODE BEGIN 2 */
  	HD44780_Init();

   //Initializing push buttons variables
   	char port = 'A';
   	int8_t mode = 0;
   	int16_t buttonsPin[] = {GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_7,GPIO_PIN_8};// pins for the PB
   	int8_t numberOfButtons = (sizeof(buttonsPin))/(sizeof(buttonsPin[0]));// number of PBs used

   	deBounceInit(buttonsPin[0], port, mode); //push1 PA3
   	deBounceInit(buttonsPin[1], port, mode); //push2 PA4
   	deBounceInit(buttonsPin[2], port, mode); //push3 PA7
 	deBounceInit(buttonsPin[3], port, mode); //pushOK PA8

 	// initializing variables
	int8_t state = 1; //state machine state
	const int processingTime = 5000; //delay time when is processing
	char printBuffer [16] = { 0 };

	char stringData[73] = {0}; //input

	//stringData format examples:
	//{"$GPGGA,014729.10,4303.5753,N,08019.0810,W,1,6,1.761,214.682,M,0,M,0,*5D"}; //
	//$GPGGA,14732.4,4306.6233,N,8042.15,W,1,5,1.761,215.74,M,0,M,0,*5D


	struct gpsData message;
	int8_t position = 6;
	int8_t buttonPushed = 0; //which button is pressed

	float time = 0;
	int hours = 0;
	int minutes = 0;
	float seconds = 0;
	float latitude = 0;
	int degrees = 0;
	int minutess = 0;
	float decimal = 0;
	float longitude = 0;
	float altitude = 0;

  /* USER CODE END 2 */

 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch(state)
	  {
	  case 1: // Retreiving Data

		  printLcdVcn("Retrieving Data"," ");
		  scanf("%s",stringData); //waiting for data through putty
		  printf("%s \r\n",stringData);

		  state = 2;

	  break;

	  case 2:// Processing Data (split data)

		    printLcdVcn("Processing Data..."," ");

			const char *delim = ",";

			message.time = strtok(stringData,delim);
			message.time = strtok(NULL,delim);

			message.latitude = strtok(NULL,delim);
			message.latitudeM = strtok(NULL,delim);

			message.longitude = strtok(NULL,delim);
			message.longitudeM = strtok(NULL,delim);

			while( position < 10)
			{

				if(position == 9){
					message.altitude = strtok(NULL,delim);
					message.altitudeM = strtok(NULL,delim);
				}
				else
				{
					message.altitudeM = strtok(NULL,delim);
				}

					position ++;
			}

		    HAL_Delay(processingTime);

		    state = 3;

		  break;

	  case 3: //task selection
		  printLcdVcn("1)Time,2)Latitu", "3)Longi,4)Altitu");
		  buttonPushed = readPush(buttonsPin, numberOfButtons, port, mode);//wait for PB

		  switch (buttonPushed){ //which PB was pressed
		  	  case 0: //PB 1
		  		  	 state = 4;
			  break;
		  	  case 1: //PB 2
		  		  	  state = 5;
		  	  break;
		  	  case 2: //PB 3
		  		  	  state = 6;
			  break;
		  	  case 3: //PB 4
		  		      state =7;
		  	  break;
		  }

	  break;

	  case 4://Time

		time = atof(message.time);
		hours = time/10000;
		minutes = (time - (hours*10000))/100;
		seconds = (time - (hours*10000)-(minutes*100));

		sprintf(printBuffer," %dh %dm %2.2fs",hours, minutes, seconds);
		printLcdVcn("Time:" ,printBuffer);

		HAL_Delay(processingTime); //wait for 5 sec

		state = 3; //go to Task selection

	 break;

	  case 5://latitude

		latitude = atoff(message.latitude);
		degrees = latitude/100;
		minutess = latitude - (degrees*100);
		decimal = (latitude - ((degrees*100)+minutess))*10000;
		sprintf(printBuffer," %dd %dm %.0fdd",degrees, minutess, decimal);
		printLcdVcn("Latitude:",printBuffer);

		HAL_Delay(processingTime); //wait for 5 sec

		state = 3; // go to Task selection

	  break;

	  case 6://Longitude

        longitude = atof(message.longitude);
        degrees = longitude/100;
        minutess = longitude - (degrees*100);
        decimal = (longitude - ((degrees*100)+minutess))*10000;
		sprintf(printBuffer," %dd %dm %4.0fdd",degrees, minutess, decimal);
		printLcdVcn("longitude:",printBuffer);

		HAL_Delay(processingTime);//wait for 5 sec

		state=3; //go to task selection

      break;

	  case 7: //Altitude
		  altitude = atof(message.altitude);
		  sprintf(printBuffer, " %.3f meter",altitude);
		  printLcdVcn("Altitude", printBuffer);

		  HAL_Delay(processingTime); //wait for 5 sec

		  state= 3; //go to task selection
	  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
  * FUNCTION           : readPush
  * DESCRIPTION        : read and identify which PB was pressed.
  *
  * PARAMETERS		   : int16_t buttonsPin[]: pin number initialized for the input.
  * 					 int8_t numberOfButtons : amount of buttons to consider.
  * 					 char port: port of the pin input.
  * 				     int8_t mode: mode of the pin input. 0 for Pullup and 1 Pulldown
  *
  * RETURN			   : int8_t button: the number of the PB pressed.
  */
int8_t readPush(int16_t buttonsPin [], int8_t numberOfButtons, char port, int8_t mode) {

int8_t button = 0;
int8_t pushContains [numberOfButtons];

for(int i = 0; i < numberOfButtons; i++)
{
	pushContains[i] = deBounceReadPin(buttonsPin[i], port, mode); // read PB 1
}


// check if a PB hasn't been pressed and is lower than the number of buttons
while(pushContains[button] && button < numberOfButtons)
		{
			pushContains[button] = deBounceReadPin(buttonsPin[button], port, mode);
			button ++;

			if(button == numberOfButtons) // restart the variable button
			{
			button = 0;
			}
		}

return button;
}

/*
  * FUNCTION           : printLcdVcn
  * DESCRIPTION        : Print values from 2 char arrays in the LCD and
  * 					 trough the serial port.
  *
  * PARAMETERS		   : char printBuffer1[]: the first line string to be printed.
  * 					 char printBuffer2[]: the second line string to be printed.
  *
  * RETURN			   : no return.
  */
void printLcdVcn(char printBuffer1[], char printBuffer2[])
{

//printing in the LCD
	HD44780_ClrScr();
	HD44780_GotoXY(0, 0);
	HD44780_PutStr(printBuffer1); //print LCD
	HD44780_GotoXY(0, 1);
	HD44780_PutStr(printBuffer2);

 //printing the same values through the serial port
	printf(printBuffer1);
	printf(" ");
	printf(printBuffer2);
	printf(" \r \n");

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
