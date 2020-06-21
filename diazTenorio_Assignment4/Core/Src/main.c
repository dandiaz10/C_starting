/* USER CODE BEGIN Header */
/*******************************************************************************
  * File Name          : main.c
  * Description        : it is a debit card transaction machine.
  * 					 it process a purchase verifying the price to pay
  * 					 type of account, check that the pin is correct and
  * 					 goes back to the initial state.
  *
  * Author:              Daniela Diaz Tenorio
  * Date:                April 13st 2020
  ******************************************************************************
  */
/* USER CODE END Header */

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
int8_t waitForPush (int8_t minValue, int8_t maxValue, char port, int8_t mode, int16_t buttonsPin[], int8_t numberOfButtons);
void printLcdVcn(char printBuffer1[], char printBuffer2[]);
int8_t checkPin(char pin[]);
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
	const int processingTime = 10000; //delay time when is processing
	const int pinSize = 4;//pin length
	char pin[pinSize];
	char printBuffer [16] = { 0 };

	int8_t getButton = 0; //which button is pressed
	float price = 0; // price to pay

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state)
	  {
	  	  case 1: // welcome screen
	  		printLcdVcn("Welcome", " ");
	  		getButton = waitForPush(-1, 3, port, mode,buttonsPin,numberOfButtons); //wait until OK is pressed
	  		state = 2; //next step
		  break;

		  case 2: // enter and accept the price
			printf("enter price: ");
			scanf("%f",&price); // wait until the price is provided through serial port
			sprintf(printBuffer, "$%.2f", price);
			printLcdVcn(printBuffer, "Ok:1 or Canc:2");

			getButton = waitForPush(1, 4, port, mode, buttonsPin,numberOfButtons);//wait until 1 or 2 is pressed

			if(getButton == 0) //pressed OK: PB 1
			{
				state = 3; //next state
			}
			else //pressed cancel: PB 2
			{
				state = 6; // cancel transaction
			}
		  break;

		  case 3: // choose chequing or saving account
		    printLcdVcn("Chequing: 1", "Saving: 2");
		    getButton = waitForPush(1, 4, port, mode, buttonsPin,numberOfButtons); //wait until 1 or 2 is pressed
			state = 4; // next state
		  break;

		  case 4:
			printLcdVcn("Enter PIN", " ");
			memset(printBuffer,' ',16);

			for(int i=0; i<pinSize; i++) //save the pin entered in pin[]
			{
				pin[i] = (readPush(buttonsPin, numberOfButtons, port, mode)+1)+'0'; //convert integer to char
				HD44780_GotoXY(i, 1);
				HD44780_PutChar('*');
			}

			getButton = waitForPush(-1, 3, port, mode, buttonsPin,numberOfButtons); //wait until OK is pressed

			state = 5; //next state
		  break;

		  case 5:

			printLcdVcn("Processing...", " ");
			HAL_Delay(processingTime);

			state = checkPin(pin); //check if the pin is correct
		  break;

		  case 6:

			printLcdVcn("Transaction", "Cancelling");
			HAL_Delay(processingTime);

			state = 1; //go back to welcome screen
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
  * FUNCTION           : waitForPush
  * DESCRIPTION        : waits for an specific PB to be pressed depending on the
  * 					 minValue and the maxValue parameters.
  *
  * 					 if PB 1 or 2 are pressed minValue = 1 and maxValue = 4
  * 					 if PB 1,2 or 3 are pressed minValue = -1 and maxValue = 3
  *
  *
  * PARAMETERS		   : int8_t minValue: minimum value to limit the buttons to read.
  * 				     int8_t maxValue: maximum value to limit the buttons to read.
  * 				     char port: port of the pin input.
  * 				     int8_t mode: mode of the pin input. 0 for Pullup and 1 Pulldown
  * 				     int16_t buttonsPin[]: pin number initialized for the input.
  * 				     int8_t numberOfButtons : amount of buttons to consider.
  *
  * RETURN			   : int8_t getButton: the number of the PB pressed.
  */
int8_t waitForPush (int8_t minValue, int8_t maxValue, char port, int8_t mode, int16_t buttonsPin[], int8_t numberOfButtons)
{
	int8_t getButton = 0;
	getButton = readPush(buttonsPin, numberOfButtons, port, mode); // waiting for pushing a button
	while(getButton > minValue && getButton < maxValue) // check if OK was pressed
	{
		getButton = readPush(buttonsPin, numberOfButtons, port, mode); // waiting for pushing a button
	}

	return getButton;
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

 //printing the same values throuhg the serial port
	printf(printBuffer1);
	printf(" ");
	printf(printBuffer2);
	printf(" \r \n");

}

/*
  * FUNCTION           : checkPin
  * DESCRIPTION        : check if the pin is correct
  *
  * PARAMETERS		   : char pin[]: is the pin entered by the user.
  *
  * RETURN			   : int8_t state: the next state for the state machine
  */
int8_t checkPin(char pin[]) // check if the pin is correct
{
	int8_t state;
	int8_t p = 0;
	int pinInt = atoi(pin);
	int rightPins[] = {1111, 1121, 1133, 1213, 2111, 2331, 3111, 3211, 3333};
	int rightPinsLength = (sizeof(rightPins)/(sizeof(rightPins[0])));

	while(pinInt != rightPins[p])
	{
		p++;

		if(p >= rightPinsLength )
		{
			printf("pin is NOT correct ...\r \n");
			state = 6;
			break;
		}
	}
	if (p == 8)
	{
		printf("pin is correct ...\r \n");
		printf("no funds ...\r \n");
		state = 6;
	}
	else if (p < 8)
	{
		printf("pin is correct ...\r \n");
		printf("printing receipt ...\r \n");
		state = 1;
	}

	return state;
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
