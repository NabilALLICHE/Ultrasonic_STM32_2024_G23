/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "lib_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM	TIM2 // timer pour cr er un delai en microsecondes
static rgb_lcd lcdData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void GPIO_INPUT(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint32_t measureHighSignalDuration() {

	    uint32_t startTime = 0;
	    uint32_t endTime = 0;

	    // Attendre que le signal passe à l'état haut
	    while (HAL_GPIO_ReadPin(GPIOA, LD2_Pin|GPIO_PIN_8) == GPIO_PIN_RESET) {
	        // Mesurer le temps de début
	        startTime = TIM2->CNT;
	    }

	    // Attendre que le signal passe à l'état bas
	    while (HAL_GPIO_ReadPin(GPIOA, LD2_Pin|GPIO_PIN_8) == GPIO_PIN_SET) {
	        // Mesurer le temps de fin
	        endTime = TIM2->CNT;
	    }

	    // Calculer la durée de l'état haut en microsecondes
	    uint32_t duration = endTime - startTime;

	    return duration;
}

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

void usDelay(uint32_t uSec);  // d claration de la fonction qui permet d'avoir un delai en microsecondes

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float speedOfSound = 0.0340/2;   // vitesse du son
float distance; // variable ou stocker la distance

char uartBuf[100]; // buffer qui permet de transmettre au pc la distance calcul
char uartBuf1[100];
void configureInputSIGPin()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Entrée
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t numTicks = 0;  // variable utilise  pour calculer  le delai
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    //DWT_Delay_Init();
    lcd_init(&hi2c1, &lcdData); // initialise le lcd
    lcd_position(&hi2c1,0,0);
    lcd_print(&hi2c1,"Bilna");
    reglagecouleur(0,0,255);
    HAL_Delay(3000);
    clearlcd();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(1000);

	  	  //mettre le Trig a l' tat bas pendant 3us
	  	 	  		//HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	  	 	  		//usDelay(3);
	  	          MX_GPIO_Init();
	  	          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	  	          usDelay(3);
	  	 	  		//*** commencer la mesure ***//
	  	 	  		//1. on maintien Trig   l' tat haut pendant 10us pour commencer le calcul
	  	 	  		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	  	 	  		usDelay(10);
	  	 	  		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	  	 	  	    configureInputSIGPin();
	  	 	  	  //2. Attendre le front montant du pin Echo
	  	 	  	  	 	  		while(HAL_GPIO_ReadPin(TRIG_GPIO_Port, TRIG_Pin) == GPIO_PIN_RESET);

	  	 	  	  	 	  		//3. Commencer   mesurer la largeur d'impulsion Echo en usec
	  	 	  	  	 	  		numTicks = 0;
	  	 	  	  	 	  		while(HAL_GPIO_ReadPin(TRIG_GPIO_Port, TRIG_Pin) == GPIO_PIN_SET)
	  	 	  	  	 	  		{
	  	 	  	  	 	  			numTicks++;
	  	 	  	  	 	  			usDelay(2); //2.8us en vrai car la fonction utilis  ne donne pas exactement 2us
	  	 	  	  	 	  		};

	  	 	  	  	 	  		//4. calcule de la distance en cm
	  	 	  	  	 	  		distance = (numTicks + 0.0f)*2.8*speedOfSound+0.4;

	  	 	  	  	 	  		//5. transmettre la distance vers le pc pour l'afficher
	  	 	  	  	 	  		sprintf(uartBuf, "Dis  = %.1f ",  distance);
	  	 	  	  	 	        lcd_position(&hi2c1,0,0);
	  	 	  	  	 	   	    lcd_print(&hi2c1,uartBuf);

	  	 	  	          	   lcd_position(&hi2c1,14,0);
	  	 	  	          	   lcd_print(&hi2c1, "cm");
	  	 	  	  	           lcd_print(&hi2c1,uartBuf);

	  	 	  	  	 	  		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

	  	 	  	  	 	  		//HAL_Delay(1000);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec) // fonction qui nous sert a cr er un d lai en us
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
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
