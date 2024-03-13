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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_txt.h"
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int array[10]={};
int col=0;
int ispravan_pass[10]={1,1,2,2};

int kliknuo_OK=0;
int kliknuo_UNOS=0;
int vrata_otvorena=0;
int dozvoljen_unos=0;
int desio_se_interrupt=0;
int enable_dugme_UNOS=1;



/* USER CODE END 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(vrata_otvorena==1)
		  enable_dugme_UNOS=0;
	else{
   		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);

		  /* Prevent unused argument(s) compilation warning */
		  UNUSED(GPIO_Pin);

		  enable_dugme_UNOS=1;
		  kliknuo_OK=0;
		  kliknuo_UNOS=1;
		  vrata_otvorena=0;
	}
}

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);

  lcd_init();
  lcd_puts(0,0,(int8_t*)"Sef je zakljucan");
  HAL_Delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(col<=10){

	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"0"); //lcd_puts(1,col,(int8_t*)"*"); //za prekrivanje unosa
		  HAL_Delay(500);
		  array[col]=0;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"1");
		  HAL_Delay(500);
		  array[col]=1;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"2");
		  HAL_Delay(500);
		  array[col]=2;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"3");
		  HAL_Delay(500);
		  array[col]=3;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"4");
		  HAL_Delay(500);
		  array[col]=4;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"5");
		  HAL_Delay(500);
		  array[col]=5;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_13)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"6");
		  HAL_Delay(500);
		  array[col]=6;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"7");
		  HAL_Delay(500);
		  array[col]=7;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"8");
		  HAL_Delay(500);
		  array[col]=8;
		  col+=1;
	  }
	  if(col<10 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0 && kliknuo_OK==0 && dozvoljen_unos==1){
		  kliknuo_UNOS=0;
		  lcd_puts(1,col,(int8_t*)"9");
		  HAL_Delay(500);
		  array[col]=9;
		  col+=1;
	  }
	  if(kliknuo_OK==0 && kliknuo_UNOS==1 && vrata_otvorena==0 && enable_dugme_UNOS==1)
	  {
		  lcd_clear();
		  lcd_puts(0,0,(int8_t*)"Unesite sifru");
		  for(int i=0; i<col; i++){
			array[i]=0;
		  }
		  col=0;
		  dozvoljen_unos=1;
		  kliknuo_UNOS=0;

		  HAL_Delay(500);
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==0 && vrata_otvorena==0 && dozvoljen_unos==1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
		  kliknuo_OK=1;
		  kliknuo_UNOS=0;
		  dozvoljen_unos=0;
		  lcd_clear();
		  if(col==0) {
			  lcd_puts(0,0,(int8_t*)"Nema unosa");
			  HAL_Delay(1000);
			  lcd_clear();
			  lcd_puts(0,0,(int8_t*)"Sef je zakljucan");
		  }
		  else if (array[0]==ispravan_pass[0] && array[1]==ispravan_pass[1] &&
			  array[2]==ispravan_pass[2] && array[3]==ispravan_pass[3]
			  && array[4]==0 && ispravan_pass[4]==0){
			lcd_puts(0,0,(int8_t*)"Ispravna sifra");
			lcd_puts(1,0,(int8_t*)"Otkljucavanje...");
			enable_dugme_UNOS=0;

			//motor - otvara vrata
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
			HAL_Delay(500);

			vrata_otvorena=1;

			lcd_clear();
			lcd_puts(0,0,(int8_t*)"Vrata otkljucana");
			HAL_Delay(500);
		  }
		  else{
			lcd_puts(0,0,(int8_t*)"Neispravan unos");
			vrata_otvorena=0;
			HAL_Delay(1000);
			lcd_clear();
			lcd_puts(0,0,(int8_t*)"Sef je zakljucan");
		  }

		 HAL_Delay(500);
	 }
	 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0 && vrata_otvorena==1){
		 vrata_otvorena=0;
		 dozvoljen_unos=0;
		 kliknuo_OK=0;
		 enable_dugme_UNOS=1;
		 lcd_clear();
		 lcd_puts(0,0,(int8_t*)"Zatvaranje...");

		 //motor - zatvara vrata
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
		 HAL_Delay(2000);
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
		 HAL_Delay(500);

		 lcd_clear();
		 lcd_puts(0,0,(int8_t*)"Sef je zakljucan");

	  }
	 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0 && dozvoljen_unos==1){
		 //ponisti unos
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
		 lcd_clear();
		 lcd_puts(0,0,(int8_t*)"Ponisten unos");
		  for(int i=0; i<col; i++){
			array[i]=0;
		  }
		  col=0;
		  dozvoljen_unos=0;
		 HAL_Delay(1000);
		 lcd_clear();
		 lcd_puts(0,0,(int8_t*)"Sef je zakljucan");
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 146;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA10 PA11
                           PA12 PA13 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
}
*/

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


		if( htim->Instance == TIM3) {
		    if(vrata_otvorena==1)
			{
				//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);

			}
			else HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_RESET);
		  }





  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
