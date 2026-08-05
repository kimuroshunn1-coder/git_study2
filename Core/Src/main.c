/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INV_SQRT_2 0.70710678f // √2分の１の定義

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//ESP32からの速度指令を受け取るための変数
volatile float Vx = 0.0f;
volatile float Vy = 0.0f;
volatile float omega = 0.0f;

//車体中心からオムニまでの長さ
const float R = 0.21f;

//ESP32からの速度指令を受け取るための箱
char rxBuf[64];
volatile uint8_t rxFlag = 0; // 受信完了フラグ
uint8_t rxChar; // 受信文字
volatile uint8_t rxIndex = 0; // 受信箱のインデックス

//講習モーターのためのテキストヘッダー
FDCAN_TxHeaderTypeDef TxHeader_motor;

//モーター変数　反時計回りに右上からABCD
float motor_speed_A = 0.0f;
float motor_speed_B = 0.0f;
float motor_speed_C = 0.0f;
float motor_speed_D = 0.0f;

//電流地
uint8_t TxData[8];

//PID制御のための変数
typedef struct {
  uint16_t CANID;
  float trgVel;
  int16_t actVel;
  int16_t p_actVel;
  float hensa;
  float ind;
  int16_t cu; 
} Robomas_t;
Robomas_t robomas[4];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
void OmniKinematics(void);
void SendMotorCurrent(int16_t current_A, int16_t current_B, int16_t current_C, int16_t current_D);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// CAN_SEND(0x7FE, TxData, &hfdcan1, &TxHeader); // example usage
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  Hfdcan_Filter_Settings->FilterID1 = 0x200;
  Hfdcan_Filter_Settings->FilterID2 = 0x410;
}

void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x200;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_CLASSIC_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)

{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  motor_CAN_filter_init(&FDCAN_Filter_settings);
  motor_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan3))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
 	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

    /* Retrieve Rx messages from RX FIFO0 */
		uint8_t RxData_motor[8] = {};
    FDCAN_RxHeaderTypeDef RxHeader_motor;
 		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor)) {
 			printf("fdcan_getrxmessage_motor is error\r\n");
 			Error_Handler();
 		}
		/*receive robomas's status*/
 		for (int i=0; i < 4; i++){
 			if (RxHeader_motor.Identifier == (robomas[i].CANID)) {
 				//robomas[i].actangle = (int16_t)((RxData_motor[0] << 8) | RxData_motor[1]);
 				robomas[i].actVel = (int16_t)((RxData_motor[2] << 8) | RxData_motor[3]);
 				//robomas[i].actCurrent = (int16_t)((RxData_motor[4] << 8) | RxData_motor[5]);
 			}
 		}
 	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if (&htim6 == htim) {
		float k_p = 7, k_i = 0.5, k_d = 0.1;
		for (int i = 0; i < 4; i++){
			robomas[i].hensa = robomas[i].trgVel - robomas[i].actVel;
			if (robomas[i].hensa >= 1000) robomas[i].hensa = 1000;
			else if (robomas[i].hensa <= -1000) robomas[i].hensa = -1000;
			float d = (robomas[i].p_actVel - robomas[i].actVel) / 0.001f;
			robomas[i].ind += robomas[i].hensa*0.001f;
			if (d >= 30000) d = 30000;
			else if (d <= -30000) d = -30000;
			if (robomas[i].ind >= 10000) robomas[i].ind = 10000;
			else if (robomas[i].ind <= -10000) robomas[i].ind = -10000;


			float t = k_p*robomas[i].hensa;
			if (t>=10000) t = 10000;
			else if (t<=-10000) t = -10000;
			robomas[i].cu = (int16_t)(t+k_i*robomas[i].ind+k_d*d);
			if (robomas[i].cu <= -10000) robomas[i].cu = -10000;
			else if (robomas[i].cu >= 10000) robomas[i].cu = 10000;

			robomas[i].p_actVel = robomas[i].actVel;

		}
    SendMotorCurrent(robomas[0].cu, robomas[1].cu, robomas[2].cu, robomas[3].cu);
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); // Start TIM6 in interrupt mode
  HAL_UART_Receive_IT(&huart2, &rxChar, 1); // Start UART reception in interrupt mode
  //講習にて導入　 
  __HAL_RCC_CLEAR_RESET_FLAGS(); // Clear reset flags
  if (HAL_OK != motor_CAN_RxTxSettings_init(&TxHeader_motor)) Error_Handler();
  //モーターのCANIDを設定
  robomas[0].CANID = 0x201;
  robomas[1].CANID = 0x202;
  robomas[2].CANID = 0x203;
  robomas[3].CANID = 0x204;
  //一応初期化
  for (int i = 0; i < 4; i++){
    robomas[i].trgVel = 0;
    robomas[i].actVel = 0;
    robomas[i].p_actVel = 0;
    robomas[i].hensa = 0;
    robomas[i].ind = 0;
    robomas[i].cu = 0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*uint8_t txdata[8] = {0 ,1 ,2,3,4,5,6,7}; // Example data to send
    int16_t cu = 842; // Example current value
    txdata[0] = (uint8_t)(cu >> 8);
    txdata[1] = (uint8_t)(cu & 0xFF);
    txdata[2] = (uint8_t)(cu >> 8);
    txdata[3] = (uint8_t)(cu & 0xFF);
    txdata[4] = (uint8_t)(cu >> 8);
    txdata[5] = (uint8_t)(cu & 0xFF);
    txdata[6] = (uint8_t)(cu >> 8);
    txdata[7] = (uint8_t)(cu & 0xFF);
    CAN_SEND(0x200, txdata, &hfdcan3, &TxHeader_motor); // Send CAN message
    HAL_Delay(100);
    */
   Vx = 0.0;
   Vy = 0.0;
   omega = 0.0;  //1.0くらいを想定している
   OmniKinematics();
   HAL_Delay(100);
    
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//逆運動学オムニ
void OmniKinematics(void){
  motor_speed_A = INV_SQRT_2 * (Vx - Vy) - omega * R;
  motor_speed_B = INV_SQRT_2 * (Vx + Vy) - omega * R;
  motor_speed_C = INV_SQRT_2 * (- Vx + Vy) - omega * R;
  motor_speed_D = INV_SQRT_2 * (- Vx - Vy) - omega * R;
  //正規化
  float max_speed = fabsf(motor_speed_A);
  if (fabsf(motor_speed_B) > max_speed) {max_speed = fabsf(motor_speed_B);}
  if (fabsf(motor_speed_C) > max_speed) {max_speed = fabsf(motor_speed_C);}
  if (fabsf(motor_speed_D) > max_speed) {max_speed = fabsf(motor_speed_D);}
  if (max_speed > 1.0f) {
    motor_speed_A /= max_speed;
    motor_speed_B /= max_speed;
    motor_speed_C /= max_speed;
    motor_speed_D /= max_speed;
  }
  //PIDにtrgVelを渡す
  robomas[0].trgVel = motor_speed_A*3000;
  robomas[1].trgVel = motor_speed_B*3000;
  robomas[2].trgVel = motor_speed_C*3000;
  robomas[3].trgVel = motor_speed_D*3000;
}

//ESP32からの速度指令を受け取るためのUART割り込みコールバック関数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) { // Check if the interrupt is from USART2
    if (rxChar == '\n') { // Check for newline character
      rxBuf[rxIndex] = '\0'; // Null-terminate the string
      rxFlag = 1; // Set the flag to indicate a complete command has been received
      rxIndex = 0; // Reset index for next command
    } else {
      rxBuf[rxIndex++] = rxChar; // Store received character and increment index
      
      if (rxIndex >= sizeof(rxBuf) - 1) { // Prevent buffer overflow
        rxIndex = 0; // Reset index if buffer is full
      }
    }
    HAL_UART_Receive_IT(&huart2, &rxChar, 1); // Restart UART reception in interrupt mode
  }
}

//電流値の送信関数
void SendMotorCurrent(
    int16_t current_A,
    int16_t current_B,
    int16_t current_C,
    int16_t current_D)
{
    TxData[0] = (current_A >> 8) & 0xFF;
    TxData[1] = current_A & 0xFF;

    TxData[2] = (current_B >> 8) & 0xFF;
    TxData[3] = current_B & 0xFF;

    TxData[4] = (current_C >> 8) & 0xFF;
    TxData[5] = current_C & 0xFF;

    TxData[6] = (current_D >> 8) & 0xFF;
    TxData[7] = current_D & 0xFF;

    CAN_SEND(
        0x200,
        TxData,
        &hfdcan3,
        &TxHeader_motor
    );
}


/*void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

		uint8_t RxData[64] = {};
    FDCAN_RxHeaderTypeDef RxHeader;
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}
    switch (RxHeader.Identifier)
    {
      case 0x200:
        break;
        default:
        break;
    }
	}
}

void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x7ff;
}

void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x00;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_FD_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}
*/




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
#ifdef USE_FULL_ASSERT
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
