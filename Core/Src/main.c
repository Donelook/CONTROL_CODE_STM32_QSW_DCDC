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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "math.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_INIT,
    STATE_STANDBY,
    STATE_SOFT_START,
    STATE_REGULATION,
    STATE_FAULT,
    STATE_SHUTDOWN
} ConverterState;

typedef enum {
    EVENT_START,
    EVENT_FAULT,
    EVENT_CLEAR_FAULT,
    EVENT_SHUTDOWN
} ConverterEvent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MA_WINDOW_SIZE 10  		// Moving average window
#define BUFFER_SIZE 100  		// Buffer size for DAC test - not used in final program
#define res12_b 4096
#define L_IND 0.000094 			// 94uH
#define C_CAP 0.0000000022 		// 2.2nF
#define wr sqrt(L_IND*C_CAP)		// Omega of LC resonance
#define Z sqrt(L_IND/(2*C_CAP)) // impedance of inductor and two capacitor on Dren-Source MOSFETs
#define Ts 0.00005			// Sampling rate of control loop 20khz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;
DMA_HandleTypeDef hdma_adc5;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac2_ch1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_ADC5_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

// Simple moving average filter
void calculateMovingAverage(uint16_t src[5][MA_WINDOW_SIZE], float dst[5][MA_WINDOW_SIZE]);

//PWM setting for FAN BUCK converter
void Set_PWM_DutyCycle(uint32_t dutyCycle);

//USB
void USB_SendString(char* message);

//State machine
ConverterState handle_event(ConverterState currentState, ConverterEvent event);

// PWM for send delay times to FPGA function TIM1 TIM8
void Update_PWM_Frequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);
void Update_PWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t dutyCycle);
void FillBuffer(void);

//Checks
uint8_t Check_Faults();
uint8_t Check_Ready();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t dac_buffer[BUFFER_SIZE];
void Read_ADC3(void);
float CURRENT_SENSOR1_VREF = 0; // 1.5V - 0 A
float CURRENT_SENSOR2_VREF = 0; // 1.5V - 0 A
float IMAX1 = 0;
float IMAX2 = 0;
float IMIN = 0;
float INPUT_VOLTAGE = 0; 		// 0.2V BIAS
float PCB_TEMP = 0;				// Temperature of PCB
float HEAT_SINK_TEMP = 0; 		// Temperature of heatsink
void FAN_Drive();
volatile static uint32_t ADC3_DMA_BUFFER[5];
volatile static uint32_t ADC3_MEASURMENTS[5][MA_WINDOW_SIZE];
float ADC3_MOVING_AVERAGE[5][MA_WINDOW_SIZE];


/* ADC3_MEASURMENTS[x]
 * 0 - Current Sensor 1 Vref(1.5V Nominal)
 * 1 - Current Sensor 2 Vref(1.5V Nominal)
 * 2 - Input Voltage(0V2 bias)
 * 3 - PCB temperature (MCP9700)
 * 4 - Heatsink Temprature (TMP236)
 */
uint16_t Vref = 54; 			//  Reference voltage its compare to output voltage
uint16_t step_size = 0.00125;
float OUTPUT_VOLTAGE = 0; 	// Measured voltage 0.2V BIAS
uint16_t Vout = 0; 				//Voltage comparing to Vref
uint16_t Vramp = 0; 			// ramp voltage
volatile static uint32_t ADC4_DMA_BUFFER;
volatile static uint32_t ADC4_MEASURMENTS;
uint8_t RAMP();
void regulatorPI(float *out, float *integral, float in, float in_zad, float limp, float limn, float kp, float ti, float Ts1);
float delay_tr = 0; // DELAY/DEADTIME after first stage inductor  positive ramp
float delay_hc = 0; // DELAY/DEADTIME after second stage inductor negative ramp

//Filter butterworth 500khz sampleing rate 200khz cutoff
/*#define N 4 // Order of the filter
float x[N+1] = {0}; // Input samples
float y[N+1] = {0}; // Output samples
float b_z[N+1] = { 0.4328,   1.7314,   2.5971,   1.7314,   0.4328};
float a_z[N+1] = { 1.0000,   2.3695,   2.3140,   1.0547,   0.1874 };
void  BUTTERWORHT_FILTER(float new_sample);*/

//Simple low pass filter
#define ALPHA 0.1 // smoothing factor 0-1
float Low_pass_filter(float new_sample, float old_sample);


/* ADC4_MEASRUMENTS[X]
 * 0 - Output Voltage(0V2 bias)
 *
 */



/* ADC5_MEASRUMENTS[X]
 * 0 - IMAX2_SUM
 *
 */
float IMAX2_SUM = 0;
volatile static uint32_t ADC5_DMA_BUFFER[MA_WINDOW_SIZE];
volatile static uint32_t ADC5_MEASURMENTS[MA_WINDOW_SIZE];
float ADC5_MOVING_AVERAGE;

// USB INTERFACE DISPLAY AND SET
// Buffer to hold incoming data
uint8_t USB_RX_Buffer[64];
uint8_t USB_TX_Buffer[128];
void SendUSBMessage(const char* message);
void ParseUSBCommand(void);
void DisplayAllVariables(void);
volatile uint8_t dataReceivedFlag = 0; // Flags to indicate new data received


//Regulator PI
float Kp = 0.15; 			// Proportional part of PI
float Ti = 0.005; 			// Integral part of PI
float LIM_PEAK_POS = 40; 	// Positive limit for PI regulator
float LIM_PEAK_NEG = -4; 	// Negative limit for PI regulator
float Integral_I = 0;		// Integral part of PI
float prev_delta = 0; 		// buffer  error n-1

uint8_t START = 0;
uint8_t CLEAR = 0;
ConverterEvent event = EVENT_SHUTDOWN;
ConverterState currentState = STATE_INIT;

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
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USB_Device_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//event = EVENT_START;
  //USB_SendString("PRZED WHILEM\r\n");


  while (1)
  {


	  	  	  	  if (dataReceivedFlag) {
	  	  	  	      // Process the data
	  	  	  	      ParseUSBCommand();  // Function to handle the received command

	  	  	  	      // Clear the flag after processing
	  	  	  	      dataReceivedFlag = 0;
	  	  	  	  }

	  	  	  	  uint8_t interlock = HAL_GPIO_ReadPin(INTERLOCK_GPIO_Port, INTERLOCK_Pin);
	  	          if (interlock == 1 && START && Check_Faults()   && Check_Ready()/* start condition */) {
	  	        	//USB_SendString("State: EVENT START \r\n");
	  	              event = EVENT_START;
	  	          } else if (HAL_GPIO_ReadPin(INTERLOCK_GPIO_Port, INTERLOCK_Pin)/* fault condition */) {
	  	              event = EVENT_FAULT;
	  	          } else if (CLEAR) {
	  	        	  /* clear fault condition */
	  	              event = EVENT_CLEAR_FAULT;
	  	          } else if (0/* shutdown condition */) {
	  	              event = EVENT_SHUTDOWN;
	  	          }

	  	          // Handle the event and update the state
	  	          currentState = handle_event(currentState, event);

	  	          // Perform actions based on the current state
	  	          switch (currentState) {
	  	              case STATE_INIT:
	  	                  // Initialize hardware
	  	              {
	  	            	  //GPIOs
	  	            	  // CUrrent Sensors OCD pin needed to go low in reset condition after fault event
	  	            	HAL_GPIO_WritePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin, GPIO_PIN_SET);
	  	            	HAL_GPIO_WritePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin, GPIO_PIN_SET);
	  	            	HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_SET);

	  	            	  // Start PWM for delay time transfer to FPGA
	  	            	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  	            	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  	            	// Timer for clear fault  event to reset current sensor by pull down OCD pins
	  	            	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	  	            	  //DAC for  current reference
	  	            	  ///DAC1_OUT1 	- MAX1
	  	            	  //DAC1_OUT2 	- MAX2
	  	            	  //DAC2_OUT1	- MIN
	  	            	HAL_DAC_Start(&hdac1,DAC1_CHANNEL_1);
	  	            	HAL_DAC_Start(&hdac1,DAC1_CHANNEL_2);
	  	            	HAL_DAC_Start(&hdac2,DAC2_CHANNEL_1);

	  	            	//if( HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buffer, BUFFER_SIZE, DAC_ALIGN_12B_R)!= HAL_OK) printf("error");
	  	            	//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	  	            	  // FAN PWM
	  	            	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);


	  	            	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	  	            	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
	  	            	HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
	  	            	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_BUFFER, 5);
	  	            	HAL_ADC_Start_DMA(&hadc4, (uint32_t*)ADC4_DMA_BUFFER, 1);
	  	            	HAL_ADC_Start_DMA(&hadc5, (uint32_t*)ADC5_DMA_BUFFER, 1);

	  	            //	Set_PWM_DutyCycle(20);
	  	            	currentState = STATE_STANDBY;
	  	              }
	  	                  break;
	  	              case STATE_STANDBY:
	  	                  // Wait for start signal
	  	              {
	  	            	  if(START){
	  	            		  currentState = STATE_SOFT_START;
	  	            	  }

	  	              }
	  	                  break;
	  	              case STATE_SOFT_START:
	  	                  // Gradually ramp up the output
	  	              {
	  	            	  //Start timer that start ramp and pi regulation
	  	            //	HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_GPIO_Port, 0); // RESET =  1  = reset turn on
	  	            //	HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1);
	  	            	HAL_TIM_Base_Start_IT(&htim15);
	  	              }
	  	                  break;
	  	              case STATE_REGULATION:
	  	                  // Maintain output voltage/current
	  	            	  // 20khz sample time of regulators Timer 15
	  	                  {


	  	                  }
	  	                  break;
	  	              case STATE_FAULT:
	  	                  // Handle fault condition
	  	            	  // Turn off all gate drivers and stop FPGA
	  	              {
	  	            	HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 1); // RESET =  1  = reset turn on
	  	            	HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 0);
	  	            	HAL_GPIO_WritePin(NOT_RST_1_GPIO_Port,NOT_RST_1_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_2_GPIO_Port,NOT_RST_2_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_3_GPIO_Port,NOT_RST_3_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_4_GPIO_Port,NOT_RST_4_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin, 0);
	  	            	HAL_GPIO_WritePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin, 0);
	  	            	HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, 1);
	  	            	HAL_TIM_Base_Start(&htim7);


	  	            	HAL_TIM_Base_Stop_IT(&htim15);
	  	            	START = 0;
	  	              }
	  	                  break;
	  	              case STATE_SHUTDOWN:
	  	                  // Safely shut down the converter
	  	              {
	  	            	//HAL_TIM_Base_Stop_IT(&htim15);
	  	              }

	  	                  break;
	  	              default:
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 5;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_2;
  sConfig.Offset = 37;
  sConfig.OffsetSign = ADC_OFFSET_SIGN_POSITIVE;
  sConfig.OffsetSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.OffsetNumber = ADC_OFFSET_2;
  sConfig.Offset = 8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = ENABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = ENABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = ENABLE;
  hadc5.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc5.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc5.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc5.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 210;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 679;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 210;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1499;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 4;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 14999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RESET_INTERLOCK_Pin|CS_OCD_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, START_STOP_FPGA_Pin|NOT_RST_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NOT_RST_3_Pin|NOT_RST_2_Pin|NOT_RST_1_Pin|CS_OCD_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_INTERLOCK_Pin CS_OCD_1_Pin */
  GPIO_InitStruct.Pin = RESET_INTERLOCK_Pin|CS_OCD_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : INTERLOCK_Pin */
  GPIO_InitStruct.Pin = INTERLOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTERLOCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_FPGA_Pin */
  GPIO_InitStruct.Pin = RESET_FPGA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_FPGA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_STOP_FPGA_Pin NOT_RST_4_Pin */
  GPIO_InitStruct.Pin = START_STOP_FPGA_Pin|NOT_RST_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : READY_4_Pin READY_3_Pin READY_2_Pin READY_1_Pin
                           PD5 NOT_FAULT_3_Pin NOT_FAULT_2_Pin */
  GPIO_InitStruct.Pin = READY_4_Pin|READY_3_Pin|READY_2_Pin|READY_1_Pin
                          |GPIO_PIN_5|NOT_FAULT_3_Pin|NOT_FAULT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : NOT_FAULT_1_Pin CS_FAULT_2_Pin */
  GPIO_InitStruct.Pin = NOT_FAULT_1_Pin|CS_FAULT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NOT_RST_3_Pin NOT_RST_2_Pin NOT_RST_1_Pin CS_OCD_2_Pin */
  GPIO_InitStruct.Pin = NOT_RST_3_Pin|NOT_RST_2_Pin|NOT_RST_1_Pin|CS_OCD_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_FAULT_1_Pin */
  GPIO_InitStruct.Pin = CS_FAULT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CS_FAULT_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

ConverterState handle_event(ConverterState currentState, ConverterEvent event) {
    switch (currentState) {
        case STATE_INIT:
            if (event == EVENT_START)
            {
                return STATE_SOFT_START;
            }
            break;
        case STATE_STANDBY:
            if (event == EVENT_START)
            {
                return STATE_SOFT_START;
            }
            break;
        case STATE_SOFT_START:
            if (event == EVENT_FAULT)
            {
                return STATE_FAULT;
            } else if (RAMP()) {
                return STATE_REGULATION;
            }
            break;
        case STATE_REGULATION:
        	//USB_SendString("State: INIT -> RUNNING\r\n");
            if (event == EVENT_FAULT)
            {
                return STATE_FAULT;
            } else if (event == EVENT_SHUTDOWN)
            {
                return STATE_SHUTDOWN;
            }

            break;
        case STATE_FAULT:
            if (event == EVENT_CLEAR_FAULT)
            {
                return STATE_STANDBY;
            }
            break;
        case STATE_SHUTDOWN:
            if (1)
            {
                return STATE_STANDBY;
            }
            break;
        default:
            break;
    }
    return currentState; // No state change
}



void Read_ADC3(void) {
	//HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    //HAL_ADC_Start(&hadc3); // Start ADC conversion

        // Wait for the end of conversion

        /*if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) != HAL_OK) {

        }
    	HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        // Get the ADC value
        uint32_t adc_value = HAL_ADC_GetValue(&hadc3);

        // Stop the ADC to save power, if needed
        HAL_ADC_Stop(&hadc3);*/
       // float TEMPERATURE_AFTER_CONVERSION_FROM_ADC = (((adc_value/4096.0f)*3.3f) - 0.4f)/0.0195f;
       // return TEMPERATURE_AFTER_CONVERSION_FROM_ADC;

}

void Set_PWM_DutyCycle(uint32_t dutyCycle) { // dutycyle for FAN SPEED CONTROl
    if (dutyCycle > 100) dutyCycle = 100;
    uint32_t pulse = (htim4.Init.Period + 1) * dutyCycle / 100 - 1;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}
//ADC save to array and moving average
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC5)
		    {
		        uint32_t sum = 0;
		        for (int i = 0; i < MA_WINDOW_SIZE; i++)
		        {
		            sum += ADC5_DMA_BUFFER[i];
		        }
		        ADC5_MOVING_AVERAGE = (((float)sum / MA_WINDOW_SIZE)/4096)*3.3;

		       // adc5_data_ready = 1; // Set flag to indicate new data is ready

		        // Restart the DMA transfer
		        HAL_ADC_Start_DMA(hadc, (uint32_t*)ADC5_DMA_BUFFER, MA_WINDOW_SIZE);
		    }


}

void calculateMovingAverage(uint16_t src[5][MA_WINDOW_SIZE], float dst[5][MA_WINDOW_SIZE])
{
    for (int ch = 0; ch < 5; ch++)
    {
        for (int i = 0; i < MA_WINDOW_SIZE; i++)
        {
            int start = (i - MA_WINDOW_SIZE + 1) >= 0 ? (i - MA_WINDOW_SIZE + 1) : 0;
            int count = i - start + 1;
            uint32_t sum = 0;
            for (int j = start; j <= i; j++)
            {
                sum += src[ch][j];
            }
            dst[ch][i] = (float)sum / count;
        }
    }
}

void Update_PWM_Frequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency)
{
	 uint32_t timer_clock = 150000000;  // Assuming a 150 MHz timer clock
	    uint32_t prescaler = htim->Init.Prescaler + 1;
	    uint32_t period = (timer_clock / (prescaler * frequency)) - 1;

	    // Ensure period is within valid range
	    if (period > 65535) {
	        // Adjust prescaler to bring period within range
	        prescaler = (prescaler * (period / 65536)) + 1;
	        period = (timer_clock / (prescaler * frequency)) - 1;
	        __HAL_TIM_SET_PRESCALER(htim, prescaler - 1);
	    }

	    // Update the period register (ARR) with double buffering
	    __HAL_TIM_SET_AUTORELOAD(htim, period);

	    // Set the duty cycle to approximately 50%
	    uint32_t pulse = period / 2;
	    __HAL_TIM_SET_COMPARE(htim, channel, pulse);

	    // Manually generate an update event by setting the UG bit in the EGR register
	    htim->Instance->EGR = TIM_EGR_UG;


	/*uint32_t timer_clock = 150000000;  // Adjust if TIM8 is using a different clock
    uint32_t prescaler = htim->Init.Prescaler + 1;
    uint32_t period = (timer_clock / (prescaler * frequency)) - 1;

    // Update the period register
    __HAL_TIM_SET_AUTORELOAD(htim, period);

    // Restart the timer PWM generation
    HAL_TIM_PWM_Stop(htim, channel);
    HAL_TIM_PWM_Start(htim, channel);*/
}

void Update_PWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t dutyCycle)
{
    __HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}

void FillBuffer(void)
{
	 for (int i = 0; i < BUFFER_SIZE; i++)
	    {
	        dac_buffer[i] = (((1/3.3)*res12_b) + ((0.5/3.3)*res12_b) * sin(2 * M_PI * i/BUFFER_SIZE));
	    }
}

uint8_t Check_Faults()
{
	// Faults pins are from gate driver and they are active pull down
	// 4 fault pins from 4 gate driver + 2 fault pins from  2 currents sensors  = 6 pins
	if(HAL_GPIO_ReadPin(CS_FAULT_1_GPIO_Port, CS_FAULT_1_Pin) || HAL_GPIO_ReadPin(CS_FAULT_2_GPIO_Port, CS_FAULT_2_Pin)
			|| HAL_GPIO_ReadPin(NOT_FAULT_1_GPIO_Port, NOT_FAULT_1_Pin) || HAL_GPIO_ReadPin(NOT_FAULT_2_GPIO_Port, NOT_FAULT_2_Pin)
			|| HAL_GPIO_ReadPin(NOT_FAULT_3_GPIO_Port, NOT_FAULT_3_Pin) || HAL_GPIO_ReadPin(NOT_RST_4_GPIO_Port, NOT_RST_4_Pin) )
		return 0;// if there is some fault return 0

	return 1;
}

uint8_t Check_Ready()
{
	// Ready pins are from gate driver and they are active pull up
	// 4 ready pins from 4 gate drivers
	if(HAL_GPIO_ReadPin(READY_1_GPIO_Port, READY_1_Pin) && HAL_GPIO_ReadPin(READY_2_GPIO_Port, READY_2_Pin)
			&& HAL_GPIO_ReadPin(READY_3_GPIO_Port, READY_3_Pin) && HAL_GPIO_ReadPin(READY_4_GPIO_Port, READY_4_Pin) )
			return 1; // return 1 when is gate drivers ready

		return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// SOFT START RAMP REALISATION
	//  Ts 20khz
	if(htim->Instance == TIM15)
	{
		if(currentState == STATE_SOFT_START || currentState == STATE_REGULATION )
		{
		CURRENT_SENSOR1_VREF = (ADC3_DMA_BUFFER[0]/4096)*3.3;//(Low_pass_filter(ADC3_DMA_BUFFER[0], PCB_TEMP)/4096)*3.3;
		CURRENT_SENSOR2_VREF = (ADC3_DMA_BUFFER[1]/4096)*3.3;//(Low_pass_filter(ADC3_DMA_BUFFER[1], PCB_TEMP)/4096)*3.3;

		PCB_TEMP = (ADC3_DMA_BUFFER[3]/4096)*3.3;//(Low_pass_filter(ADC3_DMA_BUFFER[3], PCB_TEMP)/4096)*3.3;
		HEAT_SINK_TEMP = (ADC3_DMA_BUFFER[4]/4096)*3.3;//(Low_pass_filter(ADC3_DMA_BUFFER[4], HEAT_SINK_TEMP)/4096)*3.3;

		INPUT_VOLTAGE = (ADC3_DMA_BUFFER[2]/4096)*3.3;//((Low_pass_filter(ADC3_DMA_BUFFER[2], INPUT_VOLTAGE)/4096)*3.3-0.2)*27.1;
		OUTPUT_VOLTAGE = (ADC4_DMA_BUFFER/4096)*3.3;//((Low_pass_filter(ADC4_DMA_BUFFER, OUTPUT_VOLTAGE)/4096)*3.3-0.2)*27.1;

		IMAX2_SUM = (ADC5_MOVING_AVERAGE-1.45)*0.384; // 0.20V - -0.5A || 1.45v - 0A || 2.77V - 0.5A		0.384 A/V
		float Gv = OUTPUT_VOLTAGE*(1/INPUT_VOLTAGE);

		if(Gv<2) //CZARY
		{
			delay_tr = acos(1-Gv)/wr;
			IMIN = OUTPUT_VOLTAGE*sqrt((2-Gv)/Gv)/Z; // Negative current needed to Zero voltage switching in resonance
		} else if(Gv>=2)
		{
			delay_tr = (M_PI-acos(1/(Gv-1)))/wr;
			IMIN = 0;
		}
		//int delay_tr_freq = 1/delay_tr;
		//Update_PWM_Frequency(&htim1, TIM_CHANNEL_1, delay_tr_freq); // Set TIM1 CH1 to freq that is delay tr and send to fpga


		FAN_Drive(); // Control Fan speed dpend on two temperatures pcb and radiator

		if(currentState == STATE_SOFT_START) RAMP(); // Adding to Vramp stepping voltage to create starting ramp

		regulatorPI(&IMAX1, &Integral_I, OUTPUT_VOLTAGE, Vramp, LIM_PEAK_POS, LIM_PEAK_NEG, Kp, Ti, Ts);
		//delay_hc = (2*C_CAP*OUTPUT_VOLTAGE)/IMAX1;

		//int delay_hc_freq = 1/delay_hc;
		//Update_PWM_Frequency(&htim8, TIM_CHANNEL_1, delay_hc_freq); // Set TIM8 CH1 o freq that is delay hc and send to fpga

		IMAX2 = IMAX1 + IMAX2_SUM; // IMAX2_SUM signal from FPGA
		// IMAX1,2 each for branches to make 180 degree shift
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048+((int)IMAX1*25)); // IMAX1  1.5V is 0A;  1A is 20mV; 1 bit is 0.8mV; x A * 0.02V / 0.0008V = Value for DAC
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048+((int)IMAX2*25)); // IMAX2
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048+((int)IMIN*25)); // IMIN*/

		}
		//HAL_TIM_Base_Stop_IT(&htim15);
	}

	if (htim->Instance == TIM7)
	    {
	        // Turn off OCD pins of currents sensors to reset current sensor

	        HAL_GPIO_TogglePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin);
	        HAL_GPIO_TogglePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin);
	        HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, 0);
	        // Stop the timer
	        HAL_TIM_Base_Stop_IT(&htim7);
	    }
}

uint8_t RAMP()
{
	// RAMP Voltage to soft-start
				if((Vout-Vref)<1)
				{
					Vramp = Vout+Vref*step_size; // 1s ramp 0 to 800V
					return 0; // not finished ramp
				}
				else if((Vout-Vref)>1)
				{
					Vramp = Vref;
					return 1; // Finished ramp
				}
				return 0;
}

void regulatorPI(float *out, float *integral, float in, float in_zad, float limp, float limn, float kp, float ti, float Ts1)
{
	// Tustin transfrom of PI regulator s -> 2/T * (Z-1)/(Z+1)
    float delta;

    delta = in_zad - in; // error
    *integral = *integral + (delta + prev_delta) * (kp / ti) * Ts1 * 0.5 ; // I part
    prev_delta = delta;
    if (*integral >= limp) // limit peak positive
    {
        *integral = limp;
    }
    if (*integral <= limn)// limit peak negative
    {
        *integral = limn;
    }
    *out = (delta * kp + *integral); // Sum of P and I
    if (*out >= limp) // limit peak positive
    {
        *out = limp;
    }
    if (*out <= limn)// limit peak negative
    {
        *out = limn;
    }
}

/*void  BUTTERWORHT_FILTER(float new_sample)
{
	// Shift old samples
	    for (int i = N; i > 0; i--)
	    {
	        x[i] = x[i-1];
	        y[i] = y[i-1];
	    }

	    // Add new sample
	    x[0] = new_sample;

	    // Compute new output
	    float output = 0;
	    for (int i = 0; i < (N+1); i++)
	    {
	        output += b_z[i] * x[i];
	    }
	    for (int i = 1; i < (N+1); i++)
	    {
	        output -= a_z[i] * y[i];
	    }
	    y[0] = output / a_z[0];

	    return y[0];
}*/

float Low_pass_filter(float new_sample, float old_sample)
{
	float Low_passed_sample = 0;
	Low_passed_sample = ALPHA * new_sample + (1.0 - ALPHA) * old_sample;

	return Low_passed_sample;

}
void FAN_Drive()
{
	int duty_cycle = 20;
		float temperature = 20;
		PCB_TEMP = (PCB_TEMP-0.4)/0.0195;
		HEAT_SINK_TEMP = (HEAT_SINK_TEMP-0.5)/0.01;

		// Choose the higher of the two temperatures
		temperature = (PCB_TEMP > HEAT_SINK_TEMP) ? PCB_TEMP : HEAT_SINK_TEMP;

		// Apply a non-linear (exponential) scaling for the fan speed
		// This scales the temperature to a value between 0 and 1, then applies an exponential curve
		float normalized_temp = (temperature - 20) / 80;  // Normalizing between 0 (20°C) and 1 (100°C)
		if (normalized_temp > 1.0) normalized_temp = 1.0;
		if (normalized_temp < 0.0) normalized_temp = 0.0;

		duty_cycle = 20 + (int)(pow(normalized_temp, 3) * 79);  // Cubic curve for fan speed control

		// Enforce minimum and maximum duty cycles
		if (temperature < 20) {
		        duty_cycle = 20;
		} else if (temperature > 100) {
		    	duty_cycle = 99;
		}

		Set_PWM_DutyCycle(duty_cycle);

}



void SendUSBMessage(const char* message) {
    uint16_t len = strlen(message);
    if (len > 127) len = 127;  // Limit to buffer size
    memcpy(USB_TX_Buffer, message, len);
    USB_TX_Buffer[len] = '\0';  // Ensure null-terminated string
    uint8_t result;
    /*do {
        result = CDC_Transmit_FS(USB_TX_Buffer, len);
        if (result == USBD_OK) {
            break;
        }
      // HAL_Delay(10);  // Small delay before retrying
    } while (retry_count-- > 0);*/
    do {
            result = CDC_Transmit_FS((uint8_t*)message, len);
        } while (result == USBD_BUSY); // Retry while USB is busy


}

void ParseUSBCommand(void) {
	 if (dataReceivedFlag) {
	        if (strncmp((char*)USB_RX_Buffer, "SET_KP", 6) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_KP %f", &Kp);
	            SendUSBMessage("KP Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_Ti", 6) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_Ti %f", &Ti);
	            SendUSBMessage("Ti Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_VREF", 8) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_VREF %hu", &Vref);
	            SendUSBMessage("Vref Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_CS1_VREF", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_CS1_VREF %f", &CURRENT_SENSOR1_VREF);
	            SendUSBMessage("CURRENT_SENSOR1_VREF Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_CS2_VREF", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_CS2_VREF %f", &CURRENT_SENSOR2_VREF);
	            SendUSBMessage("CURRENT_SENSOR2_VREF Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_IMAX2_SUM", 13) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_IMAX2_SUM %f", &IMAX2_SUM);
	            SendUSBMessage("IMAX2_SUM Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_DELAY_TR", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_DELAY_TR %f", &delay_tr);
	            SendUSBMessage("delay_tr Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_DELAY_HC", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_DELAY_HC %f", &delay_hc);
	            SendUSBMessage("delay_hc Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_KP", 6) == 0) {
	            sprintf((char*)USB_TX_Buffer, "KP = %f\n", Kp);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_Ti", 6) == 0) {
	            sprintf((char*)USB_TX_Buffer, "Ti = %f\n", Ti);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_VREF", 8) == 0) {
	            sprintf((char*)USB_TX_Buffer, "Vref = %hu\n", Vref);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_CS1_VREF", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "CURRENT_SENSOR1_VREF = %f\n", CURRENT_SENSOR1_VREF);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_CS2_VREF", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "CURRENT_SENSOR2_VREF = %f\n", CURRENT_SENSOR2_VREF);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_IMAX2_SUM", 13) == 0) {
	            sprintf((char*)USB_TX_Buffer, "IMAX2_SUM = %f\n", IMAX2_SUM);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_DELAY_TR", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "delay_tr = %f\n", delay_tr);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_DELAY_HC", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "delay_hc = %f\n", delay_hc);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "DISPLAY_ALL", 11) == 0) {
	            DisplayAllVariables();

	        } else {
	            SendUSBMessage("Unknown Command\n");
	        }
	        memset(USB_RX_Buffer, 0, sizeof(USB_RX_Buffer));  // Clear buffer
	        dataReceivedFlag = 0;
	    }
}

void DisplayAllVariables(void) {
    char buffer[128];

    sprintf(buffer, "KP = %f\n", Kp);
        SendUSBMessage(buffer);

        sprintf(buffer, "Ti = %f\n", Ti);
        SendUSBMessage(buffer);

        sprintf(buffer, "Vref = %hu\n", Vref);
        SendUSBMessage(buffer);

        sprintf(buffer, "IMAX1 = %f\n", IMAX1);
        SendUSBMessage(buffer);

        sprintf(buffer, "IMAX2 = %f\n", IMAX2);
        SendUSBMessage(buffer);

        sprintf(buffer, "IMIN = %f\n", IMIN);
        SendUSBMessage(buffer);

        sprintf(buffer, "INPUT_VOLTAGE = %f\n", INPUT_VOLTAGE);
        SendUSBMessage(buffer);

        sprintf(buffer, "OUTPUT_VOLTAGE = %f\n", OUTPUT_VOLTAGE);
        SendUSBMessage(buffer);

        sprintf(buffer, "PCB_TEMP = %f\n", PCB_TEMP);
        SendUSBMessage(buffer);

        sprintf(buffer, "HEAT_SINK_TEMP = %f\n", HEAT_SINK_TEMP);
        SendUSBMessage(buffer);

        sprintf(buffer, "CURRENT_SENSOR1_VREF = %f\n", CURRENT_SENSOR1_VREF);
        SendUSBMessage(buffer);

        sprintf(buffer, "CURRENT_SENSOR2_VREF = %f\n", CURRENT_SENSOR2_VREF);
        SendUSBMessage(buffer);

        sprintf(buffer, "IMAX2_SUM = %f\n", IMAX2_SUM);
        SendUSBMessage(buffer);

        sprintf(buffer, "delay_tr = %f\n", delay_tr);
        SendUSBMessage(buffer);

        sprintf(buffer, "delay_hc = %f\n", delay_hc);
        SendUSBMessage(buffer);
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
