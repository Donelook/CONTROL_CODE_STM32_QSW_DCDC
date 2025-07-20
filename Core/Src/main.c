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
    STATE_REGULATION,
    STATE_FAULT,
    STATE_SHUTDOWN
} ConverterState;

typedef enum {
    EVENT_START,
    EVENT_FAULT,
    EVENT_CLEAR_FAULT,
    EVENT_SHUTDOWN,
} ConverterEvent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MA_WINDOW_SIZE 10  		// Moving average window
#define BUFFER_SIZE 100  		// Buffer size for DAC test - not used in final program
#define res12_b 4096
#define L_IND 0.000094  	// 94uH
#define C_CAP 0.0000000044 		// 4.4nF
#define wr 1554926				//	1/sqrt(L_IND*C_CAP)	- Omega of LC resonance
#define INV_wr 0.0000006431		//	1/wr
#define Z 146.16304718				//sqrt(L_IND/(2*C_CAP)) // impedance of inductor and two capacitor on Dren-Source MOSFETs
#define INV_Z 0.0068416745		// 1/Z
#define Ts 0.00005				// Sampling rate of control loop 20khz
#define ALPHA 0.1 // smoothing factor 0-1

#define LUT_SIZE 256

// fixed point format
#define FP_SHIFT 24
#define FP_SCALE (1 << FP_SHIFT)

// Precomputed lookup table for acos(x) for x in [0, 1] (in radians).
// Generated with: for (i=0;i<256;i++){ x = i/255.0; table[i] = acos(x); }
static const float acos_lut[LUT_SIZE] = {
     1.57079633, 1.56466040, 1.55852448, 1.55238856 ,
     1.54625263, 1.54011671, 1.53398079, 1.52784486 ,
     1.52170894, 1.51557302, 1.50943710, 1.50330117 ,
     1.49716525, 1.49102933, 1.48489340, 1.47875748 ,
     1.47262156, 1.46648563, 1.46034971, 1.45421379 ,
     1.44807786, 1.44194194, 1.43580602, 1.42967009 ,
     1.42353417, 1.41739825, 1.41126232, 1.40512640 ,
     1.39899048, 1.39285456, 1.38671863, 1.38058271 ,
     1.37444679, 1.36831086, 1.36217494, 1.35603902 ,
     1.34990309, 1.34376717, 1.33763125, 1.33149532 ,
     1.32535940, 1.31922348, 1.31308755, 1.30695163 ,
     1.30081571, 1.29467978, 1.28854386, 1.28240794 ,
     1.27627202, 1.27013609, 1.26400017, 1.25786425 ,
     1.25172832, 1.24559240, 1.23945648, 1.23332055 ,
     1.22718463, 1.22104871, 1.21491278, 1.20877686 ,
     1.20264094, 1.19650501, 1.19036909, 1.18423317 ,
     1.17809725, 1.17196132, 1.16582540, 1.15968948 ,
     1.15355355, 1.14741763, 1.14128171, 1.13514578 ,
     1.12900986, 1.12287394, 1.11673801, 1.11060209 ,
     1.10446617, 1.09833024, 1.09219432, 1.08605840 ,
     1.07992247, 1.07378655, 1.06765063, 1.06151471 ,
     1.05537878, 1.04924286, 1.04310694, 1.03697101 ,
     1.03083509, 1.02469917, 1.01856324, 1.01242732 ,
     1.00629140, 1.00015547, 0.99401955, 0.98788363 ,
     0.98174770, 0.97561178, 0.96947586, 0.96333993 ,
     0.95720401, 0.95106809, 0.94493217, 0.93879624 ,
     0.93266032, 0.92652440, 0.92038847, 0.91425255 ,
     0.90811663, 0.90198070, 0.89584478, 0.88970886 ,
     0.88357293, 0.87743701, 0.87130109, 0.86516516 ,
     0.85902924, 0.85289332, 0.84675739, 0.84062147 ,
     0.83448555, 0.82834963, 0.82221370, 0.81607778 ,
     0.80994186, 0.80380593, 0.79767001, 0.79153409 ,
     0.78539816, 0.77926224, 0.77312632, 0.76699039 ,
     0.76085447, 0.75471855, 0.74858262, 0.74244670 ,
     0.73631078, 0.73017486, 0.72403893, 0.71790301 ,
     0.71176709, 0.70563116, 0.69949524, 0.69335932 ,
     0.68722339, 0.68108747, 0.67495155, 0.66881562 ,
     0.66267970, 0.65654378, 0.65040785, 0.64427193 ,
     0.63813601, 0.63200008, 0.62586416, 0.61972824 ,
     0.61359232, 0.60745639, 0.60132047, 0.59518455 ,
     0.58904862, 0.58291270, 0.57677678, 0.57064085 ,
     0.56450493, 0.55836901, 0.55223308, 0.54609716 ,
     0.53996124, 0.53382531, 0.52768939, 0.52155347 ,
     0.51541754, 0.50928162, 0.50314570, 0.49700978 ,
     0.49087385, 0.48473793, 0.47860201, 0.47246608 ,
     0.46633016, 0.46019424, 0.45405831, 0.44792239 ,
     0.44178647, 0.43565054, 0.42951462, 0.42337870 ,
     0.41724277, 0.41110685, 0.40497093, 0.39883500 ,
     0.39269908, 0.38656316, 0.38042724, 0.37429131 ,
     0.36815539, 0.36201947, 0.35588354, 0.34974762 ,
     0.34361170, 0.33747577, 0.33133985, 0.32520393 ,
     0.31906800, 0.31293208, 0.30679616, 0.30066023 ,
     0.29452431, 0.28838839, 0.28225246, 0.27611654 ,
     0.26998062, 0.26384470, 0.25770877, 0.25157285 ,
     0.24543693, 0.23930100, 0.23316508, 0.22702916 ,
     0.22089323, 0.21475731, 0.20862139, 0.20248546 ,
     0.19634954, 0.19021362, 0.18407769, 0.17794177 ,
     0.17180585, 0.16566993, 0.15953400, 0.15339808 ,
     0.14726216, 0.14112623, 0.13499031, 0.12885439 ,
     0.12271846, 0.11658254, 0.11044662, 0.10431069 ,
     0.09817477, 0.09203885, 0.08590292, 0.07976700 ,
     0.07363108, 0.06749515, 0.06135923, 0.05522331 ,
     0.04908739, 0.04295146, 0.03681554, 0.03067962 ,
     0.02454369, 0.01840777, 0.01227185, 0.00613592 ,
};
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

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac2_ch1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
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
static void MX_TIM6_Init(void);
static void MX_CORDIC_Init(void);
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
uint16_t current_sensor1_vref = 0; // 1.5V - 0 A
uint16_t current_sensor2_vref = 0; // 1.5V - 0 A
int32_t imax1 = 0;
int32_t imax2 = 0;
uint32_t imin = 0;
uint32_t input_voltage = 0; 		// 0.2V BIAS
uint32_t pcb_temp = 0;				// Temperature of PCB
uint32_t heat_sink_temp = 0; 		// Temperature of heatsink
void FAN_Drive();
volatile static uint16_t adc3_dma_buffer[5];
volatile static uint16_t adc3_measurments[5][MA_WINDOW_SIZE];
uint16_t adc3_moving_average[5][MA_WINDOW_SIZE];


/* adc3_measurments[x]
 * 0 - Current Sensor 1 vref(1.5V Nominal)
 * 1 - Current Sensor 2 vref(1.5V Nominal)
 * 2 - Input Voltage(0V2 bias)
 * 3 - PCB temperature (MCP9700)
 * 4 - Heatsink Temprature (TMP236)
 */
int32_t vref = 48000; 			//[mV]  Reference voltage its compare to output voltage
//uint32_t step_size = 2000;
int32_t output_voltage = 21000; 	// Measured voltage 0.2V BIAS
int32_t vout = 21000; 				//Voltage comparing to vref
int32_t Vramp = 21000; 			// ramp voltage
volatile static uint16_t adc4_dma_buffer[2];
volatile static uint16_t adc4_measurments;
int32_t RAMP(int32_t Vout, int32_t Vref, int32_t Ramp_ratio, float period_loop);
uint8_t RAMP_FINISHED = 0;
void regulatorPI(int32_t *out, int32_t *integral, int32_t in, int32_t in_zad, int32_t limp, int32_t limn, float kp, float ti, float Ts1);
int32_t prev_out;
int32_t delta;
float delay_tr = 1e-7; // DELAY/DEADTIME after first stage inductor  positive ramp
float delay_hc = 1e-7; // DELAY/DEADTIME after second stage inductor negative ramp
int delay_tr_freq = 1000000;
int delay_hc_freq = 1000000;
float delay_tr_freq_ACC = 1;
float delay_hc_freq_ACC = 1;
float Gv = 1;
float Gv_prev = 1;
//Filter butterworth 500khz sampleing rate 200khz cutoff
/*#define N 4 // Order of the filter
float x[N+1] = {0}; // Input samples
float y[N+1] = {0}; // Output samples
float b_z[N+1] = { 0.4328,   1.7314,   2.5971,   1.7314,   0.4328};
float a_z[N+1] = { 1.0000,   2.3695,   2.3140,   1.0547,   0.1874 };
void  BUTTERWORHT_FILTER(float new_sample);*/
float b[2] = {0.13575525, 0.13575525};
float a = 0.7284895;
float yfilter[2] = {1,1};
//Simple low pass filter
float Low_pass_filter(float new_sample, float old_sample, float old_sample_n1, float old_passed);


/* ADC4_MEASRUMENTS[X]
 * 0 - Output Voltage(0V2 bias)
 *
 */



/* ADC5_MEASRUMENTS[X]
 * 0 - imax2_sum
 *
 */
int16_t imax2_sum = 0;
volatile static uint16_t adc5_dma_buffer[MA_WINDOW_SIZE];
volatile static uint16_t adc5_measurments[MA_WINDOW_SIZE];
uint16_t adc_moving_average = 0;

// USB INTERFACE DISPLAY AND SET
// Buffer to hold incoming data
uint8_t USB_RX_Buffer[64];
uint8_t USB_TX_Buffer[128];
void SendUSBMessage(const char* message);
void ParseUSBCommand(void);
void DisplayAllVariables(void);
volatile uint8_t dataReceivedFlag = 0; // Flags to indicate new data received


//Regulator PI of voltage
float Kp = 0.01; 			// Proportional part of PI
float Ti = 5e-5; 			// Integral part of PI
int32_t LIM_PEAK_POS = 8000; 	// Positive limit for PI regulator [mA]
int32_t LIM_PEAK_NEG = 0; 	// Negative limit for PI regulator [mA]
int32_t Integral_I = 0;		// Integral part of PI
int32_t prev_delta = 0; 		// buffer  error n-1



uint8_t start_program = 0;
uint8_t stop_program = 0;
uint8_t clear_fault = 0;
ConverterEvent event = EVENT_SHUTDOWN;
ConverterState currentState = STATE_INIT;

int duty_cycle = 20; // start duty cycle for FAN speed

uint8_t checkfaults = 0;
uint8_t checkreads = 0;

float Imin_Factor = 1;
int once = 0;
uint8_t interlock = 0;
uint32_t sythick1 = 0;
uint32_t sythick2 = 0;
uint8_t flag_control = 0;
uint32_t input_vol = 21000;
uint32_t output_vol = 21000;
uint32_t input_vol_x_n1 = 1;
uint32_t input_vol_y_n1 = 1;
uint32_t output_vol_x_n1 = 1;
uint32_t output_vol_y_n1 = 1;


// Convert float to Q8.24 fixed-point
static inline int32_t float_to_fixed(float x);

// Convert Q8.24 fixed-point back to float (if needed)
static inline float fixed_to_float(int32_t x);

// CORDIC
int32_t float_to_integer(float in, int scaling_factor, uint8_t bits);
float integer_to_float(int32_t result_cordic_integer, int squarted_scaling_factor, int8_t mode, uint8_t bits);
static inline float approx_acos(float x);
static inline float approx_acos2(float x);
static inline float approx_acos3(float x);

CORDIC_HandleTypeDef hcordic;        // CORDIC handle (assume it's declared globally or in main)
CORDIC_ConfigTypeDef sCordicConfig;  // config structure
int32_t result_q31 = 1;
int16_t result_q16 = 1;
int32_t value_cordic = 1;

int32_t start_ticks = 0;
int32_t stop_ticks = 0;
int32_t elapsed_ticks = 0;
int32_t elapsed_ticks2 = 0;
int32_t elapsed_ticks3 = 0;
int32_t elapsed_ticks4 = 0;
int32_t elapsed_ticks5 = 0;
float resultcordic = 1;
float obl = 1;
int32_t cordic_input = 1;
float resultcordic_float = 1;
float acos1, acos2, acos3, acos4, acos5;

float lookup_acos(float x) ;
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
  MX_TIM6_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */

  sCordicConfig.Function   = CORDIC_FUNCTION_SQUAREROOT;       /* Compute sine (and cosine) */
  sCordicConfig.Precision  = CORDIC_PRECISION_6CYCLES;    /* Maximum precision (24 iterations) */
  sCordicConfig.Scale      = CORDIC_SCALE_0;              /* No additional scaling */
  sCordicConfig.NbWrite    = CORDIC_NBWRITE_1;            /* One input (angle); implicit modulus = 1 */
  sCordicConfig.NbRead     = CORDIC_NBREAD_1;             /* Two outputs (sine and cosine) */
  sCordicConfig.InSize     = CORDIC_INSIZE_32BITS;        /* 32-bit input (Q1.31 format) */
  sCordicConfig.OutSize    = CORDIC_OUTSIZE_32BITS;       /* 32-bit output (Q1.31 format) */


  if (HAL_CORDIC_Configure(&hcordic, &sCordicConfig) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
  	  //start_ticks = SysTick->VAL;
 /* Gv = 1.001;
  float funk = ((2-Gv)/Gv);
  	 // beforea = float_to_integer(obl, 100, 32);
  	 // HAL_CORDIC_Calculate(&hcordic, &beforea, &result_q31, 1, 100);//sqrt((2-Gv)/Gv)/Z)
  	 // resultcordic = approx_acos(1.5);//approx_acos((1-Gv))/wr;//sqrt(obl);
  	  //resultcordic = integer_to_float(result_q31, 10, 1, 32);
  start_ticks = SysTick->VAL;
  	  	acos4 =	acos(funk);
  	  	    	  	  	  stop_ticks = SysTick->VAL;

  	  	    	  	  	  elapsed_ticks4 = start_ticks-stop_ticks;
  	  	    	  	  	  funk = 0;
  	 start_ticks = SysTick->VAL;
  	//acos1 = approx_acos(funk);
  	funk = ((2-Gv)/Gv)/Z;
  	  stop_ticks = SysTick->VAL;

  	  elapsed_ticks = start_ticks-stop_ticks;
  	funk = 0;
  	 start_ticks = SysTick->VAL;
  	//acos2 = approx_acos2(funk);
  	funk = ((2-Gv)/Gv)*(1/Z);
  	  	  stop_ticks = SysTick->VAL;

  	  	  elapsed_ticks2 = start_ticks-stop_ticks;

  	  	 start_ticks = SysTick->VAL;
  	  	acos3 =	approx_acos3(funk);
  	  	  	  stop_ticks = SysTick->VAL;

  	  	  	  elapsed_ticks3 = start_ticks-stop_ticks;

  	  	  start_ticks = SysTick->VAL;
  	  	   	  	acos5 =	lookup_acos(funk);
  	  	   	  	  	  stop_ticks = SysTick->VAL;

  	  	   	  	  	  elapsed_ticks5 = start_ticks-stop_ticks;
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//event = EVENT_START;
  //USB_SendString("PRZED WHILEM\r\n");


  while (1)
  {


	  	  	  	  checkfaults = Check_Faults();

	  	  	  	  if (dataReceivedFlag) {
	  	  	  	      // Process the data
	  	  	  	      ParseUSBCommand();  // Function to handle the received command

	  	  	  	      // Clear the flag after processing
	  	  	  	      dataReceivedFlag = 0;
	  	  	  	  }

	  	  	  	  interlock = HAL_GPIO_ReadPin(INTERLOCK_GPIO_Port, INTERLOCK_Pin);

	  	          if (interlock &&  start_program && !(checkfaults)) {
	  	        	//USB_SendString("State: EVENT start_program \r\n");
	  	              event = EVENT_START;
	  	              start_program = 0;
	  	          }else if (interlock &&  stop_program && !(checkfaults)) {
		  	        	//USB_SendString("State: EVENT start_program \r\n");
		  	              event = EVENT_SHUTDOWN;
		  	              stop_program = 0;
		  	          }
	  	          else if (clear_fault) {
	  	        	  /* clear fault condition */
	  	              event = EVENT_CLEAR_FAULT;
	  	          }

	  	         if (!interlock || checkfaults /* fault condition */)
	  	       	 {
	  	       	  	event = EVENT_FAULT;
	  	       	 }

	  	          // Handle the event and update the state
	  	          currentState = handle_event(currentState, event);

	  	          // Perform actions based on the current state
	  	          switch (currentState) {
	  	              case STATE_INIT:
	  	                  // Initialize hardware
	  	              {
	  	            	//GPIOs
	  	            	//CUrrent Sensors OCD pin needed to go low in reset condition after fault event
	  	            	//HAL_GPIO_WritePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin, GPIO_PIN_SET);
	  	            	//HAL_GPIO_WritePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin, GPIO_PIN_SET);
	  	            	HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 1); // RESET =  1  = reset turn on IMPORTANT!! WAZNE!!!

	  	            	HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, GPIO_PIN_RESET); // STOP

	  	            	HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_SET);
	  	            	HAL_Delay(1000);
	  	            	HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_RESET);
	  	            	  // Start PWM for delay time transfer to FPGA
	  	            	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  	            	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  	            	// Timer for clear fault  event to reset current sensor by pull down OCD pins
	  	            	//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	  	            	  //DAC for  current reference
	  	            	  ///DAC1_OUT1 	- MAX1
	  	            	  //DAC1_OUT2 	- MAX2
	  	            	  //DAC2_OUT1	- MIN
	  	            	HAL_DAC_Start(&hdac1,DAC1_CHANNEL_1);
	  	            	HAL_DAC_Start(&hdac1,DAC1_CHANNEL_2);
	  	            	HAL_DAC_Start(&hdac2,DAC2_CHANNEL_1);

	  	            	//if( HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buffer, BUFFER_SIZE, DAC_ALIGN_12B_R)!= HAL_OK) printf("error");
	  	            	//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	  	            	  // FAN PWM and 5s timer6 for check temperature and change duty cycle
	  	            	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  	            	HAL_TIM_Base_Start_IT(&htim6);

	  	            	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	  	            	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
	  	            	HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

	  	            	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_dma_buffer, 5);
	  	            	HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc4_dma_buffer, 2);
	  	            	HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_dma_buffer, 10);

	  	            	Set_PWM_DutyCycle(20);

	  	            	current_sensor1_vref = adc3_dma_buffer[0];// reference for imax imin
	  	            	current_sensor2_vref = adc3_dma_buffer[1];// reference for imax imin

	  	            	currentState = STATE_STANDBY;
	  	              }
	  	                  break;
	  	              case STATE_STANDBY:
	  	                  // Wait for start_program signal
	  	              {
	  	            	//HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 0); // RESET =  0  = reset turn off
	  	            	//HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1); // START FPGA DANCE
	  	            	checkreads = Check_Ready();
	  	            	  if(start_program && interlock &&  !(checkfaults)   && checkreads){
	  	            		  currentState = STATE_REGULATION;//STATE_SOFT_START;

		  	            	  /*if(once == 0){
		  	            	  //Start timer that start_program ramp and pi regulation
		  	            	HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 0); // RESET =  0  = reset turn off
		  	            	//HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1); // START FPGA DANCE
		  	            	HAL_TIM_Base_Start_IT(&htim15); // START TIM15 THATS IS MAIN CONTROL LOOP
		  	            	//HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1); // START FPGA DANCE
		  	            	RAMP_FINISHED = 0;
		  	            	//once = 1;
		  	            	  }*/
	  	            	  }

	  	              }
	  	                  break;

	  	              case STATE_REGULATION:
	  	                  // Maintain output voltage/current
	  	            	  // 20khz sample time of regulators Timer 15
	  	                  {
	  	                	if(once == 0){
	  	                		//Start timer that start_program ramp and pi regulation
	  	                		HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 0); // RESET =  0  = reset turn off
	  	                		HAL_TIM_Base_Start_IT(&htim15); // START TIM15 THATS IS MAIN CONTROL LOOP
	  	                		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  	                		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  	                		 }
	  	                	  if(flag_control)
	  	                	  {
	  	                		// start_ticks = SysTick->VAL;

	  	                		  	  	input_vol = (int32_t)Low_pass_filter(input_voltage, input_vol, input_vol_x_n1, input_vol_y_n1); //input_voltage;
	  	                		  	  	output_vol = (int32_t)Low_pass_filter(output_voltage, output_vol, output_vol_x_n1, output_vol_y_n1); //output_voltage;
	  	                		  	  	 //yfilter[1] = a*yfilter[];




	  	                		  	  	Gv = (float)output_vol/(float)input_vol;//output_voltage/input_voltage;
	  	                		  	  	//if(abs(Gv-Gv_prev)>=0.02)
	  	                		  	  	//{
	  	                				if(Gv<2) //CZARY
	  	                				{

	  	                					delay_tr = approx_acos2((1-Gv))*INV_wr;
	  	                					 // start_ticks = SysTick->VAL;

	  	                					cordic_input = float_to_integer(((2-Gv)/Gv), 100, 32);
	  	                					HAL_CORDIC_Calculate(&hcordic, &cordic_input, &result_q31, 1, 100);//sqrt((2-Gv)/Gv))
	  	                					resultcordic = integer_to_float(result_q31, 10, 1, 32); // result of sqrt((((2-Gv)/Gv)) ) in float


	  	                					imin = (int)(Imin_Factor*output_vol*resultcordic*INV_Z); //[mA] Negative current needed to Zero voltage switching in resonance

	  	                					if(imin>4000) imin = 4000;
	  	                				} else if(Gv >= 2)
	  	                				{
	  	                					delay_tr = (M_PI-approx_acos2((1/(Gv-1)))) * INV_wr;
	  	                					imin = 0;
	  	                				}
	  	                		  	  	//}
	  	                		  		//Gv_prev = Gv;
	  	                				if(/*once == 0*/delay_tr < 0.001 /*&& RAMP_FINISHED == 1*/){

	  	                					delay_tr_freq = (int)(1/delay_tr);

	  	                					if(delay_tr_freq>20000000) delay_tr_freq = 20000000;//10Mhz

	  	                					if(abs(delay_tr_freq_ACC-delay_tr_freq) >= 20000) {
	  	                						if(RAMP_FINISHED == 0) Update_PWM_Frequency(&htim1, TIM_CHANNEL_1, delay_tr_freq); // Set TIM1 CH1 to freq that is delay tr and send to fpga
	  	                						delay_tr_freq_ACC = delay_tr_freq;
	  	                					}
	  	                				}

	  	                				if(/*once == 0 output_vol> 47000 && RAMP_FINISHED == 1 */ imax1 > 0 /*&& output_vol> 47000 */ ){

	  	            	  	                		delay_hc = (float)(((float)C_CAP*output_vol) * (float)(1/(float)imax1));
	  	                					  	    delay_hc_freq = (int)(1/delay_hc);

	  	                					  	   if(delay_hc_freq>20000000) delay_hc_freq = 20000000;//10Mhz jakis problem

	  	                					  	   if(abs(delay_hc_freq_ACC-delay_hc_freq) >= 100000) {
	  	                					  		 start_ticks = SysTick->VAL;

	  	                					  	     Update_PWM_Frequency(&htim8, TIM_CHANNEL_2, delay_hc_freq); // Set TIM8 CH1 o freq that is delay hc and send to fpga

	  	                					  	    stop_ticks = SysTick->VAL;
	  	                					  	    elapsed_ticks = start_ticks-stop_ticks;

	  	                					  	    delay_hc_freq_ACC = delay_hc_freq;
	  	                					  	   }
	  	                				}
	  	                				if(RAMP_FINISHED == 0) Vramp = RAMP(Vramp, vref, 160000, Ts); // Adding to Vramp stepping voltage to create starting ramp

	  	                				if (Vramp > 0 ) regulatorPI(&imax1, &Integral_I, output_vol, Vramp, LIM_PEAK_POS, LIM_PEAK_NEG, Kp, Ti, Ts);



	  	                				imax2 =  imax1 ;//+ imax2_sum;//

	  	                				if(once == 0){
	  	                					//HAL_Delay(500);
	  	                					HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1); // START FPGA DANCE
	  	                					once = 1;
	  	                				}

	  	                				flag_control = 0;

	  	                	  }

	  	                  }
	  	                  break;
	  	              case STATE_FAULT:
	  	                  // Handle fault condition
	  	            	  // Turn off all gate drivers and stop FPGA
	  	              {
	  	            	HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, GPIO_PIN_RESET); // STOP drives mosfet etc

	  	            	//HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 1); // RESET =  1  = reset turn on IMPORTANT!! WAZNE!!!


	  	            	HAL_TIM_Base_Stop_IT(&htim15);

	  	            	HAL_GPIO_WritePin(NOT_RST_1_GPIO_Port,NOT_RST_1_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_2_GPIO_Port,NOT_RST_2_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_3_GPIO_Port,NOT_RST_3_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(NOT_RST_4_GPIO_Port,NOT_RST_4_Pin, GPIO_PIN_RESET);

	  	            	HAL_GPIO_WritePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin, GPIO_PIN_RESET);

	  	            	HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_SET);

	  	            	HAL_TIM_Base_Start(&htim7); // timer for reset OCD and INTERLOCK reset turn off


	  	            	once = 0;
	  	            	start_program = 0;
	  	            	currentState = STATE_SHUTDOWN;
	  	              }
	  	                  break;
	  	              case STATE_SHUTDOWN:
	  	                  // Safely shut down the converter
	  	              {
	  	            	HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, GPIO_PIN_RESET);
	  	            	HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 1); // RESET =  1  = reset turn on IMPORTANT!! WAZNE!!!
	  	            	HAL_TIM_Base_Stop_IT(&htim15);
	  	            	//HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, 0); // RESET =  1  = reset turn on IMPORTANT!! WAZNE!!!
	  	            	RAMP_FINISHED = 0;
						once = 0;
	  	            	imax1 = 1;
						imax2 = 1;
						imax2_sum = 1;
						vout = 1;
						Vramp = 1;
						delay_tr = 1;
						delay_hc = 1;
						Gv = 1;
						Integral_I = 1;
						prev_delta = 1;
						input_vol = 1;
						input_voltage =1;
						output_vol = 1;
						output_voltage =1;
						input_vol_x_n1 = 1;
						input_vol_y_n1 = 1;
						output_vol_x_n1 = 1;
						output_vol_y_n1 = 1;
						imin = 1;
						HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
						HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  	            	currentState = STATE_STANDBY;
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
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 38;
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
  sConfig.OffsetNumber = ADC_OFFSET_2;
  sConfig.Offset = 37;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.OffsetNumber = ADC_OFFSET_3;
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
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

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
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
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
  htim6.Init.Prescaler = 11549;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 64934;
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
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE END TIM6_Init 2 */

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
  htim7.Init.Prescaler = 14999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
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
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_2);
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
  HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_FPGA_GPIO_Port, RESET_FPGA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, NOT_RST_2_Pin|CS_OCD_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NOT_RST_4_GPIO_Port, NOT_RST_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NOT_RST_3_GPIO_Port, NOT_RST_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NOT_RST_1_GPIO_Port, NOT_RST_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : NOT_FAULT_1_Pin CS_FAULT_2_Pin NOT_FAULT_2_Pin CS_FAULT_1_Pin */
  GPIO_InitStruct.Pin = NOT_FAULT_1_Pin|CS_FAULT_2_Pin|NOT_FAULT_2_Pin|CS_FAULT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_INTERLOCK_Pin */
  GPIO_InitStruct.Pin = RESET_INTERLOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_INTERLOCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_OCD_2_Pin */
  GPIO_InitStruct.Pin = CS_OCD_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_OCD_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : READY_3_Pin */
  GPIO_InitStruct.Pin = READY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(READY_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERLOCK_Pin NOT_FAULT_4_Pin */
  GPIO_InitStruct.Pin = INTERLOCK_Pin|NOT_FAULT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_FPGA_Pin */
  GPIO_InitStruct.Pin = RESET_FPGA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_FPGA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_STOP_FPGA_Pin */
  GPIO_InitStruct.Pin = START_STOP_FPGA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(START_STOP_FPGA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : READY_2_Pin */
  GPIO_InitStruct.Pin = READY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(READY_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NOT_RST_2_Pin CS_OCD_1_Pin */
  GPIO_InitStruct.Pin = NOT_RST_2_Pin|CS_OCD_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : NOT_RST_4_Pin */
  GPIO_InitStruct.Pin = NOT_RST_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NOT_RST_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : READY_4_Pin */
  GPIO_InitStruct.Pin = READY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(READY_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NOT_RST_3_Pin */
  GPIO_InitStruct.Pin = NOT_RST_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NOT_RST_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : READY_1_Pin PD5 NOT_FAULT_3_Pin */
  GPIO_InitStruct.Pin = READY_1_Pin|GPIO_PIN_5|NOT_FAULT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : NOT_RST_1_Pin */
  GPIO_InitStruct.Pin = NOT_RST_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NOT_RST_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

ConverterState handle_event(ConverterState currentState, ConverterEvent event) {
    switch (currentState) {
        case STATE_INIT:
            if (event == EVENT_START)
            {

            	currentState = STATE_STANDBY;
                return STATE_STANDBY;
            }
            break;

        case STATE_STANDBY:
            if (event == EVENT_START)
            {
            	currentState = STATE_REGULATION;
                return STATE_REGULATION;
            }
            break;

        case STATE_REGULATION:
        	//USB_SendString("State: INIT -> RUNNING\r\n");
            if (event == EVENT_FAULT)
            {
            	currentState = STATE_FAULT;
                return STATE_FAULT;

            } else if (event == EVENT_SHUTDOWN)
            {
            	currentState = STATE_SHUTDOWN;
                return STATE_SHUTDOWN;
            }

            break;

        case STATE_FAULT:
            if (event == EVENT_CLEAR_FAULT)
            {
            	currentState = STATE_STANDBY;
                return STATE_STANDBY;
            }
            break;

        case STATE_SHUTDOWN:
            if (event == EVENT_START)
            {
            	once = 0;
            	currentState = STATE_REGULATION;
                return STATE_REGULATION;
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
				if(flag_control){
		        uint32_t sum = 0;
		        for (int i = 0; i < MA_WINDOW_SIZE; i++)
		        {
		            sum += adc5_dma_buffer[i];
		        }
		        adc_moving_average = ((sum / MA_WINDOW_SIZE)*3300)/4096;
		        imax2_sum=(adc_moving_average-1450)*0.384;
		        if(imax2_sum<-1000) imax2_sum = -1000;
		        if(imax2_sum>1000) imax2_sum = 1000;
				}
		       // adc5_data_ready = 1; // Set flag to indicate new data is ready

		        // Restart the DMA transfer
		       // HAL_ADC_Start_DMA(hadc, (uint32_t*)adc5_dma_buffer, MA_WINDOW_SIZE);
		    }


}

void calculateMovingAverage(uint16_t src[5][MA_WINDOW_SIZE], float dst[5][MA_WINDOW_SIZE])
{
    for (int ch = 0; ch < 5; ch++)
    {
        for (int i = 0; i < MA_WINDOW_SIZE; i++)
        {
            int start_program = (i - MA_WINDOW_SIZE + 1) >= 0 ? (i - MA_WINDOW_SIZE + 1) : 0;
            int count = i - start_program + 1;
            uint32_t sum = 0;
            for (int j = start_program; j <= i; j++)
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
	if(HAL_GPIO_ReadPin(CS_FAULT_1_GPIO_Port, CS_FAULT_1_Pin) && HAL_GPIO_ReadPin(CS_FAULT_2_GPIO_Port, CS_FAULT_2_Pin)
			&& HAL_GPIO_ReadPin(NOT_FAULT_1_GPIO_Port, NOT_FAULT_1_Pin) && HAL_GPIO_ReadPin(NOT_FAULT_2_GPIO_Port, NOT_FAULT_2_Pin)
			&& HAL_GPIO_ReadPin(NOT_FAULT_3_GPIO_Port, NOT_FAULT_3_Pin) && HAL_GPIO_ReadPin(NOT_FAULT_4_GPIO_Port, NOT_FAULT_4_Pin) )
		return 0; // if all pins is 1 then all is ready, there is not faults then return 0

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
	// SOFT start_program RAMP REALISATION
	//  Ts 20khz
	if(htim->Instance == TIM15)
	{
		//sythick1 =  HAL_GetTick();
		if(currentState == STATE_REGULATION )
		{
		//current_sensor1_vref = adc3_dma_buffer[0]*3300/4096;//(Low_pass_filter(adc3_dma_buffer[0], pcb_temp)/4096)*3.3;
		//current_sensor2_vref = adc3_dma_buffer[1]*3300/4096;//(Low_pass_filter(adc3_dma_buffer[1], pcb_temp)/4096)*3.3;

		input_voltage = (int)((((adc3_dma_buffer[2])*0.8056)-200)*18.81);//[mV]		((Low_pass_filter(adc3_dma_buffer[2], input_voltage)/4096)*3.3-0.2)*27.1;
		output_voltage = (int)((((adc4_dma_buffer[1])*0.8056)-200)*18.81);//[mV] 		((Low_pass_filter(adc4_dma_buffer, output_voltage)/4096)*3.3-0.2)*27.1;

		//imax2_sum = //(adc_moving_average-1450)*0.384; //[mA] 0.20V - -0.5A || 1.45v - 0A || 2.77V - 0.5A		0.384 A/V
		/*Gv = (float)output_voltage/(float)input_voltage;//output_voltage/input_voltage;

		if(Gv<2) //CZARY
		{
			delay_tr = acos(1-Gv)/wr;
			imin = (int)(Imin_Factor*output_voltage*sqrt((2-Gv)/Gv)/Z); //[mA] Negative current needed to Zero voltage switching in resonance
			if(imin>500) imin = 500;
		} else if(Gv>=2)
		{
			delay_tr = (M_PI-acos(1/(Gv-1)))/wr;
			imin = 0;
		}
		if(delay_tr<0.001){
		int delay_tr_freq = (int)(1/delay_tr);
		if(delay_tr_freq>20000000) delay_tr_freq = 15000000;//10Mhz
		if(once == 0) Update_PWM_Frequency(&htim1, TIM_CHANNEL_1, 221454); // Set TIM1 CH1 to freq that is delay tr and send to fpga
		}

		if(currentState == STATE_SOFT_START) RAMP(); // Adding to Vramp stepping voltage to create starting ramp

		regulatorPI(&imax1, &Integral_I, output_voltage, Vramp, LIM_PEAK_POS, LIM_PEAK_NEG, Kp, Ti, Ts);

		if(output_voltage>40000)
		{
		delay_hc = (2*C_CAP*output_voltage)/imax1;
		int delay_hc_freq = (int)(1/delay_hc);
		if(delay_hc_freq>20000000) delay_hc_freq = 15000000;//10Mhz jakis problem
		if(once == 0) Update_PWM_Frequency(&htim8, TIM_CHANNEL_2, 7100000); // Set TIM8 CH1 o freq that is delay hc and send to fpga
		}

		imax2 = imax1 + imax2_sum; // imax2_sum signal from FPGA
		// imax1,2 each for branches to make 180 degree shift*/
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, current_sensor1_vref+((int32_t)imax1*0.025)); // imax1  1.5V is 0A;  1A is 20mV; 1 bit is 0.8mV; imax[mA]*0.02 [V/A]/0.8[mV] = Value for DAC
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, current_sensor2_vref+((int32_t)imax2*0.025)); // imax2
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, current_sensor1_vref-((int32_t)imin*0.25)); // imin uzyto tutaj wzmacniacza 10x dla sygnalu z sensora pradu wiec ma wzmocnienie 200mv/A a nie 20mv/a

		}
		//HAL_TIM_Base_Stop_IT(&htim15);
		//if(once == 0){
		//	HAL_Delay(10);
		//HAL_GPIO_WritePin(START_STOP_FPGA_GPIO_Port, START_STOP_FPGA_Pin, 1); // START FPGA DANCE
		//once = 1;
		//}

		//sythick2 =  HAL_GetTick() - sythick1;
		flag_control = 1;
	}

	if (htim->Instance == TIM6) // 5 sec period
		    {
		//if(currentState == STATE_SOFT_START || currentState == STATE_REGULATION )
				//{
				FAN_Drive(); // Control Fan speed dpend on two temperatures pcb and radiator
				//}
		    }


	if (htim->Instance == TIM7)
	    {
	        // Turn off OCD pins of currents sensors to reset current sensor 1us

	        HAL_GPIO_TogglePin(CS_OCD_1_GPIO_Port, CS_OCD_1_Pin);
	        HAL_GPIO_TogglePin(CS_OCD_2_GPIO_Port, CS_OCD_2_Pin);
	        HAL_GPIO_WritePin(RESET_INTERLOCK_GPIO_Port, RESET_INTERLOCK_Pin, GPIO_PIN_RESET);
	        // Stop the timer
	        HAL_TIM_Base_Stop_IT(&htim7);
	    }


}

int32_t RAMP(int32_t Vout, int32_t Vref, int32_t Ramp_ratio, float period_loop)
{
	// RAMP Voltage to soft-start
				if(((int32_t)Vref - (int32_t)Vout) > 10)
				{

					Vout = (int32_t)(Vout + Ramp_ratio * period_loop); // 20khz loop - preferred 0.1V/Ts voltage ramp   that mean ramp ratio = 2000
					//RAMP_FINISHED = 0;
				}
				else if(((int32_t)Vref - (int32_t)Vout) < -10) // 100 = 100mV
				{
					Vout = (int32_t)(Vout - Ramp_ratio * period_loop);
				}
				if(Vout >= Vref-50)
				{
					Vout = Vref; // 48V
					RAMP_FINISHED = 1;
					//currentState = STATE_REGULATION;
				}

				return Vout;
}

void regulatorPI(int32_t *out, int32_t *integral, int32_t in, int32_t in_zad, int32_t limp, int32_t limn, float kp, float ti, float Ts1)
{
	// Tustin transfrom of PI regulator s -> 2/T * (Z-1)/(Z+1)


    delta = in_zad - in; // error
    *integral = (*integral + (int32_t)((delta + prev_delta) * ((kp / ti) * Ts1 * 0.5f))) ; // I part
    prev_delta = delta;
    prev_out = *out;
    if (*integral >= limp) // limit peak positive
    {
        *integral = limp;
    }
    if (*integral <= limn)// limit peak negative
    {
        *integral = limn;
    }
    *out = ((int32_t)((float)delta*kp) + *integral); // Sum of P and I
    if (*out >= limp) // limit peak positive
    {
        *out = limp;
    }
    if (*out <= limn)// limit peak negative
    {
        *out = limn;
    }
   // if(abs((*out - prev_out)) <= 50) // histeresis to probably prevent jitter must be checked
  //  {
   // 	*out = prev_out;
   // }
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

float Low_pass_filter(float new_sample, float old_sample, float old_sample_n1, float old_passed)
{

	//Low_passed_sample = (float)ALPHA * new_sample + (1.0 - ALPHA) * old_sample;
	//(1-ALPHA)*0.5*(new_sample+old_sample_n1+ALPHA*old_passed);//(float)ALPHA * new_sample + (1.0 - ALPHA) * old_sample;
	return ((1-ALPHA) * old_sample + ALPHA * new_sample) ;//(float)ALPHA * new_sample + (1.0 - ALPHA) * old_sample;

}
void FAN_Drive()
{ //@ToDo poprawic ogarnac zeby a intach bylo
		pcb_temp = (adc3_dma_buffer[3]*3300)/4096;//(Low_pass_filter(adc3_dma_buffer[3], pcb_temp)/4096)*3.3;
		heat_sink_temp = (adc3_dma_buffer[4]*3300)/4096;//(Low_pass_filter(adc3_dma_buffer[4], heat_sink_temp)/4096)*3.3;

		uint32_t temperature = 20;

		pcb_temp = (pcb_temp-400)/20;
		heat_sink_temp = (heat_sink_temp-500)/10;
		// Choose the higher of the two temperatures
		temperature = (pcb_temp > heat_sink_temp) ? pcb_temp : heat_sink_temp;
		// Apply a non-linear (exponential) scaling for the fan speed
		// This scales the temperature to a value between 0 and 1, then applies an exponential curve
		uint32_t normalized_temp = ((temperature - 20) *100)/ 80;  // Normalizing between 0 (20°C) and 1000 (100°C)
		if (normalized_temp > 100) normalized_temp = 100;
		if (normalized_temp < 0) normalized_temp = 0;

		duty_cycle = 20 + ((int)(pow(normalized_temp, 3) * 79))/1000000;  // Cubic curve for fan speed control

		// Enforce minimum and maximum duty cycles
		if (temperature < 20) {
		        duty_cycle = 20;
		} else if (temperature > 100) {
		    	duty_cycle = 99;
		}

		//Set_PWM_DutyCycle(duty_cycle);

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
	            sscanf((char*)USB_RX_Buffer, "SET_VREF %hu", &vref);
	            SendUSBMessage("vref Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_CS1_VREF", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_CS1_VREF %d", &current_sensor1_vref);
	            SendUSBMessage("current_sensor1_vref Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_CS2_VREF", 12) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_CS2_VREF %d", &current_sensor2_vref);
	            SendUSBMessage("current_sensor2_vref Updated\n");

	        } else if (strncmp((char*)USB_RX_Buffer, "SET_IMAX2_SUM", 13) == 0) {
	            sscanf((char*)USB_RX_Buffer, "SET_IMAX2_SUM %d", &imax2_sum);
	            SendUSBMessage("imax2_sum Updated\n");

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
	            sprintf((char*)USB_TX_Buffer, "vref = %hu\n", vref);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_CS1_VREF", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "current_sensor1_vref = %f\n", current_sensor1_vref);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_CS2_VREF", 12) == 0) {
	            sprintf((char*)USB_TX_Buffer, "current_sensor2_vref = %f\n", current_sensor2_vref);
	            SendUSBMessage((char*)USB_TX_Buffer);

	        } else if (strncmp((char*)USB_RX_Buffer, "GET_IMAX2_SUM", 13) == 0) {
	            sprintf((char*)USB_TX_Buffer, "imax2_sum = %f\n", imax2_sum);
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

// CORDIC
int32_t float_to_integer(float in, int scaling_factor, uint8_t bits){

	int32_t acc;
	if(bits == 32){
	if(in <= 1){
		 acc = (uint32_t)(in*scaling_factor*2147483648);
	}

	if(in > 1){
		 acc = (uint32_t)((in/scaling_factor)*2147483648);
		}
}
	if(bits == 16){
		if(in <= 1){
			 acc = (uint32_t)(in*scaling_factor*32767);
		}

		if(in > 1){
			 acc = (uint32_t)((in/scaling_factor)*32767);
			}
	}


	return acc;
}

float integer_to_float(int32_t result_cordic_integer, int squarted_scaling_factor, int8_t mode, uint8_t bits){
	float acc;

	if(bits == 32){
		// mode = 1 when float_to_integer() in is  <= 1
		if(mode){
			acc = (float)((result_cordic_integer/2147483648.0f)/squarted_scaling_factor);
		}
		// mode = 0 when float_to_integer() in is > 1
		if(!mode){
			acc = (float)((result_cordic_integer/2147483648.0f)*squarted_scaling_factor);
		}
	}

	if(bits == 16){
		// mode = 1 when float_to_integer() in is  <= 1
		if(mode){
			acc = (float)((result_cordic_integer/32767.0f)/squarted_scaling_factor);
		}
		// mode = 0 when float_to_integer() in is > 1
		if(!mode){
			acc = (float)((result_cordic_integer/32767.0f)*squarted_scaling_factor);
		}
	}

	return acc;

}


// Convert float to Q8.24 fixed-point
static inline int32_t float_to_fixed(float x) {
    return (int32_t)(x * FP_SCALE + (x >= 0 ? 0.5f : -0.5f));
}

// Convert Q8.24 fixed-point back to float (if needed)
static inline float fixed_to_float(int32_t x) {
    return ((float)x) / FP_SCALE;
}

// Simple polynomial approximation for acos(x) for x in [0.5, 1]:
static inline float approx_acos(float x) {
    // Example coefficients – you would need to adjust these for your range and precision.
    float a0 = 1.5708f;  // ~pi/2
    float a1 = -1.5700f;
    return a0 + a1 * x;
}
static inline float approx_acos2(float x) {
    // Example coefficients – you would need to adjust these for your range and precision.
	if (x < -1.0f) x = -1.0f;
	    else if (x > 1.0f) x = 1.0f;
	    float sqrt_val = sqrtf(1.0f - x);
	    return sqrt_val * (1.5707963050f + x * (-0.2145988016f + 0.0889789874f * x));
}

static inline float approx_acos3(float x) {
    // Example coefficients – you would need to adjust these for your range and precision.
	return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;

}

float lookup_acos(float x) {
	// Clamp x to [0, 1] quickly using fminf/fmaxf.
	    x = fmaxf(0.0f, fminf(x, 1.0f));

	    // Multiply by 256 instead of dividing by 0.00390625.
	    int index = (int)(x * 256.0f);

	    // Make sure index is within [1, LUT_SIZE-1] so that index-1 is valid.
	    if(index < 1)
	        index = 1;
	    else if(index >= LUT_SIZE)
	        index = LUT_SIZE - 1;

	    // Return the corresponding LUT entry.
	    return acos_lut[index - 1];
}

void DisplayAllVariables(void) {
    char buffer[128];

    sprintf(buffer, "KP = %f\n", Kp);
        SendUSBMessage(buffer);

        sprintf(buffer, "Ti = %f\n", Ti);
        SendUSBMessage(buffer);

        sprintf(buffer, "vref = %hu\n", vref);
        SendUSBMessage(buffer);

        sprintf(buffer, "imax1 = %f\n", imax1);
        SendUSBMessage(buffer);

        sprintf(buffer, "imax2 = %f\n", imax2);
        SendUSBMessage(buffer);

        sprintf(buffer, "imin = %f\n", imin);
        SendUSBMessage(buffer);

        sprintf(buffer, "input_voltage = %f\n", input_voltage);
        SendUSBMessage(buffer);

        sprintf(buffer, "output_voltage = %f\n", output_voltage);
        SendUSBMessage(buffer);

        sprintf(buffer, "pcb_temp = %f\n", pcb_temp);
        SendUSBMessage(buffer);

        sprintf(buffer, "heat_sink_temp = %f\n", heat_sink_temp);
        SendUSBMessage(buffer);

        sprintf(buffer, "current_sensor1_vref = %f\n", current_sensor1_vref);
        SendUSBMessage(buffer);

        sprintf(buffer, "current_sensor2_vref = %f\n", current_sensor2_vref);
        SendUSBMessage(buffer);

        sprintf(buffer, "imax2_sum = %f\n", imax2_sum);
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
