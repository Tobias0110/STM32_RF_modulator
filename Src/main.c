
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
	USER CODE COPYRIGHT(c) 2020 Tobias Ecker OE3TEC and Matthias Preymann
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#define ARM_MATH_CM3
#include "arm_math.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#define TX_FREQ 36000000 //3600kHz
#define PHASE_RES 58968
#define MAX_F 50000 //Max NFM diveration

//Macros
#define toQ31 2147483648*
#define mulQ31( a, b ) ((int32_t)(( ((int64_t) a) * ((int64_t) b) ) >> 31 ))
#define rW( n ) w[(pos- n)&0xF]
#define fromQ31( v ) (((float)v) / 2147483648 )
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
volatile uint8_t new_sample = 0, carrier = 0;  //new_sample: tells the main programm if there is a new sample to process
volatile static int16_t adc_value = 0, amplitude = 0; //in and outputs
volatile int32_t dif = 0;
volatile static int32_t offset;

inline static void set_ad9850_freq(uint64_t freq_buf, uint8_t pw_d) //freq in * 0,1 Hz (dezi Hz)
{
	static uint8_t w[5], i;
	static const uint64_t ref = 1250000000;
	static uint64_t w_erg;
	
	w_erg = freq_buf * 4294967296 / ref;  //calculate the 32bit frequency register word for the AD9850
	
	//w_erg = 0xFF00FF00; //Test
	
	w[0] = (~pw_d & 0x01) << 2; //allowes to turn of the carrier
	w[1] = (w_erg >> 24) & 0xFF;
	w[2] = (w_erg >> 16) & 0xFF;
	w[3] = (w_erg >> 8) & 0xFF;
	w[4] = w_erg & 0xFF;
	
	//Set FQ_UD to low
	GPIOC->ODR &= ~(0x01 << 8);
	
	for(i=0;i<=4;i++)
	{
		GPIOC->ODR &= ~0xFF;
		GPIOC->ODR |= w[i];
		GPIOC->ODR |= 0x01 << 9; //W_CLK high
		GPIOC->ODR &= ~(0x01 << 9);  //W_CLK low
	}
	
	//Set FQ_UD to high
	GPIOC->ODR |= 0x01 << 8;
	
}
//I do not plan on implementing AM modulation (makes the transmitter simpler and cheaper)
__forceinline static void bc_am()  //broadcast quality AM. Uses max. sample rate while optaining 12 bit PWM range.
{
	//DC Removal variables
	volatile static float w_dc = 0, w_dc1 = 0;
	static const float a_dc = 0.99;

	//Remove DC
	w_dc = adc_value + a_dc * w_dc1;
	amplitude = ((int) (w_dc - w_dc1)) + 2048; //Add DC offset for carrier (2^12 / 2 = 2048)
	w_dc1 = w_dc;
	new_sample = 0;
}
__forceinline static void ssb(uint8_t lsb)
{
	//DC Removal variables
	volatile static float w_dc = 0, w_dc1 = 0;
	static const float a_dc = 0.99;
	int16_t hilbert_input_i;
	static q31_t hilbert_input;

	//Hilbert-Filter variables
	static q31_t w[16], y_I = 0, y_Q = 0;
	static q31_t amplitude_q;
	static const q31_t a[4] = {toQ31(0.052981), toQ31(0.088199), toQ31(0.18683), toQ31(0.62783)};  //calculated with octave
	static uint8_t pos = 0; //ringbuffer position
	
	//Phase to Frquency variables
	static float  phase;
	static uint16_t prev_phase;
	
	//will be implemented in q31 in the future
	w_dc = adc_value + a_dc * w_dc1;
	hilbert_input_i = (int) (w_dc - w_dc1);
	w_dc1 = w_dc;
	
	hilbert_input = hilbert_input_i * ( toQ31(1) / 2048 );

	w[++pos & 0x0F] = hilbert_input;
	y_I = rW(7);
	//structure optimation inspired by wikipedia: Hilbert-Transformation
	//Designed with Octave
	y_Q = mulQ31(rW(14) - rW(0), a[0]) +
        mulQ31(rW(12) - rW(2), a[1]) +
        mulQ31(rW(10) - rW(4), a[2]) +
        mulQ31(rW(8) - rW(6), a[3]);
				
//++++++++THE FOLLOWING PART IS UNDER CONSTRUCTION++++++++
	
	//Komponentenschreibweise zu Winkelschreibweise (Component notation to Angular notation?)
	arm_sqrt_q31(mulQ31(y_I, y_I) + mulQ31(y_Q, y_Q), &amplitude_q);
	//amplitude = (uint16_t) mulQ31( amplitude_q, (q31_t) toQ31( 4096 / toQ31(1))); //Not tested
	
	if((y_Q != 0) && (y_I != 0))
	{
		phase = atan2f(fromQ31(y_Q), fromQ31(y_I));
		//phase = y_Q / y_I;
		//phase = (PI / 4) * phase + 0.285 * phase * (1 - fabs(phase));
		if(phase < 0) phase = 2 * PI + phase; //convert negativ phase to positiv phase (atan2f returns a value between -pi and +pi)
		phase = (phase / (2 * PI)) * PHASE_RES; //PHASE_RES is the phase resulution (importent when converting to int)
	}
	else
	{
		phase = 0;
	}
	
	//The differential of a phase gives the frequency change (you can convert a frequency modulator into a phase modulator)
	dif = (int) phase - prev_phase;
	prev_phase = (int) phase;
	
	//a "circular" nummber set is needed
	if(dif < 0) dif = dif + PHASE_RES; //avoid negativ phase differences (and therefore negative frequencies)
	
	if(dif > MAX_F) //Reducing the bandwidth while maintaining the required frequency shift (delaying to the next sample)
	{
		prev_phase = phase - (dif - MAX_F);
		dif = MAX_F;
	}
	
	offset = dif;
	
	carrier = 1;
	if(amplitude_q == 0) carrier = 0; //no carrier when there is no audio input and thus no amplitude
	
	//LSB
	if(lsb == 1) offset = offset * -1; //mirror the frequencies on the carrier.
	
	new_sample = 0; //calculation finished
	
//++++++++END OF CONSTRUCTION AREA++++++++
}

__forceinline static void nfm(uint8_t comp)
{
	//DC Removal variables
	volatile static float w_dc = 0, w_dc1 = 0;
	static const float a_dc = 0.99;
	int16_t out_dc;
	
	//Preemphasis Filter variables
	static q31_t w[16], input, output;
	static const q31_t b[6] = {toQ31(-0.0049835), toQ31(0.0098803), toQ31(-0.035046), toQ31(0.066495), toQ31(-0.21751), toQ31(0.22571)};
	static uint8_t pos = 0;
	static int16_t gain = 0;
	static uint16_t to_high = 0, to_low = 0;
	
	//DC blocking filter (IIR Filter)
	w_dc = adc_value + a_dc * w_dc1;
	out_dc = (int) (w_dc - w_dc1);
	w_dc1 = w_dc;
	
	//input = adc_value * ( toQ31(1) / 2048 ); //Deactivates the DC blocking filter (not recommended)
	
	input = out_dc * ( toQ31(1) / 2048 ); //converting to q31
	
	//Preemphasis Filter (High pass filter 6dB/Octave)
	w[++pos & 0x0F] = input;
	output = 	mulQ31(rW(0), b[0]) +
						mulQ31(rW(1), b[1]) +
						mulQ31(rW(2), b[2]) +
						mulQ31(rW(3), b[3]) +
						mulQ31(rW(4), b[4]) +
						mulQ31(rW(5), b[5]) +
						mulQ31(rW(6), b[4]) +
						mulQ31(rW(7), b[3]) +
						mulQ31(rW(8), b[2]) +
						mulQ31(rW(9), b[1]) +
						mulQ31(rW(10), b[0]);
	
	//output = input; //for deactivating the Preemphasis Filter
	
	if(comp == 0) gain = 0; //Deactivates the compressor
	offset = ((int64_t)output * 2048 * (384 + gain)) / toQ31(1); // Gain: low frequencies souldn't be changed, high frequencys are amplified (compared to the input signal)
	
	//Compressor
	if(offset > 50000) //Limmit frequency drift
	{
		offset = 50000;
		to_high++;
	}
	if(offset < -50000) //Limmit frequency drift
	{
		offset = -50000;
		to_high++;
	}
	
	if(to_high > 0) //Release
	{
		gain--;
		to_high--;
	}
	else if (offset > 100) //avoid triggering on silence
	{
		to_low++;
		if(to_low > 5) //Attack
		{
			gain++;
			to_low = 0;
		}
	}
	
	carrier = 1; //Carrier continuously on
	new_sample = 0; //calculation finished
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t mod = 5; //0 = No_TX; 1 = AM; 2 = FM; 3 = USB; 4 = LSB; 5 = NFM
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, DDS_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, DDS_Reset_Pin, GPIO_PIN_RESET);
	
	HAL_ADC_Start_IT(&hadc1); //Start audio ADC
  HAL_TIM_Base_Start(&htim3); //This timer defines the audio sample rate
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //RF amplitude PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //RF frequency PWM (not used anymore)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//Speedtest for DDS communication
/*while(1)
{
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); //Speed Test
	set_ad9850_freq(36000000, 0);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); //Speed Test
	HAL_Delay(1);
}*/
while(1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if((mod == 1) && (new_sample == 1))
	  {
		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); //Speed Test

		  bc_am();

		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); //Speed Test
	  }
	  else if((mod == 3) && (new_sample == 1))
	  {
		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); //Speed Test

		  ssb(0);

		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); //Speed Test
	  }
		else if((mod == 4) && (new_sample == 1))
		{
		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); //Speed Test

		  ssb(1);

		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); //Speed Test
	  }
		else if((mod == 5) && (new_sample == 1))
		{
		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); //Speed Test

		  nfm(1);

		  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); //Speed Test
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	//sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8190;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8190;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4095;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, dds0_Pin|dds1_Pin|dds2_Pin|dds3_Pin 
                          |dds4_Pin|dds5_Pin|dds6_Pin|dds7_Pin 
                          |fq_ud_Pin|w_clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DDS_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dds0_Pin dds1_Pin dds2_Pin dds3_Pin 
                           dds4_Pin dds5_Pin dds6_Pin dds7_Pin 
                           fq_ud_Pin w_clk_Pin */
  GPIO_InitStruct.Pin = dds0_Pin|dds1_Pin|dds2_Pin|dds3_Pin 
                          |dds4_Pin|dds5_Pin|dds6_Pin|dds7_Pin 
                          |fq_ud_Pin|w_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DDS_Reset_Pin */
  GPIO_InitStruct.Pin = DDS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DDS_Reset_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint16_t cnt = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) //Called if new audio sample is available
{
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //Test speed
	adc_value = HAL_ADC_GetValue(hadc1) - 2048; //Read the ADC value
	//ADC is very noisy at the moment (input circuit should be improved)
  if((adc_value < 100) && (adc_value > -100)) //makes sure that the carrier is turned off when there is no audio input
	{
		if(cnt == 800) //number of samples near 0 that trigger carrier deactivation
		{
			adc_value = 0;
		}
		else
		{
			cnt++;
		}
	}
	else
	{
		cnt = 0;
	}
	
	new_sample = 1; //Tells the main program to compute a new sample
	TIM4->CCR1 = amplitude; //output the last calculated RF sample amplitude
	//TIM2->CCR1 = dif; //output the last calculated RF sample frequency (not used any more)
	set_ad9850_freq(TX_FREQ + offset, carrier); //Set the DDS frequnecy
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
