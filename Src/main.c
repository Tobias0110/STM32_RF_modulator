
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

#define PHASE_RES 58968
//#define SSB_LIM
#define IIR_DC_BLOCK
#define MAX_F 30000 //Max SSB frequency offset
#define PHASE_RES_2 31

//Macros
#define GPIOB_PIN(n) (*(PERIPH_BB_BASE + (GPIOB->ODR - PERIPH_BASE) * 0x20 + n * 4)) //Bit banding
//#define toQ31 2147483648*
#define toQ31( v ) ((int32_t) ((2147483648 - 1)*v))
#define mulQ31( a, b ) ((int32_t)(( ((int64_t) (a)) * ((int64_t) (b)) ) >> 31 ))
#define rW( n ) w[(pos- n)&0xF]
#define fromQ31( v ) (((float)v) / (2147483648 - 1) )
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
volatile static uint8_t phase_reg;
volatile static uint32_t freq = 36000000; //TX/RX Frequency

static void lcd_write(uint8_t data, uint8_t RS)
{
	uint8_t i;
	
	GPIOB->ODR &= 0x005F; //Clear all Bits
	GPIOB->ODR |= RS << 7; //Set RS
	GPIOB->ODR |= 1 << 5; //Set E to high
	GPIOB->ODR |= data << 8; //Output data
	for(i=0;i<255;i++); //Delay
	GPIOB->ODR &= ~(1 << 5); //Set E to low
	for(i=0;i<255;i++); //Delay
}

static void lcd_init()
{
	HAL_Delay(15);
	lcd_write(0x30, 0);
	HAL_Delay(5);
	lcd_write(0x30, 0);
	HAL_Delay(1);
	lcd_write(0x30, 0);
	//Select Mode (cursor increment, position fix)
	lcd_write(0x06, 0);
	HAL_Delay(1);
	//Display on, cursor off
	lcd_write(0x0C, 0);
	//lcd_write(0x0F, 0);
	HAL_Delay(5);
	//move cursor to the rigth
	lcd_write(0x14, 0);
	HAL_Delay(5);
	//8bit, two lines, 5x7 Font
	lcd_write(0x38, 0);
	HAL_Delay(5);
}

static void lcd_write_string(char data[], uint8_t line)
{
	uint8_t i = 0;
	
	//Clear Display
	if(line == 0) lcd_write(0x80, 0);
	else lcd_write(0xC0, 0);
	//HAL_Delay(1);
	while(data[i] != 0)
	{
		lcd_write((uint8_t) data[i], 1);
		i++;
		
	}
}

static void update_smeter(uint8_t s_value, uint8_t mod)
{
	char out[17], buffer[5];
	uint8_t i;
	
	out[0] = 'S';
	for(i=1; i<=9; i++) //S-Meter Graph
	{
		if(i <= s_value) out[i] = '>';
		else out[i] = ' ';
	}
	out[10] = 0x0;
	if(s_value < 10)
	{
		sprintf(buffer, "%d ", s_value);
		strcat(out, buffer);
	}
	else 
	{
		out[10] = '9';
		out[11] = '+';
	}
	out[12] = 0x0;
	
	if(mod == 5) strcat(out, " NFM");
	else if(mod == 3) strcat(out, " USB");
	else if(mod == 4) strcat(out, " LSB");
	
	lcd_write_string(out, 1);
}

static void update_freq(uint8_t tx)
{
	char out[17], buffer[16];
	uint8_t len, i;
	
	sprintf(out, "%d", freq);
	len = strlen(out);
	for(i=0;i<=(8-len); i++) buffer[i] = '0'; //Add 0 to get constant string size
	buffer[i++] = 0;
	strcat(buffer, out);
	
	//Add points
	for(i=0;i<=1;i++) out[i] = buffer[i];
	out[2] = '.';
	for(i=3;i<=5;i++) out[i] = buffer[i-1];
	out[6] = '.';
	for(i=7;i<=9;i++) out[i] = buffer[i-2];
	out[10] = ',';
	out[11] = buffer[8];
	out[12] = 'H';
	out[13] = 'z';
	out[14] = ' ';
	if(tx == 1) out[15] = '!';
	else out[15] = ' ';
	out[16] = 0;
	
	lcd_write_string(out, 0);
}

inline static void set_ad9850_freq(uint64_t freq_buf, uint8_t pw_d, uint8_t phase_reg_in) //freq in * 0,1 Hz (dezi Hz)
{
	static uint8_t w[5], i;
	static const uint64_t ref = 1250000000;
	static uint64_t w_erg;
	
	w_erg = freq_buf * 4294967296 / ref;  //calculate the 32bit frequency register word for the AD9850
	
	//w_erg = 0xFF00FF00; //Test
	
	w[0] = ((phase_reg_in << 1) | (~pw_d & 0x01)) << 2; //allowes to turn of the carrier
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
	w_dc = (adc_value >> 3) + a_dc * w_dc1;
	amplitude = ((int) (w_dc - w_dc1)) + 2048; //Add DC offset for carrier (2^12 / 2 = 2048)
	w_dc1 = w_dc;
	new_sample = 0;
}

__forceinline int32_t abs( int32_t a ) { return a < 0 ? (0 -a) : a; }

//only positiv values allowed
/*__forceinline static int32_t divQ31(int32_t a, int32_t b)
{
    static int32_t erg;

    if( a == b) return toQ31(1);
    else
    {
    erg = a / (b >> 15);
    return erg << 16;
    }
}*/

//Sounds better
__forceinline static int32_t divQ31(int32_t a, int32_t b)
{
	return toQ31(fromQ31(a) / fromQ31(b));
}

__forceinline static int32_t atanQ31(int32_t x)
{
    //x = [-1 1] --> y = [-0.25 0.25] = [-45° 45°]
    return mulQ31( toQ31(1/PI), mulQ31(toQ31(PI/4), x) + mulQ31(mulQ31(toQ31(0.285), x), (toQ31(1) - abs(x))));
}

__forceinline static int32_t atan2_Q31(int32_t q, int32_t i)
{
    static int32_t erg;

    if(abs(q) > abs(i))
    {
        erg = toQ31(0.5) - atanQ31(divQ31(abs(i), abs(q))); //arctan(z) = 90-arctan(1/z) (0.5 = 90°)
    }
    else
    {
        erg = (i == 0) ? 0 : atanQ31(divQ31(abs(q), abs(i))); //arctan(z)
    }
    erg = (i < 0) ? toQ31(1) - erg : erg; //arctan(-z) = -arctan(z) (1 = 180°)
    return (q < 0) ? 0-erg : erg;
}

__forceinline static void ssb(uint8_t lsb)
{
	//DC Removal variables
	volatile static float w_dc = 0, w_dc1 = 0;
	static q31_t hilbert_input;

	//Hilbert-Filter variables
	static q31_t w[16], y_I = 0, y_Q = 0;
	static q31_t amplitude_q;
	static const q31_t a[4] = {toQ31(0.052981), toQ31(0.088199), toQ31(0.18683), toQ31(0.62783)};  //calculated with octave
	static uint8_t pos = 0; //ringbuffer position
	static const q31_t div = toQ31(0.5);
	
	//Phase to Frquency variables
	static int32_t phase_i = 0, prev_phase_i = 0;
	
	#ifdef IIR_DC_BLOCK
	static q31_t adc_value_q, wdc_q, wdc1_q;
	static const q31_t adc_q = toQ31(0.99);
	
	adc_value_q = adc_value * ( toQ31(1) / 2048 );
	adc_value_q = adc_value_q >> 10;  //Avoids overflow
	wdc_q = adc_value_q + mulQ31(adc_q,  wdc1_q);
	hilbert_input = wdc_q - wdc1_q;
	wdc1_q = wdc_q;
	
	hilbert_input = hilbert_input << 7; //More resulution for the hilbert transformer
	
	#else
	static int32_t dc;
	dc += (adc_value - dc) / 2;
	adc_value -= dc;
	hilbert_input = adc_value * ( toQ31(1) / 2048 );
	#endif

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
	//arm_sqrt_q31(mulQ31(mulQ31(y_I, y_I), div) + mulQ31(mulQ31(y_Q, y_Q), div), &amplitude_q);
	amplitude_q = mulQ31(mulQ31(y_I, y_I), div) + mulQ31(mulQ31(y_Q, y_Q), div); //is sufficient for detecting 0
	//amplitude = (uint16_t) mulQ31( amplitude_q, (q31_t) toQ31( 4096 / toQ31(1))); //Not tested
	
	static q31_t phase_q;
	
		if((y_I == 0) && (y_Q > 0)) phase_q = toQ31(0.5); //phase strebt gegen 90° wenn I gegen 0 geht und Q positiv ist
		else if ((y_I == 0) && (y_Q < 0)) phase_q = toQ31(-0.5); //phase strebt gegen -90° wenn I gegen 0 geht und Q negativ ist
		//else phase_q = toQ31(atan2f(fromQ31(y_Q), fromQ31(y_I)) / PI); //[-1 1]
		else phase_q = atan2_Q31(y_Q, y_I);
		//phase = phase / 2;
		phase_q = phase_q >> 1;
		//convert negativ phase to positiv phase (atan2f returns a value between -pi and +pi)
		if(phase_q < 0) phase_q = toQ31(1) + phase_q;
		//PHASE_RES is the phase resulution (importent when converting to int)
		phase_i = (((phase_q >> 16) * PHASE_RES) >> 15);
	
	//The differential of a phase gives the frequency change (you can convert a frequency modulator into a phase modulator)
	
	dif = phase_i - prev_phase_i;
	prev_phase_i = phase_i;
	
	//a "circular" nummber set is needed
	if(dif < 0) dif = dif + PHASE_RES; //avoid negativ phase differences (and therefore negative frequencies)
	
	//Reduces bandwith but lowers quality
	#ifdef SSB_LIM
	if(dif > MAX_F) //Reducing the bandwidth while maintaining the required frequency shift (delaying to the next sample)
	{
		prev_phase_i = phase_i - (dif - MAX_F);
		dif = MAX_F;
	}
	#endif
	
	offset = dif;
	
	carrier = 1;
	if(amplitude_q < 20) carrier = 0; //no carrier when there is no audio input and thus no amplitude
	
	//LSB
	if(lsb == 1) offset = offset * -1; //mirror the frequencies on the carrier.
	
	//DDS Phase Reg
	phase_reg = 0;
	
	new_sample = 0; //calculation finished
	
//++++++++END OF CONSTRUCTION AREA++++++++
}

__forceinline static void nfm(uint8_t comp)
{
	static int16_t gain = 0;
	static uint16_t to_high = 0, to_low = 0;
	
	//DC Blocking and preemphis
	static int32_t dc;
	dc += (adc_value - dc) / 2;
	adc_value -= dc;
	
	if(comp == 0) gain = 0; //Deactivates the compressor
	offset = (int32_t) (adc_value * (50 + gain));
	
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
	else if (offset > 1000) //avoid triggering on silence
	{
		to_low++;
		if(to_low > 5) //Attack
		{
			gain++;
			to_low = 0;
		}
	}
	carrier = 1;
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
	uint8_t mod = 5; //1 = AM; 3 = USB; 4 = LSB; 5 = NFM
	uint8_t tx = 0, tx_prev = 0;
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
	
	lcd_init();
	
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
	
//READ TX_RX_IN Pin
tx = HAL_GPIO_ReadPin(GPIOB, TX_RX_IN_Pin);

//++++RECEIVE++++
	
	if(tx == 0)
	{
		if(tx_prev == 1) //change configuration after TX
		{
			HAL_TIM_Base_Stop(&htim3); //Stop MIC sampling
			set_ad9850_freq(0,0,0); //No TX
			update_freq(tx); //TX indicator
			HAL_Delay(200); //entprellen
		}
		tx_prev = tx;
		
		//High speed stuff
		HAL_Delay(1);
		update_smeter(10, mod);
		update_freq(tx);
	}
	
//++++TRANSMIT++++	
	
	else
	{
	if(tx_prev == 0) //change configuration after RX
	{
		HAL_Delay(200); //entprellen
		update_freq(tx); //TX indicator
		update_smeter(0, mod);
		HAL_TIM_Base_Start(&htim3); //Start MIC sampling
	}
	tx_prev = tx;
	
	//High Speed stuff
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|E_Pin|RS_Pin 
                          |D0_Pin|D1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LED_Pin DDS_Reset_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DDS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin E_Pin RS_Pin 
                           D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|E_Pin|RS_Pin 
                          |D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_RX_IN_Pin */
  GPIO_InitStruct.Pin = TX_RX_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TX_RX_IN_GPIO_Port, &GPIO_InitStruct);

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
	set_ad9850_freq(freq + offset, carrier, phase_reg); //Set the DDS frequnecy
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
