/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define dds0_Pin GPIO_PIN_0
#define dds0_GPIO_Port GPIOC
#define dds1_Pin GPIO_PIN_1
#define dds1_GPIO_Port GPIOC
#define dds2_Pin GPIO_PIN_2
#define dds2_GPIO_Port GPIOC
#define dds3_Pin GPIO_PIN_3
#define dds3_GPIO_Port GPIOC
#define Audio_Input_Pin GPIO_PIN_0
#define Audio_Input_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define dds4_Pin GPIO_PIN_4
#define dds4_GPIO_Port GPIOC
#define dds5_Pin GPIO_PIN_5
#define dds5_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define dds6_Pin GPIO_PIN_6
#define dds6_GPIO_Port GPIOC
#define dds7_Pin GPIO_PIN_7
#define dds7_GPIO_Port GPIOC
#define fq_ud_Pin GPIO_PIN_8
#define fq_ud_GPIO_Port GPIOC
#define w_clk_Pin GPIO_PIN_9
#define w_clk_GPIO_Port GPIOC
#define DDS_Reset_Pin GPIO_PIN_9
#define DDS_Reset_GPIO_Port GPIOA
#define TX_RX_IN_Pin GPIO_PIN_4
#define TX_RX_IN_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_5
#define E_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
