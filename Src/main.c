
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FMASTER				      48000000                        // Frequency of CPU in Hz
#define TIM1_CLK			      16000000                        // Hz
#define	TIM1_PSC			      (FMASTER / TIM1_CLK -1)
#define TIM1_PWM_FREQ	      32000                           // Hz
#define TIM1_ARR			      (TIM1_CLK / TIM1_PWM_FREQ - 1)
#define TIM1_PWM_100	      (TIM1_CLK / TIM1_PWM_FREQ)
#define DEAD_TIME			      500	                            // ns
#define	DEAD_TIME_CNT	      ((uint8_t)((float)TIM1_CLK * DEAD_TIME / 1000000000)) 
#define CCW                 0                               // ClockCounterWise
#define CW                  1                               // ClockWise
#define DUTY_CYCLE_PPT_MAX  967                             // 96.7%
#define Duty_Cycle_CCW_PPT Duty_Cycle_1_PPT
#define Duty_Cycle_CW_PPT  Duty_Cycle_2_PPT
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t Calc_CCR_Value (uint16_t Duty_Cycle_PPT);
void Set_Motor_State (int16_t Duty_Cycle_PPT_Target);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t Duty_Cycle_1_PPT  = 500;  // 50.0% just initial
uint16_t Duty_Cycle_2_PPT  = 250;  // 25.0% just initial
uint16_t Duty_Cycle_3_PPT  = 500;  // 50.0% just initial


uint16_t CCR1_Val;
uint16_t CCR2_Val;
uint16_t CCR3_Val;
uint8_t	Dead_Time = DEAD_TIME_CNT;

int16_t Motor_Duty_Cycle_PPT        = 0;    // Current DutyCycle, in PPT (-1000 to 1000, but limited to +-DUTY_CYCLE_PPT_MAX PartsPerThousand)
                                        // 0 is Stopped, positive is CCW, negative is CW
int16_t Motor_Duty_Cycle_PPT_Target = 0;    // Target DutyCycle, in PPT (-1000 to 1000, and 0 is Stopped)
uint16_t Ramp_PPT                 = 1;       // Motor Speed Up/Dn ramp
_Bool Main_Loop_Scan_Flag = 0;      // 1ms Main loop Scan Trigger TIM4 Interrup
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    while (!Main_Loop_Scan_Flag);                               // waits 1ms to release the main loop scan
    Main_Loop_Scan_Flag = 0;                                // reset Loop Scan flag
    
    /* Set the Motor Duty Cycle (CW or CCW) movement */
    Set_Motor_State (Motor_Duty_Cycle_PPT_Target);
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* USER CODE BEGIN 4 */
uint16_t Calc_CCR_Value (uint16_t Duty_Cycle_PPT)
{
  return ((uint16_t)((uint32_t)Duty_Cycle_PPT * TIM1_PWM_100 / 500) + 1) >> 1;
}

void Set_Motor_State (int16_t Duty_Cycle_PPT_Target)
{
  if (Duty_Cycle_PPT_Target > DUTY_CYCLE_PPT_MAX)                               // Limiting max CCW
    Duty_Cycle_PPT_Target = DUTY_CYCLE_PPT_MAX;
  else if (Duty_Cycle_PPT_Target < (-DUTY_CYCLE_PPT_MAX))                       // Limiting max CW
    Duty_Cycle_PPT_Target = (-DUTY_CYCLE_PPT_MAX);
  if (Ramp_PPT == 0) Ramp_PPT = 1;                                              // Limiting min Ramp Up/Dn
  
  if (Motor_Duty_Cycle_PPT < Duty_Cycle_PPT_Target)                             // Motor Changing Speed tending to CCW
  {
    Motor_Duty_Cycle_PPT += Ramp_PPT;                                           // Add Ramp
    if (Motor_Duty_Cycle_PPT > Duty_Cycle_PPT_Target)                           // Limiting value to target
      Motor_Duty_Cycle_PPT = Duty_Cycle_PPT_Target;
  }
  else if (Motor_Duty_Cycle_PPT > Duty_Cycle_PPT_Target)                        // Motor Changing Speed tending to CW
  {
    Motor_Duty_Cycle_PPT -= Ramp_PPT;                                           // Sub Ramp
    if (Motor_Duty_Cycle_PPT < Duty_Cycle_PPT_Target)                           // Limiting value to target
      Motor_Duty_Cycle_PPT = Duty_Cycle_PPT_Target;
  }
  else // if (Motor_Duty_Cycle_PPT == Duty_Cycle_PPT_Target)                    // Motor keeping Speed constant
  {
    // Nothing to do
  }
  
  if (Motor_Duty_Cycle_PPT == 0)
  {
    Duty_Cycle_CCW_PPT = 0;                                                     // 0% DutyCycle, means CCW arm at Low (0V)
    Duty_Cycle_CW_PPT = 0;                                                      // 0% DutyCycle, means CW arm at Low (0V) 
  }
  else if (Motor_Duty_Cycle_PPT > 0)                                            // Motor running CCW
  {
    Duty_Cycle_CCW_PPT = Motor_Duty_Cycle_PPT;                                  // Set CCW DutyCycle as positive voltage
    Duty_Cycle_CW_PPT = 0;                                                      // 0% DutyCycle, means CW arm at Low (0V) 
  }
  else // if (Motor_Duty_Cycle_PPT < 0)                                         // Motor running CW
  {
    Duty_Cycle_CW_PPT = -Motor_Duty_Cycle_PPT;                                  // Set CW DutyCycle as positive voltage
    Duty_Cycle_CCW_PPT = 0;                                                     // 0% DutyCycle, means CCW arm at Low (0V) 
  }
  CCR1_Val = Calc_CCR_Value(Duty_Cycle_CCW_PPT);
  CCR2_Val = Calc_CCR_Value(Duty_Cycle_CW_PPT);
	TIM1->CCR1 = CCR1_Val;
	TIM1->CCR2 = CCR2_Val;
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
