/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "function.h"
#include "gpio.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "ILI9341.h"
#include "ov5640.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef IwdgHandle;

extern uint8_t lighting_led_mode;
uint8_t switch_power_mode = 0;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

__asm void wait() 
{ 
      BX lr 
} 

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  printf("hardware fault.\n");
//	wait(); 
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}



uint32_t counter_second = 0;//counter that decide whether go to sleep.
uint32_t counter_scanner = 0;
uint8_t system_current_status = AWAKE;

extern uint8_t Message_USB[3];
extern uint8_t key_CAMERA_TRIGGER_pressed_flag;
extern uint8_t scanning_next_page_flag;
extern uint8_t key_CAMERA_FOCUS_pressed_flag;

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  static uint32_t minisecond_counter = 0;
	if(minisecond_counter % 50 == 0)
	{
	   scanning_next_page_flag = 1;
	}
	if(minisecond_counter >= 1000)
	{ 
//		HAL_IWDG_Refresh(&IwdgHandle);//feed the dog every 1 second.won't work.
//		key_CAMERA_TRIGGER_pressed_flag = PRESSED;//CAMERA DEBUG keep on taking photo
		minisecond_counter = 0;
		counter_second++;
		counter_scanner++;
		if(counter_second%10==0) 
		{  printf("none action consecutive seconds:%d.\n",counter_second);//print every 10 second.
//			 LCD_C5_TEST();
//     	 key_CAMERA_FOCUS_pressed_flag = PRESSED;//take photo every 10 seconds automatically. for debuging
//			 counter_second = 0;//DEBUG.
		}

  }
	else
		minisecond_counter ++;
		if(system_current_status == AWAKE && (counter_second == SECOND_SYSTEM_GO_TO_SLEEP || switch_power_mode == 1))
		{	 switch_power_mode = 0;
			 system_current_status = SLEEPING; 
			 HAL_GPIO_WritePin(LCD_LIGHT_EN_GPIO_Port,LCD_LIGHT_EN_Pin,GPIO_PIN_RESET);
			 lighting_led_mode = 0;
 			 OV5640_Flash_Ctrl(0);
			 printf("system go to sleep.\n");
			 Message_USB[2] = DEVICE_GO_TO_SLEEP;
			 CDC_Transmit_HS_delay(Message_USB,3,300);
			 counter_second = SECOND_SYSTEM_GO_TO_SLEEP;
		}
		if(system_current_status == SLEEPING && (counter_second == 0 ||  switch_power_mode == 1))
		{  switch_power_mode = 0;
			 HAL_GPIO_WritePin(LCD_LIGHT_EN_GPIO_Port,LCD_LIGHT_EN_Pin,GPIO_PIN_SET);
			 system_current_status = AWAKE;
			 printf("system wake up.\n");
			 Message_USB[2] = DEVICE_WAKE_UP;
			 CDC_Transmit_HS_delay(Message_USB,3,300);   
			 counter_second = 0;
		}	
		HAL_IWDG_Refresh(&IwdgHandle);//feed the dog every 1 minisecond.
	
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
//void EXTI0_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI0_IRQn 0 */

//  /* USER CODE END EXTI0_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
//  /* USER CODE BEGIN EXTI0_IRQn 1 */

//  /* USER CODE END EXTI0_IRQn 1 */
//}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
//void EXTI3_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI3_IRQn 0 */

//  /* USER CODE END EXTI3_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
//  /* USER CODE BEGIN EXTI3_IRQn 1 */

//  /* USER CODE END EXTI3_IRQn 1 */
//}

void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}



extern UART_HandleTypeDef huart5;
extern uint8_t rx_buffer_uart5[BARCODE_MAXIM_BYTE];
extern uint8_t rx_len_uart5;
extern uint8_t recv_end_flag_uart5;

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
	rx_buffer_uart5[rx_len_uart5++] =  huart5.Instance->DR ; //(uint16_t)(&hlpuart1->DR & (uint16_t)0x01FF);
	recv_end_flag_uart5 = 1;
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}


/**
* @brief This function handles USB On The Go HS global interrupt.
*/
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
