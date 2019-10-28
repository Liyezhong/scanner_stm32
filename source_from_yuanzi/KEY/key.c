#include "key.h"
#include "delay.h"
#include "main.h"
#include "gpio.h"
#include "dcmi.h"
#include "ILI9341.h"

//extern uint8_t key_SCANNING_QR_pressed_flag ;
extern uint8_t key_CAMERA_TRIGGER_pressed_flag ;
extern uint8_t key_CAMERA_FOCUS_pressed_flag ;
extern uint8_t key_SWITCHING_MODE_pressed_flag ;

extern uint8_t key_ENTER_pressed_flag;
extern uint8_t key_SWITCHING_LED_pressed_flag;

extern uint8_t current_working_mode;
extern uint32_t counter_second ;
extern uint8_t system_current_status;

//按键初始化函数
void KEY_Init(void)
{

//  GPIO_InitTypeDef GPIO_InitStruct;

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();

//  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = KEY2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

//  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = KEY_UP_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(KEY_UP_GPIO_Port, &GPIO_InitStruct);

//  /*Configure GPIO pins : PHPin PHPin */
//  GPIO_InitStruct.Pin = KEY1_Pin|KEY0_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

//  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	
	

  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOH_CLK_ENABLE();	
  __HAL_RCC_GPIOE_CLK_ENABLE();	

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	
//	GPIO_InitStruct.Pin = GPIO_PIN_2;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	
	
//  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);	


}



//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
//u8 KEY_Scan(u8 mode)
//{
//    static u8 key_up=1;     //按键松开标志
//    if(mode==1)key_up=1;    //支持连按
//    if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
//    {
//        delay_ms(10);
//        key_up=0;
//        if(KEY0==0)       return KEY0_PRES;
//        else if(KEY1==0)  return KEY1_PRES;
//        else if(KEY2==0)  return KEY2_PRES;
//        else if(WK_UP==1) return WKUP_PRES;          
//    }else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1;
//    return 0;   //无按键按下
//}



extern uint32_t focus_show_counter;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	
//  if(GPIO_Pin == PIN_SCANNING_QR)
//	{  delay_ms(5);
//		 if(KEY_SCANNING_QR == RESET)
//		 {  key_SCANNING_QR_pressed_flag = PRESSED;
//			  HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_RESET);
//	      printf("Key SCANNING_QR pressed.\n");
//		 }
//	}
	if(GPIO_Pin == PIN_CAMERA_TRIGGER)
	{  
		 delay_ms(5);
		 if(KEY_CAMERA_TRIGGER == RESET)
		 {  
			 if(system_current_status == AWAKE) 
			    key_CAMERA_TRIGGER_pressed_flag = PRESSED;
	      printf("Key CAMERA_TRIGGER pressed.\n");
			  counter_second = 0;
		 }
	}
//	if(GPIO_Pin == PIN_CAMERA_FOCUS)
//	{
		//  delay_ms(5); //delete this to make the scanning qr more efficient.
//if(KEY_CAMERA_FOCUS == RESET)
//		 {  
//			  if(system_current_status == AWAKE) key_CAMERA_FOCUS_pressed_flag = PRESSED;
//	      printf("Key CAMERA_FOCUS pressed.\n");
//			  counter_second = 0;	
//        if(system_current_status == AWAKE && current_working_mode == PHOTO_MODE) focus_show_counter = 0;		
//		 }
//	}
//	if(GPIO_Pin == PIN_SWITCHING_MODE) 
//	{
//		 delay_ms(5);
//		 if(KEY_SWITCHING_MODE == RESET)
//		 {  
//			  if(system_current_status == AWAKE) key_SWITCHING_MODE_pressed_flag = PRESSED;
//	      printf("Key SWITCHING_MODE pressed.\n");
//			  counter_second = 0;
//		 }
//	}
  if(GPIO_Pin == PIN_CAMERA_FOCUS)
	{
		delay_ms(5);
		if(KEY_CAMERA_FOCUS == RESET)
		{
			  if(system_current_status == AWAKE) key_SWITCHING_LED_pressed_flag = PRESSED;//key_ENTER_pressed_flag = PRESSED;
			  counter_second = 0;	
		}
	}
	if(GPIO_Pin == PIN_SWITCHING_MODE) 
	{
		 delay_ms(1);
		 if(KEY_SWITCHING_MODE == RESET)
		 {  
				if(system_current_status == AWAKE) key_ENTER_pressed_flag = PRESSED;//key_SWITCHING_LED_pressed_flag = PRESSED;
			  counter_second = 0;
		 }
	}

}
