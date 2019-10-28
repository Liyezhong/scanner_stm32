
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "sys.h"
#include "delay.h"
//#include "usart.h"
#include "key.h"
#include "ILI9341.h"
#include "ltdc.h"
#include "sdram.h"
#include "malloc.h"
#include "ov5640.h" 
#include "dcmi.h"  
#include "pcf8574.h" 
//#include "ff.h"
//#include "exfuns.h"
//#include "fontupd.h"
//#include "text.h"
#include "main.h"
#include "function.h"

#include "usb_device.h"
#include "GUI.h"
#include "scannerDLG.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


uint8_t key_CAMERA_TRIGGER_pressed_flag = UNPRESSED;
uint8_t key_CAMERA_FOCUS_pressed_flag = UNPRESSED;
uint8_t key_SWITCHING_MODE_pressed_flag = UNPRESSED;
 
uint8_t key_ENTER_pressed_flag = UNPRESSED;
uint8_t key_SWITCHING_LED_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_PHOTO_MODE_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_BARCODE_MODE_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_AUTOSCAN_MODE_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_SINGLESCAN_MODE_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_ENABLE_USB_MODE_pressed_flag = UNPRESSED;
uint8_t key_SWITCH_TO_DISABLE_USB_MODE_pressed_flag = UNPRESSED;
uint8_t key_INQUIRE_STAUTS_pressed_flag = UNPRESSED;
uint8_t key_PC_TRIGGER_pressed_flag = UNPRESSED;

uint8_t Message_USB[4] = {0xBB,0xDD,0x00,0x00};

uint8_t current_working_mode = BARCODE_MODE ; 
uint8_t current_scan_mode = SINGLESCAN_MODE;
uint8_t lighting_led_mode = 0;
uint8_t usb_comm_mode = 1;

extern uint8_t ov5640_flash_light_flag;
extern TIM_HandleTypeDef htim5;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern struct USB_message message_photo;
extern struct USB_message message_qrcode;
extern uint32_t counter_second;
extern uint8_t system_current_status;
extern IWDG_HandleTypeDef IwdgHandle;
extern GUI_CONST_STORAGE GUI_BITMAP bmscanning_pic;
extern GUI_CONST_STORAGE GUI_BITMAP bmBarcode;
extern uint8_t scanning_line_y_axis;
extern uint32_t counter_scanner;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void LCD_reinit()
{
	SDRAM_Init();	
	TFTLCD_Init();
 	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池   
	GUI_Init(); //emwin needed.
	DCMI_Init();						//DCMI配置 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t counter = 0,i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); 

  /* USER CODE BEGIN Init */
   Stm32_Clock_Init(384,25,2,8);//设置时钟,192Mhz
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  delay_init(180);				//初始化延时函数 
  my_GPIO_Init();
	
	LCD_Start_Reset();
	uart_init(115200);			//初始化串口波特率为115200 
	printf("hi,there.\n");


	usart2_init(961200);
	usart5_init(9600);	
	SDRAM_Init();						//初始化SDRAM 
	TFTLCD_Init();							//初始化LCD
	KEY_Init();							//初始化按键
	OV5640_Init();					//初始化OV5640
 	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池    	
//	POINT_COLOR=RED; 
//	LCD_Clear(BLUE); 	
	__HAL_RCC_CRC_CLK_ENABLE();		//使能CRC时钟  emwin needed
	GUI_Init(); //emwin needed.
  MX_TIM5_Init();
	printf("hello, world.\n");	

//  lcd_show_sucess();//debug
//  lcd_show_USB_Error();
//	while(1); //debug

	emwin_demo();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USB_DEVICE_Init();
 
  //OV5640_ImageWin_Set(580,0,1463,1951);
	OV5640_RGB565_Mode();		//RGB565模式 
	OV5640_Focus_Init(); 
	OV5640_Light_Mode(0);		//自动模式
	OV5640_Color_Saturation(3);	//色彩饱和度3
	OV5640_Brightness(4);		//亮度4
	OV5640_Contrast(3);			//对比度3
	OV5640_Sharpness(20);		//自动锐度33 
	//OV5640_Special_Effects(0);
	OV5640_Focus_Constant();//启动持续对焦
	DCMI_Init();						//DCMI配置
  message_photo.ID = 0;//	
	message_qrcode.ID = 0;
	
	watchdog_init();
	
	beep_start();	
	
	Message_USB[2] = SWITCH_MODE_TO_QR  ;
  CDC_Transmit_HS_delay(Message_USB,3,300);//while(CDC_Transmit_HS(Message_USB,4)!=USBD_OK) ;//				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Message_USB, 4);	
			
//  while(CDC_Transmit_HS("Laica scanner",13)!=USBD_OK) //	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, "Laica scanner", 13);	
//this one will cause hardware fault sometimes, maybe it's because sending usb data before usb init ready.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	current_scan_mode = SINGLESCAN_MODE;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(current_working_mode == PHOTO_MODE)
		{
//				if(key_0_pressed_flag == PRESSED)
//				{  
//					 HAL_Delay(10);
//					 if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)== GPIO_PIN_RESET)
//					 {
//						 ov5640_flash_light_flag= !ov5640_flash_light_flag;
//						 if(ov5640_flash_light_flag == 0)
//							 OV5640_Flash_Ctrl(0);//关闭闪光灯; 
//						 else			 
//							 OV5640_Flash_Ctrl(1);//开启闪光灯;
//					 }			 
//					 key_0_pressed_flag = UNPRESSED;
//				}
//				if(key_CAMERA_FOCUS_pressed_flag == PRESSED)
//				{ 
//					 HAL_Delay(10);
//					 if(KEY_CAMERA_FOCUS == GPIO_PIN_RESET)
//					 {
//						  
////		          GUI_DrawRect(5,105,95,205);//  LCD_Draw_Circle(160,120,10);
////						  DCMI_Stop();
////						  LCD_Draw_Circle(160,120,20);
////						  DCMI_Start();
//						  beep_camera_focus_OK();
//							OV5640_Focus_Single();  //按KEY1手动单次自动对焦	
//					 }
//					 key_CAMERA_FOCUS_pressed_flag = UNPRESSED;
//				}
				if(key_CAMERA_TRIGGER_pressed_flag == PRESSED/* || key_CAMERA_FOCUS_pressed_flag == PRESSED*/)
				{			 
					 //OV5640_Focus_Single();  //按KEY1手动单次自动对焦	
					 //OV5640_Wait_Focus();
					if(OV5640_Wait_For_Focus())
					{
					 beep_take_pic_OK();
					 take_photo();
					 key_CAMERA_TRIGGER_pressed_flag = UNPRESSED;
					 key_CAMERA_FOCUS_pressed_flag = UNPRESSED; 
					}
				}
	  }
    else	
		{ 
			if(current_scan_mode == AUTOSCAN_MODE)
			{
				if(counter_scanner>2)
				{
					counter_scanner = 0;
					HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_SET);	
					HAL_Delay(50);
					HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_RESET);	
				}
			}
			else
			{
				if(KEY_CAMERA_TRIGGER==0) 
					HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_RESET);	
				else 
					HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_SET);	
			}
			scan_barcode();
		}	
    
	  if(key_INQUIRE_STAUTS_pressed_flag == PRESSED)
		{
			key_INQUIRE_STAUTS_pressed_flag = UNPRESSED;
			Message_USB[2] = STATUS_MODE;
			Message_USB[3] = 0xe0;
			if(!OV5640_CHECK()) Message_USB[3] = 0xe2;
			CDC_Transmit_HS_delay(Message_USB,4,300);
		}
		
		if(key_ENTER_pressed_flag == PRESSED)
		{
			key_ENTER_pressed_flag = UNPRESSED;
			Message_USB[2] = DEVICE_ENTER;
			CDC_Transmit_HS_delay(Message_USB,3,300);
		}
		
		if(key_PC_TRIGGER_pressed_flag == PRESSED)
		{
			key_PC_TRIGGER_pressed_flag = UNPRESSED;
			Message_USB[2] = TRIGGER_PHOTO;
			CDC_Transmit_HS_delay(Message_USB,3,300);
			key_CAMERA_TRIGGER_pressed_flag = PRESSED;
		}
		
		if(key_SWITCH_TO_ENABLE_USB_MODE_pressed_flag == PRESSED)
		{
				key_SWITCH_TO_ENABLE_USB_MODE_pressed_flag = UNPRESSED;
				Message_USB[2] = ENABLE_USB_MODE;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
				usb_comm_mode = 1;
		}
		
		if(key_SWITCH_TO_DISABLE_USB_MODE_pressed_flag == PRESSED)
		{
				key_SWITCH_TO_DISABLE_USB_MODE_pressed_flag = UNPRESSED;
				Message_USB[2] = DISABLE_USB_MODE;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
				usb_comm_mode = 0;
		}
			
		if(key_SWITCHING_LED_pressed_flag == PRESSED)
		{
			HAL_Delay(10);
			key_SWITCHING_LED_pressed_flag = UNPRESSED;
			if(lighting_led_mode == 1)
			{
				lighting_led_mode = 0; //close led
			  Message_USB[2] = LED_OFF;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
				OV5640_Flash_Ctrl(0);
				//HAL_GPIO_WritePin(LED_LIGHT_GPIO_Port,LED_LIGHT_Pin,GPIO_PIN_RESET);
			}
			else
			{
				lighting_led_mode = 1; //open led
			  Message_USB[2] = LED_ON;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
				OV5640_Flash_Ctrl(1);
				//HAL_GPIO_WritePin(LED_LIGHT_GPIO_Port,LED_LIGHT_Pin,GPIO_PIN_SET);
			}
		}
		
		if (key_SWITCH_TO_AUTOSCAN_MODE_pressed_flag == PRESSED)
		{
			key_SWITCH_TO_AUTOSCAN_MODE_pressed_flag = UNPRESSED;
			  Message_USB[2] = SWITCH_MODE_TO_AUTOSCAN;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
			if( current_scan_mode == SINGLESCAN_MODE)
			{
				counter_scanner = 3;
				current_scan_mode = AUTOSCAN_MODE;
			}
		}
		
		if (key_SWITCH_TO_SINGLESCAN_MODE_pressed_flag == PRESSED)
		{
			key_SWITCH_TO_SINGLESCAN_MODE_pressed_flag = UNPRESSED;
			  Message_USB[2] = SWITCH_MODE_TO_SINGLESCAN;
			  CDC_Transmit_HS_delay(Message_USB,3,300);
			if( current_scan_mode == AUTOSCAN_MODE)
			{ 
				current_scan_mode = SINGLESCAN_MODE;
			}
		}
		
		if (key_SWITCH_TO_PHOTO_MODE_pressed_flag == PRESSED)
		{
			key_SWITCH_TO_PHOTO_MODE_pressed_flag = UNPRESSED;
			Message_USB[2] = SWITCH_MODE_TO_PHOT0 ;
      CDC_Transmit_HS_delay(Message_USB,3,300); //    				  while(CDC_Transmit_HS(Message_USB,4)!=USBD_OK) ;//  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Message_USB, 4);	
			if(current_working_mode == BARCODE_MODE) key_SWITCHING_MODE_pressed_flag = PRESSED;
		}
		if (key_SWITCH_TO_BARCODE_MODE_pressed_flag == PRESSED)
		{
			key_SWITCH_TO_BARCODE_MODE_pressed_flag = UNPRESSED;
			Message_USB[2] = SWITCH_MODE_TO_QR  ;
      CDC_Transmit_HS_delay(Message_USB,3,300);//while(CDC_Transmit_HS(Message_USB,4)!=USBD_OK) ;//				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Message_USB, 4);	
			if(current_working_mode == PHOTO_MODE) key_SWITCHING_MODE_pressed_flag = PRESSED;	 
		}
		
		if(key_SWITCHING_MODE_pressed_flag == PRESSED)//
		{  
			 HAL_Delay(10);
			 //while(KEY_SWITCHING_MODE == GPIO_PIN_RESET);
		   if(current_working_mode == PHOTO_MODE)
			 { 
				  printf("switch to barcode mode.\n");
				  current_working_mode = BARCODE_MODE;
					LCD_Display_Dir(0);
				  close_camera();
//          GUI_Exit();   				 
//				  GUI_Init();
//				  CreateScanner();
//			    GUI_Exec();
				  GUI_DrawBitmap(&bmBarcode,0,0);
				  GUI_DrawBitmap(&bmscanning_pic,0,0);
				  scanning_line_y_axis = 60; //xuru disp
				  printf("remaining alloc:%d.\n",GUI_ALLOC_GetNumFreeBytes());				 
			 }
			 else
			 { 
				  HAL_GPIO_WritePin(scanner_TRIG_GPIO_Port,scanner_TRIG_GPIO_Pin,GPIO_PIN_SET);//close scanner module LED;
				   
				  printf("switch to photo mode.\n");
				  current_working_mode = PHOTO_MODE;	
				  key_CAMERA_TRIGGER_pressed_flag = UNPRESSED;
				  key_CAMERA_FOCUS_pressed_flag = UNPRESSED;
					
					LCD_Display_Dir(1);
				  open_camera();
				  printf("remaining alloc:%d.\n",GUI_ALLOC_GetNumFreeBytes());	
				  			 
			 }
			 key_SWITCHING_MODE_pressed_flag = UNPRESSED;
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
	printf("error occured.\n");
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
