/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "delay.h"
#include "gpio.h"
#include "function.h"
#include "ILI9341.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

#define USB_MAX_LENGTH 20

uint8_t USB_Receive[USB_MAX_LENGTH];
extern uint8_t key_SWITCHING_MODE_pressed_flag ;
extern uint8_t key_SWITCHING_LED_pressed_flag;
extern uint8_t key_SWITCH_TO_PHOTO_MODE_pressed_flag;
extern uint8_t key_SWITCH_TO_BARCODE_MODE_pressed_flag;
extern uint8_t key_SWITCH_TO_AUTOSCAN_MODE_pressed_flag;
extern uint8_t key_SWITCH_TO_SINGLESCAN_MODE_pressed_flag;
extern uint8_t key_SWITCH_TO_ENABLE_USB_MODE_pressed_flag;
extern uint8_t key_SWITCH_TO_DISABLE_USB_MODE_pressed_flag;
extern uint8_t key_PC_TRIGGER_pressed_flag;
extern uint8_t key_INQUIRE_STAUTS_pressed_flag;
extern uint32_t counter_second ;
extern uint8_t system_current_status;
extern uint8_t current_working_mode;
extern uint8_t switch_power_mode;
extern uint8_t lighting_led_mode;

uint8_t USB_message_CRC_matched_flag = USB_CRC_NO_RESPONSE;
uint32_t USB_sending_pic_packet_length = USB_SENDING_PIC_PACKET_MAX_LENGTH;

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
	uint8_t i;
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
	counter_second = 0;
	printf("usb receive: %s.length: %d.\n",Buf,*Len);
	for(i=0;i<*Len;i++)
	{
	   USB_Receive[i] = Buf[i];
	}
	if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x03)
     key_SWITCH_TO_BARCODE_MODE_pressed_flag = PRESSED;//key_SWITCHING_MODE_pressed_flag = PRESSED;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x04)
		 key_SWITCH_TO_PHOTO_MODE_pressed_flag = PRESSED;//key_SWITCHING_MODE_pressed_flag = PRESSED;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x01)
		 USB_message_CRC_matched_flag = USB_CRC_MATCHED;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x00)
		 USB_message_CRC_matched_flag = USB_CRC_UNMATCHED;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x02)
		 USB_message_CRC_matched_flag = USB_RECEIVED_ONE_RIGHT_PIC_PACKET;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x05) //sleep
		 switch_power_mode = 1;
	else if(system_current_status == SLEEPING && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x06) //awake
		 switch_power_mode = 1;
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x07) //open led
		 {if (lighting_led_mode == 0) key_SWITCHING_LED_pressed_flag = PRESSED;}
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x08) // close led
		 {if(lighting_led_mode == 1) key_SWITCHING_LED_pressed_flag = PRESSED;}
	else if(system_current_status == AWAKE && current_working_mode == BARCODE_MODE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0A) // start autoscan
		 {key_SWITCH_TO_AUTOSCAN_MODE_pressed_flag = PRESSED;}
	else if(system_current_status == AWAKE && current_working_mode == BARCODE_MODE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0B) // stop autoscan
		 {key_SWITCH_TO_SINGLESCAN_MODE_pressed_flag = PRESSED;}
  else if(USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0C) // start autoscan
		 {key_SWITCH_TO_ENABLE_USB_MODE_pressed_flag = PRESSED;}
	else if(USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0D) // stop autoscan
		 {key_SWITCH_TO_DISABLE_USB_MODE_pressed_flag = PRESSED;}
	else if(system_current_status == AWAKE && current_working_mode == PHOTO_MODE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0E) // stop autoscan
		 {key_PC_TRIGGER_pressed_flag = PRESSED;}
	else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x0F) // stop autoscan
		 {key_INQUIRE_STAUTS_pressed_flag = PRESSED;}
	//else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x07)
	//{	 USB_message_CRC_matched_flag = USB_RECEIVED_PIC_HEADER;
	//   USB_sending_pic_packet_length = USB_Receive[3]*256+USB_Receive[4];
	//	 printf("host set the packet length to:%d.\n",USB_sending_pic_packet_length);
	//	 if(USB_sending_pic_packet_length > USB_SENDING_PIC_PACKET_MAX_LENGTH)
	//		 USB_sending_pic_packet_length = USB_SENDING_PIC_PACKET_MAX_LENGTH;
	//}
	//else if(system_current_status == AWAKE && USB_Receive[0] == 0xAA && USB_Receive[1] == 0xDD && USB_Receive[2] == 0x08)
	//   LCD_C5_TEST(USB_Receive[3],USB_Receive[4]);
	
	for(i=0;i<*Len;i++)
	{
		 Buf[i] = 0;
	   USB_Receive[i] = 0;//clear data;
	}
	
  return (USBD_OK);
  /* USER CODE END 11 */
}
extern uint8_t usb_comm_mode;
/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
	if (usb_comm_mode == 0) return 0;
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t CDC_Transmit_HS_delay(uint8_t* Buf, uint16_t Len,int ms)
{
	 uint8_t result = 1;
   while(CDC_Transmit_HS(Buf,Len) != USBD_OK)
	 {
	    if(ms<=0)
			{
//			   printf("usb no response,exit.\n");
				 result = 0;
				 break;
			}
			else
			{
			   delay_ms(1);
				 ms--;
			}	
	 }
	 return result;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
