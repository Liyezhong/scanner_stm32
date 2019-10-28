#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define BARCODE_MAXIM_BYTE 512

#define jpeg_buf_size   6*1024*1024		//定义JPEG数据缓存jpeg_buf的大小(6M字节)
#define jpeg_line_size	8 *1024			//定义DMA接收数据时,一行数据的最大值
#define RGB565_MODE 0
#define JPEG_MODE 1

#define BYTE_PER_PACKET 32
#define SECOND_USB_SENDING_OUT_TIME_EXIT 10 // USB no response 20 second exit.
#define USB_MAX_RESEND_TIME 2

void jpeg_data_process(void);
void jpeg_dcmi_rx_callback(void);
u8 ov5640_jpg_photo(void);
void take_photo(void);
void take_photo_init(void);
void scan_barcode(void);
void open_camera(void);
void close_camera(void);
void emwin_demo(void);
void watchdog_init(void);
void lcd_show_sucess(void);
void lcd_show_USB_Error(void);
void lcd_show_sucess_jpg(void);
void lcd_show_USB_Error_jpg(void);

enum COMMAND_TYPE
{
   UPLOAD_QR_MESSAGE = 1,
	 UPLOAD_IMAGE_MESSAGE,
	 SWITCH_MODE_TO_QR,
	 SWITCH_MODE_TO_PHOT0,
	 DEVICE_GO_TO_SLEEP,
	 DEVICE_WAKE_UP,
	 LED_ON,
	 LED_OFF,
	 DEVICE_ENTER,
	 SWITCH_MODE_TO_AUTOSCAN,
	 SWITCH_MODE_TO_SINGLESCAN,
	 ENABLE_USB_MODE,
	 DISABLE_USB_MODE,
	 TRIGGER_PHOTO,
	 STATUS_MODE
};

struct USB_message
{
	uint8_t start_byte[2];
  uint8_t message_type;
	uint8_t ID;
	uint32_t length;//MSB
	uint16_t CRC_result;//MSB
	uint8_t * buffer;
};

#define LENGTH_USB_PROTOCAL 10


#define USB_CRC_NO_RESPONSE 0
#define USB_CRC_UNMATCHED 1
#define USB_CRC_MATCHED 2
#define USB_RECEIVED_PIC_HEADER 3
#define USB_RECEIVED_ONE_RIGHT_PIC_PACKET 4

#define NEED_USB_CRC_VERIFICATION 0

#define USB_SENDING_PIC_PACKET_MAX_LENGTH 504   //512-8

#endif
