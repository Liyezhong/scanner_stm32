#include "function.h"
#include "main.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
//#include "led.h"
#include "key.h"
#include "ILI9341.h"
#include "ltdc.h"
#include "sdram.h"
#include "malloc.h"
#include "ov5640.h" 
#include "dcmi.h"  
#include "pcf8574.h" 

#include "usb_device.h"
//#include "usbd_customhid.h"

#include "GUI.h"
#include "scannerDLG.h"
#include "uploadDLG.h"
#include "uploadsucessDLG.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "CRC.h"


u32 *jpeg_data_buf;						//JPEG数据缓存buf 
u8 ovx_mode=RGB565_MODE;							//bit0:0,RGB565模式;1,JPEG模式 
volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										//0,数据没有采集完;
										//1,数据采集完了,但是还没处理;
										//2,数据已经处理完成了,可以开始下一帧接收
u16 jpeg_current_line=0;							//摄像头输出数据,当前行编号
u16 jpeg_yoffset=0;							//y方向的偏移量


u32 *dcmi_line_buf[2];		//摄像头采用一行一行读取,定义行缓存  
//uint8_t photo_QRcode_mode = QRCODE_MODE; //当前工作模式是拍照还是二维码扫描.开机默认二维码扫描模式.
uint8_t ov5640_flash_light_flag = 0;//照相机闪光灯是否开机参数.

extern uint8_t USB_message_CRC_matched_flag;
extern GUI_CONST_STORAGE GUI_BITMAP bmBarcode;
extern GUI_CONST_STORAGE GUI_BITMAP bmLine;
extern GUI_CONST_STORAGE GUI_BITMAP bmscanning_pic;

extern UART_HandleTypeDef UART1_Handler; //UART句柄;
extern UART_HandleTypeDef huart5;
extern uint8_t rx_buffer_uart5[BARCODE_MAXIM_BYTE];
extern uint8_t rx_len_uart5;
extern uint8_t recv_end_flag_uart5;
extern char barcode[BARCODE_MAXIM_BYTE];
extern uint32_t counter_second;
extern uint32_t counter_scanner;

uint8_t Message_barcode_USB[128]={0xBB,0xDD,0x01};


struct USB_message message_qrcode;

uint16_t scanning_line_y_axis = 30;
uint8_t scanning_next_page_flag = 0;
uint8_t USB_sending_data_out_time_flag = 0;


//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
void jpeg_data_process(void)
{
	u16 i;
	u16 rlen;			//剩余数据长度
	u32 *pbuf; 
//	printf("jpeg data ok:%d.\n",jpeg_data_ok); //似乎这一句会影响接收效率;会提示接收到的JPEG数据不完整;
	jpeg_current_line=jpeg_yoffset;	//行数复位
	if(ovx_mode&JPEG_MODE)//只有在JPEG格式下,才需要做处理.
	{
		if(jpeg_data_ok==0)	//jpeg数据还未采集完?
		{
      __HAL_DMA_DISABLE(&DMADMCI_Handler);//关闭DMA
//// //      while(DMA2_Stream1->CR&0X01);	//等待DMA2_Stream1可配置 
//			
			rlen=jpeg_line_size-__HAL_DMA_GET_COUNTER(&DMADMCI_Handler);//得到剩余数据长度	
//			printf("remain length = %d.\n",rlen);
			pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾,继续添加
			if(DMADMCI_Handler.Instance->CR&(1<<19))
				for(i=0;i<rlen;i++)
			    pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的剩余数据
			else 
				for(i=0;i<rlen;i++)
			    pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的剩余数据 
			jpeg_data_len+=rlen;			//加上剩余长度
			jpeg_data_ok=1; 				//标记JPEG数据采集完按成,等待其他函数处理
//			printf("all the jpeg data transfered.\n");
		}
	}
  //else if(ovx_mode&RGB565_MODE)
  //{
//			printf("lcd frame:%d.\n",frame_counter++);
  //}	
}


//jpeg数据接收回调函数
void jpeg_dcmi_rx_callback(void)
{  
	u16 i;
	u32 *pbuf;
	pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾
	if(DMADMCI_Handler.Instance->CR&(1<<19))//buf0已满,正常处理buf1
	{ 
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移
	}else //buf1已满,正常处理buf0
	{
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移 
	} 
}

extern uint8_t data_to_send[64];
extern uint8_t USB_data_send_OK_flag;
extern uint8_t taking_picture_counter;

//uint8_t buf_test[20]="1234567890abcdefghi";
struct USB_message message_photo;
extern uint32_t USB_sending_pic_packet_length;

u8 ov5640_jpg_photo()
{
	u8 res=0,headok=0;
	u32 i,jpgstart,jpglen;
	u8* pbuf;
	u8 * ppbuf;
	u8 photo_try_counter;//try photo times. in case of failures.
	u8 USB_message_start[10];
	u8 CRC_not_match_counter;	
	uint32_t USB_DEBUG_counter;
	
	ovx_mode=JPEG_MODE;
	jpeg_data_ok=0;


//OV5640_WR_Reg(0x3000, 0xff);//reset MCU;
	
//	OV5640_WR_Reg(0x3000, 0x20);//reset MCU;

	
//OV5640_WR_Reg(0x3008,0x82);
	
//	delay_ms(50);

//	OV5640_RST=0;			//必须先拉低OV5640的RST脚,再上电
//	delay_ms(20); 
//	OV5640_RST=1;			//结束复位 
//	delay_ms(20); 
//	
//	OV5640_WR_Reg(0x3103,0X11);	//system clock from pad, bit[1]
//	OV5640_WR_Reg(0X3008,0X82);	//软复位

//  OV5640_Init();
//	
//	delay_ms(1000); 

	OV5640_JPEG_Mode();						//JPEG模式  
	
//	OV5640_JPEG_SETTING();

//		OV5640_OutSize_Set(16,4,1432,1944);
	OV5640_OutSize_Set(16,4,2592,1944);		//设置输出尺寸(500W) tested that only this setting can led to completed jpeg on PC.
	
//	OV5640_OutSize_Set(16,4,1280,800);//100W pixel
//	OV5640_OutSize_Set(16,4,2048,1536);//300W pixel
//	OV5640_OutSize_Set(16,4,2500,1800);
//  OV5640_OutSize_Set(16,4,160,120); //test

  delay_ms(50);//debug

	photo_try_counter = 3;
	while(photo_try_counter)
	{
		    myfree(SRAMEX,jpeg_data_buf);//release memory
		    jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//为jpeg文件申请内存(最大4MB)
		
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
				HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);	
		
				GUI_SetColor(GUI_GREEN);
				GUI_DrawRoundedFrame(145,105,175,135,5,2);
				//GUI_DrawRoundedFrame(105,145,135,175,5,2);//xuru disp
		
		    DCMI_Stop();
		    delay_ms(20);//debug
	      dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG接收数据回调函数
 	      DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,DMA_MDATAALIGN_WORD,DMA_MINC_ENABLE);//DCMI DMA配置     

		    jpeg_data_len=0;				    //数据重新开始	
				DCMI_Start(); 			//启动传输 
				DCMI->IER |= DCMI_IT_FRAME ; //使能frame中断, 否则hardware fault.
		
				while(jpeg_data_ok!=1)
				{;}	//等待第1帧图片采集完    
        jpeg_data_ok = 0;
		    jpeg_data_len=0;
		    __HAL_DMA_ENABLE(&DMADMCI_Handler);
		    while(jpeg_data_ok!=1)
				{;}	//等待第2帧图片采集完
        jpeg_data_ok = 0;
		    jpeg_data_len=0;
		    __HAL_DMA_ENABLE(&DMADMCI_Handler);
		    while(jpeg_data_ok!=1)
				{;}//等待第3帧图片采集完
//        jpeg_data_ok = 0;
//		    jpeg_data_len=0;
//		    __HAL_DMA_ENABLE(&DMADMCI_Handler);
//		    while(jpeg_data_ok!=1)
//				{;}//等待第4帧图片采集完

				DCMI_Stop(); 			//停止DMA搬运

				HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	
					
//					printf("jpeg data size:%d\r\n",jpeg_data_len*4);//串口打印JPEG文件大小
					pbuf=(u8*)jpeg_data_buf; 

					jpglen=0;	//设置jpg文件大小为0
					headok=0;	//清除jpg头标记
					for(i=0;i<jpeg_data_len*4;i++)//查找0XFF,0XD8和0XFF,0XD9,获取jpg文件大小
					{
						if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//找到FF D8
						{
							jpgstart=i;
							headok=1;	//标记找到jpg头(FF D8)
//							printf("found head.\n");
						}
						if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//找到头以后,再找FF D9
						{
							jpglen=i-jpgstart+2;
//							printf("found tail.\n");
							break;
						}
					}
					if(jpglen)			//正常的jpeg数据 
					{ 
						USB_sending_data_out_time_flag = 0;
						photo_try_counter = 0;
//						Createupload();    //添加正在上传界面
//						GUI_Exec();
						pbuf+=jpgstart;	//偏移到0XFF,0XD8处
						
						/*send by UART begin*/
			//			for(i=0;i < jpglen;i++)
			//			{  while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
			//					 USART1->DR =pbuf[i] ;		
			//			}
						/*send by UART end*/
						/*send by USB begin */
						ppbuf = (u8 *)(pbuf);
						message_photo.length = jpglen;
						
						printf("jpglen=%d. %x.\n",message_photo.length,message_photo.length);
						
						printf("ppbuf bufer start 5 byte to send:%x,%x,%x,%x,%x\n",ppbuf[0],ppbuf[1],ppbuf[2],ppbuf[3],ppbuf[4]);//debug
						printf("ppbuf bufer last 5 byte to send:%x,%x,%x,%x,%x\n",ppbuf[jpglen-5],ppbuf[jpglen-4],ppbuf[jpglen-3],ppbuf[jpglen-2],ppbuf[jpglen-1]);//debug
						
						message_photo.CRC_result = crc16(ppbuf,message_photo.length);
						printf("CRC value:%d.%x.\n",message_photo.CRC_result,message_photo.CRC_result);
						
						message_photo.start_byte[0] = 0xBB;
						message_photo.start_byte[1] = 0xDD;
						
						message_photo.message_type = UPLOAD_IMAGE_MESSAGE;
           				
            message_photo.ID ++;										
						
						USB_message_start[0] = message_photo.start_byte[0];
						USB_message_start[1] = message_photo.start_byte[1];
						USB_message_start[2] = message_photo.message_type;
						USB_message_start[3] = (message_photo.length&0xff000000)>>24;//message_photo.length/0xffffff;
						USB_message_start[4] = (message_photo.length &0xff0000)>>16;//message_photo.length %0xffffff/0xffff;
						USB_message_start[5] = (message_photo.length &0xff00)>>8;     //message_photo.length %0xffff/0xff;
						USB_message_start[6] = message_photo.length &0xff;   //message_photo.length %0xff;
						USB_message_start[7] = (message_photo.CRC_result &0xff00)>>8;//message_photo.CRC_result/0xff;
						USB_message_start[8] = message_photo.CRC_result&0xff;//message_photo.CRC_result%0xff;
						USB_message_start[9] = message_photo.ID;
						
						for(i=0;i<10;i++)
						{
						   printf("message[%d] = %x.\n",i,USB_message_start[i]);
						}	
						USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;
						CRC_not_match_counter = USB_MAX_RESEND_TIME;
            while(USB_message_CRC_matched_flag != USB_CRC_MATCHED) // keep on sending data until get CRC match USB info.
						{		
							  USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;	            					
                USB_sending_data_out_time_flag = 0;								
								while(CDC_Transmit_HS(USB_message_start,LENGTH_USB_PROTOCAL)!=USBD_OK)				
								{			
									 if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
									 {
											USB_sending_data_out_time_flag = 1;
											break;
									 }					   
								}
								if(USB_sending_data_out_time_flag == 1) break;
								
				//				delay_ms(5);//delay 5ms between sending protocal and result.
					// 			USB_DEBUG_counter = 0;
								printf("pic header sent.\n");
								while(NEED_USB_CRC_VERIFICATION&&(USB_message_CRC_matched_flag != USB_RECEIVED_PIC_HEADER))
								{
									 delay_ms(1);//necessary to have a delay in loop,otherwise will be always in loop,don't know why.
									 if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
									 {
											USB_sending_data_out_time_flag = 1;
										  printf("didn't get response for the header from host,exit.\n");
											break;
									 }					 
								}
								if(USB_sending_data_out_time_flag == 1) break;
								
								printf("received header response from host,begin sending jpg data packet.\n");
								while(jpglen&&(!USB_sending_data_out_time_flag))		
								{				
										 if(jpglen>USB_sending_pic_packet_length)//64-FS;512-HS
										 { // USB_message_CRC_matched_flag =  USB_CRC_NO_RESPONSE;//clear the host response flag before sending each packet.									 
												while(CDC_Transmit_HS(ppbuf,USB_sending_pic_packet_length)!=USBD_OK) 
												{
												   //delay_us(250);		
													 if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
													 {
														  USB_sending_data_out_time_flag = 1;
															printf("USB time out while sending picture.\n");
															break;
													 }											 
												}
												jpglen = jpglen - USB_sending_pic_packet_length;	
												ppbuf = (u8*)(ppbuf+USB_sending_pic_packet_length);	
                        //if(!NEED_USB_CRC_VERIFICATION)  delay_ms(1);  //delay_ms(1);prove 1ms works fine,but too slow. 300us not fine error when 110 times.											 
										    while(NEED_USB_CRC_VERIFICATION&&(USB_message_CRC_matched_flag != USB_RECEIVED_ONE_RIGHT_PIC_PACKET))
												{
													 delay_ms(1);
													 if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
													 {
														  USB_sending_data_out_time_flag = 1;
															printf("Did not receive the response for this packet,exit.\n");
															break;
													 }
                           if(USB_message_CRC_matched_flag == USB_CRC_UNMATCHED)//this packet is wrong.
													 {  printf("packet CRC wrong.\n");
													    break;														 
													 }
												}
												if(USB_message_CRC_matched_flag ==USB_RECEIVED_ONE_RIGHT_PIC_PACKET)
													USB_message_CRC_matched_flag = USB_CRC_NO_RESPONSE;//clear the CRC result.so we can wait next packet answer												
										 }
										 else
										 {
											  delay_ms(1);// delay_us(600);
											  USB_message_CRC_matched_flag = USB_CRC_NO_RESPONSE;//clear the last packet CRC result.
											  while(CDC_Transmit_HS(ppbuf,jpglen)!=USBD_OK) ;
											  break;
										 }
			               if(USB_message_CRC_matched_flag == USB_CRC_UNMATCHED)
											 break;	
								}				
								if(!NEED_USB_CRC_VERIFICATION) break;//if no need CRC, then break;//use when debuging only,not final product.
								if(USB_sending_data_out_time_flag!=1) printf("all JPEG packet sent to PC.waiting CRC result.\n");
				
							 while(USB_message_CRC_matched_flag != USB_CRC_MATCHED && USB_message_CRC_matched_flag != USB_CRC_UNMATCHED)//wait until get response;
							 {
								  delay_ms(1);//it is necessary to use this instead of nothing in loop.
								  if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
							   {
								    USB_sending_data_out_time_flag = 1;
								    break;
							   }
							 }
							 if(USB_message_CRC_matched_flag == USB_CRC_MATCHED)
							 {
									 printf("USB CRC matched.\n");
									 USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;
									 break;
							 }
							 else if(USB_message_CRC_matched_flag == USB_CRC_UNMATCHED)
							 {
								   printf("USB CRC unmatched.\n");
									 CRC_not_match_counter --;
								 	 printf("USB resend left:%d.\n",CRC_not_match_counter);	
									 if(CRC_not_match_counter == 0) 
									 {  printf("10 times CRC not match.exit.\n");
										  USB_sending_data_out_time_flag = 1;	//EXIT	
									 }						 												
									 ppbuf = (u8 *)(pbuf);
									 jpglen = message_photo.length ;
							 }
							 if(USB_sending_data_out_time_flag == 1) break;	
					}						
						
						/*send by USB end*/
						if(USB_sending_data_out_time_flag == 0)
						{   lcd_show_sucess_jpg();
								beep_scanning_QR_OK();
						}
						else
							lcd_show_USB_Error_jpg();
	//					delay_ms(400);
							//Createuploadsucess();//添加上传成功界面；		
						//	GUI_Exec();	
						}
					else 
					{
						photo_try_counter--;
						
						if(photo_try_counter == 0)
						{
							lcd_show_USB_Error();
						}
						
						res=0XFD;
						printf("incomplete data.try times:%d.\n",photo_try_counter);			
					}
					
						jpeg_data_ok=0;					    //标记数据未采集
						jpeg_data_len=0;				    //数据重新开始	
	}
	ovx_mode=RGB565_MODE; 
	OV5640_RGB565_Mode();	//RGB565模式  
	OV5640_OutSize_Set(16,4,lcddev.width,lcddev.height);//结束拍照后屏立马恢复显示.
	delay_ms(250);	//delay is necessary to remove the display rightdown bug;
	DCMI_DMA_Init((u32)&TFTLCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);	//DCMI DMA配置,MCU屏,竖屏
	return res;
}  

void take_photo()
{
	 printf("picture took.\n"); 
	 DCMI_Stop();//自己加;
   ov5640_jpg_photo();
//	 delay_ms(1000);		//等待1秒钟	
	 DCMI_Start();		//这里先使能dcmi,然后立即关闭DCMI,后面再开启DCMI,可以防止RGB屏的侧移问题. 
//				DCMI_Stop();	
//				DCMI_Start();//开始显示     
}

void take_photo_init()
{
	  	
		printf("current mode is photo.\n");
	  DCMI_Stop();
//	  atk_qr_destroy();//释放算法内存
	  
	  printf("3SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	  printf("3SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	  printf("3SRAM CCM:%d\r\n",my_mem_perused(SRAMCCM)); 	
//		LCD_Clear(YELLOW);						
		dcmi_line_buf[0]=mymalloc(SRAMIN,jpeg_line_size*4);	//为jpeg dma接收申请内存	
		dcmi_line_buf[1]=mymalloc(SRAMIN,jpeg_line_size*4);	//为jpeg dma接收申请内存	
		jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//为jpeg文件申请内存(最大4MB)
		DCMI_DMA_Init((u32)&TFTLCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);			//DCMI DMA配置,MCU屏,竖屏
		jpeg_yoffset=0;
		jpeg_current_line=jpeg_yoffset;		//行数复位
		OV5640_OutSize_Set(16,4,lcddev.width,lcddev.height);	//满屏缩放显示
		DCMI_Start(); 			//启动传输
}



void scan_barcode()
{
	 uint32_t i;
	 u8 USB_message_start[10];
	 u8 CRC_not_match_counter;
	
		if(scanning_next_page_flag == 1)
		{	
			  scanning_next_page_flag = 0;
			
		  	GUI_DrawBitmap(&bmBarcode,0,0);	
				//GUI_DrawBitmap(&bmscanning_pic,0,0); //xuru disp
			
				GUI_SetColor(GUI_CYAN);
				GUI_SetPenSize(2);
			
				GUI_DrawLine(30,scanning_line_y_axis,210,scanning_line_y_axis);
			  //GUI_DrawLine(40,scanning_line_y_axis,280,scanning_line_y_axis); //xuru disp
			
				//GUI_DrawBitmap(&bmLine,20,scanning_line_y_axis);
				scanning_line_y_axis+=5;
			  
				//if(scanning_line_y_axis>=220) scanning_line_y_axis = 60; //xuru disp
				if(scanning_line_y_axis >= 290) 
					scanning_line_y_axis = 30;
			
				GUI_Exec();
		}	 
	  if(recv_end_flag_uart5 == 1)		
		{
			 counter_scanner = 3;
			 USB_sending_data_out_time_flag = 0;
//			 beep_scanning_QR_OK();
//			 delay_ms(200);	//延时不够会接收不全;		 100 tested will lose packet;
			 HAL_UART_Transmit(&UART1_Handler,rx_buffer_uart5,rx_len_uart5,5000);
//       strcpy(barcode,rx_buffer_uart5);
//       CreateScanner();
//			 GUI_Exec();
			
			 GUI_DrawBitmap(&bmBarcode,0,0);
			 LCD_ShowString(15,253,210,20,16,rx_buffer_uart5);//show 2 row maxim.
			 //GUI_DrawBitmap(&bmscanning_pic,0,0);  //xuru disp
			 //LCD_ShowString(20,190,280,20,16,rx_buffer_uart5);//show 2 row maxim.
			
//			 Message_barcode_USB[3] = rx_len_uart5 ;
//       for(i=0;i<rx_len_uart5;i++)
//          Message_barcode_USB[4+i] = 	rx_buffer_uart5[i];
//       Message_barcode_USB[4+rx_len_uart5] = 0xFF;			

			      message_qrcode.length = rx_len_uart5;
						message_qrcode.CRC_result = crc16(rx_buffer_uart5,message_qrcode.length);
						printf("CRC value:%d.%x.\n",message_qrcode.CRC_result,message_qrcode.CRC_result);
						
						message_qrcode.start_byte[0] = 0xBB;
						message_qrcode.start_byte[1] = 0xDD;
						
						message_qrcode.message_type = UPLOAD_QR_MESSAGE;
           				
            message_qrcode.ID ++;	
            message_qrcode.buffer = (u8*)rx_buffer_uart5;
						
						USB_message_start[0] = message_qrcode.start_byte[0];
						USB_message_start[1] = message_qrcode.start_byte[1];
						USB_message_start[2] = message_qrcode.message_type;
						USB_message_start[3] = (message_qrcode.length&0xff000000)>>24;//message_photo.length/0xffffff;
						USB_message_start[4] = (message_qrcode.length &0xff0000)>>16;//message_photo.length %0xffffff/0xffff;
						USB_message_start[5] = (message_qrcode.length &0xff00)>>8;     //message_photo.length %0xffff/0xff;
						USB_message_start[6] = message_qrcode.length &0xff;   //message_photo.length %0xff;
						USB_message_start[7] = (message_qrcode.CRC_result &0xff00)>>8;//message_photo.CRC_result/0xff;
						USB_message_start[8] = message_qrcode.CRC_result&0xff;//message_photo.CRC_result%0xff;
						USB_message_start[9] = message_qrcode.ID;
						
						for(i=0;i<10;i++)
						{
						   printf("message[%d] = %x.\n",i,USB_message_start[i]);
						}					
						
						USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;
						CRC_not_match_counter = USB_MAX_RESEND_TIME;
            while(USB_message_CRC_matched_flag != USB_CRC_MATCHED) // keep on sending data until get CRC match USB info.
						{		
							 USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;	
               					
               USB_sending_data_out_time_flag = 0;							
						   while(CDC_Transmit_HS(USB_message_start,LENGTH_USB_PROTOCAL)!=USBD_OK)
						   {
							   if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
							   {
								    USB_sending_data_out_time_flag = 1;
								    break;
							   }
					     }
							 
									
		//	 if(CDC_Transmit_HS_delay(USB_message_start,LENGTH_USB_PROTOCAL,5000)==0)	
     //    printf("USB no response.exit\n");	
       //add LCD windows. 			 
			
//       CDC_Transmit_HS_delay(message_qrcode.buffer,message_qrcode.length,100);

				       delay_ms(5);//delay 5ms between sending protocal and result.
							 while(CDC_Transmit_HS(rx_buffer_uart5,rx_len_uart5)!= USBD_OK)
							 {
							    if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
									{
										 USB_sending_data_out_time_flag = 1;
								     break;
									}										
							 }
							 
//               if(USB_sending_data_out_time_flag == 0) 
//                  while(CDC_Transmit_HS(rx_buffer_uart5,rx_len_uart5)!=USBD_OK);								 

							 if(!NEED_USB_CRC_VERIFICATION) break;//if no need CRC, then break;//use when debuging only,not final product.

						   printf("barcode message sent,waiting CRC result.\n");

							 while(USB_message_CRC_matched_flag != USB_CRC_MATCHED && USB_message_CRC_matched_flag != USB_CRC_UNMATCHED)//wait until get response;
							 {
								  delay_ms(1);//it is necessary to use this instead of nothing in loop.
								  if(counter_second >= SECOND_USB_SENDING_OUT_TIME_EXIT)
							    {
								    USB_sending_data_out_time_flag = 1;
								    break;
							    }
							 }
							 if(USB_message_CRC_matched_flag == USB_CRC_MATCHED)
							 {
									printf("USB CRC matched.\n");
									USB_message_CRC_matched_flag= USB_CRC_NO_RESPONSE;
									break;
							 }
               else	if(USB_message_CRC_matched_flag == USB_CRC_UNMATCHED)	
							 {	
                  printf("USB CRC unmatched.\n");								 
									CRC_not_match_counter --;
 									printf("USB resend left:%d.\n",CRC_not_match_counter);	
									if(CRC_not_match_counter == 0) 
									{  printf("10 times CRC not match.exit.\n");
										 USB_sending_data_out_time_flag = 1;		
									}				
							 }
							 if(USB_sending_data_out_time_flag == 1) break;								 
					 }
					 if(USB_sending_data_out_time_flag == 0)
           {   						  
						  lcd_show_sucess();
						  beep_scanning_QR_OK();
					 }
					 else
						  lcd_show_USB_Error();             						 
           delay_ms(500);					 
					 for(i=0;i<BARCODE_MAXIM_BYTE;i++)
					 {
						 rx_buffer_uart5[i]=0;
						 barcode[i]=0;
					 }
					 recv_end_flag_uart5 = 0;
					 rx_len_uart5 = 0;				 				 
		}
}

void open_camera()
{
	
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池    	
	
	DCMI->CR|=DCMI_CR_ENABLE;
//	DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能
  take_photo_init();	
	DCMI->IER |=DCMI_IT_FRAME;   
}

void close_camera()
{
  DCMI_Stop();
}

//#define USB_MAXIM_DATA_TO_BE_SEND 64

//void send_JPEG_by_USB(uint32_t length, uint8_t *data_to_send)
//{
//    if(length>USB_MAXIM_DATA_TO_BE_SEND)
//			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, "1234567890", 10); 
//}

extern GUI_CONST_STORAGE GUI_BITMAP bmemwin_demo;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLeicalogo;
extern GUI_CONST_STORAGE GUI_BITMAP bmLeicalogo320x240;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo;
extern GUI_CONST_STORAGE GUI_BITMAP bmok;
extern GUI_CONST_STORAGE GUI_BITMAP bmerr;
extern GUI_CONST_STORAGE GUI_BITMAP bmok2;
extern GUI_CONST_STORAGE GUI_BITMAP bmerr2;
extern GUI_CONST_STORAGE GUI_BITMAP bmcolorbands;
void emwin_demo()
{
				  
//	GUI_SetBkColor(GUI_GRAY);		//设置背景颜色
//	GUI_Clear();					//清屏
//	GUI_SetFont(&GUI_Font32B_ASCII); //设置字体
//	GUI_SetColor(GUI_DARKCYAN);       //设置前景色(如文本，画线等颜色)
//	GUI_GotoXY(10,10);// Set text position (in pixels)
	
//	  GUI_SetOrientation(GUI_MIRROR_X|GUI_MIRROR_Y);  //xuru
//	GUI_DispString("Initing... ");  
//	HAL_Delay(1000);
//	GUI_Clear();					//清屏
	
//	GUI_SetFont(&GUI_Font32B_1); //设置字体
//	GUI_SetColor(GUI_DARKRED); 
//	GUI_GotoXY(0,0);// Set text position (in pixels)
//	GUI_DispString("beach I love you so much."); 
//	
//  GUI_DispStringInRectEx("beach I love you so much.",&Rect,GUI_TA_VCENTER | GUI_TA_HCENTER,30,GUI_ROTATE_0);
//	GUI_DrawBitmap(&bmcolorbands,0,0);
//	HAL_Delay(1000);
	
	GUI_DrawBitmap(&bmLogo,0,0);
//	GUI_DrawBitmap(&bmLeicalogo320x240,0,0); //xuru Disp
//	CreateScanner();

//  GUI_DrawBitmap(&bmscanning_pic,0,0);
	
//	excute_window();
	
//	GUI_Exec();
	
//	while(1); //debug
}

//extern UART_HandleTypeDef huart5;
//uint8_t start_decoding[]={}
//void Barcode_start_decode()
//{
//   HAL_UART_Transmit(&huart5,);
//}

IWDG_HandleTypeDef IwdgHandle;

//Tout = ((4xPrescaler)*Reload)/32 (ms)
//Tout = (4x32*1000)/32 =4000ms;
void watchdog_init()
{
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)!= RESET)
	{
		printf("restart from iwdg.\n");
	   __HAL_RCC_CLEAR_RESET_FLAGS();
	}		
	IwdgHandle.Instance = IWDG;
	IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
	IwdgHandle.Init.Reload = 1000; // set IWDG period = 4s
	HAL_IWDG_Init(&IwdgHandle);
	
}

//xuru disp
void lcd_show_sucess()
{
	  GUI_SetColor(0x00d8a030);   //11011 101000 00110 d8	a0 6
    GUI_FillCircle(120,160,30);//(120,160,50);//(160,120,50);
		//GUI_SetColor(GUI_WHITE);
		//GUI_SetPenSize(12);
		//GUI_DrawLine(100,160,110,180);	//(100,160,110,180);//(140,120,150,140);	
	
		//GUI_SetPenSize(12);
		//GUI_DrawLine(110,180,150,150); //(110,180,150,150);//(150,140,190,110);	

		GUI_DrawBitmap(&bmok,102,147);	
	
	  GUI_Exec();
}

void lcd_show_USB_Error()
{//120- 160   3
	
		const GUI_POINT aPoints[] = {
		{ 0, 0},
		{ -32, 55},
		{ 32, 55}
		};

	  GUI_SetColor(0x00301ce8); //00110 000111 11101 30 1c e8
		GUI_FillPolygon(aPoints, GUI_COUNTOF(aPoints), 120, 110);
	
	
    //GUI_FillCircle(120,160,50);//(120,160,50);//(160,120,50); 
	  //GUI_SetBkColor(GUI_RED);
	  //GUI_SetColor(GUI_YELLOW);
	  //GUI_SetFont(&GUI_Font24_1);
	  //GUI_DispStringAt("USB",100,130);//("USB",100,130);//("USB",140,90);

	  //GUI_SetColor(GUI_YELLOW);
	  //GUI_SetFont(&GUI_Font24_1);
	  //GUI_DispStringAt("ERROR",84,160);//("ERROR",84,160);//("ERROR",124,120);
		
		GUI_DrawBitmap(&bmerr,115,130);		
	  GUI_Exec();
}

void lcd_show_sucess_jpg()
{
	  GUI_SetColor(0x00d8a030);   //11011 101000 00110 d8	a0 6
    GUI_FillCircle(160,120,30);//(120,160,50);//(160,120,50);
		//GUI_SetColor(GUI_WHITE);
		//GUI_SetPenSize(12);
		//GUI_DrawLine(100,160,110,180);	//(100,160,110,180);//(140,120,150,140);	
	
		//GUI_SetPenSize(12);
		//GUI_DrawLine(110,180,150,150); //(110,180,150,150);//(150,140,190,110);	

		GUI_DrawBitmap(&bmok2,147,102);	
	
	  GUI_Exec();
}

void lcd_show_USB_Error_jpg()
{//120- 160   3
	
		const GUI_POINT aPoints[] = {
		{ 0, 0},
		{ -55, -32},
		{ -55, 32}
		};

	  GUI_SetColor(0x00301ce8); //00110 000111 11101 30 1c e8
		GUI_FillPolygon(aPoints, GUI_COUNTOF(aPoints), 180, 120);
	
	
    //GUI_FillCircle(120,160,50);//(120,160,50);//(160,120,50); 
	  //GUI_SetBkColor(GUI_RED);
	  //GUI_SetColor(GUI_YELLOW);
	  //GUI_SetFont(&GUI_Font24_1);
	  //GUI_DispStringAt("USB",100,130);//("USB",100,130);//("USB",140,90);

	  //GUI_SetColor(GUI_YELLOW);
	  //GUI_SetFont(&GUI_Font24_1);
	  //GUI_DispStringAt("ERROR",84,160);//("ERROR",84,160);//("ERROR",124,120);
		
		GUI_DrawBitmap(&bmerr2,130,115);		
	  GUI_Exec();
}
