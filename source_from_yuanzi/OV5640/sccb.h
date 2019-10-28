#ifndef __SCCB_H
#define __SCCB_H
#include "sys.h"
	
//IO方向设置
#define SCCB_SDA_IN()  {GPIOB->MODER&=~(3<<(3*2));GPIOB->MODER|=0<<3*2;}	//PB3输入模式
#define SCCB_SDA_OUT() {GPIOB->MODER&=~(3<<(3*2));GPIOB->MODER|=1<<3*2;}    //PB3输出模式
//IO操作
#define SCCB_SCL        PBout(4)    //SCL
#define SCCB_SDA        PBout(3)    //SDA

#define SCCB_READ_SDA   PBin(3)     //输入SDA 


///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
u8 SCCB_WR_Byte(u8 dat);
u8 SCCB_RD_Byte(void);
#endif

