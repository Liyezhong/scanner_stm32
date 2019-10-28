#ifndef _KEY_H
#define _KEY_H
#include "sys.h"


//下面的方式是通过位带操作方式读取IO
//#define KEY0        PHin(3) //KEY0按键PH3
//#define KEY1        PHin(2) //KEY1按键PH2
//#define KEY2        PCin(13)//KEY2按键PC13
//#define WK_UP       PAin(0) //WKUP按键PA0


//下面的方式是通过直接操作HAL库函数方式读取IO



//#define KEY0        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3)  //KEY0按键PH3
//#define KEY_CAMERA_FOCUS        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2)  //KEY1按键PH2
//#define KEY_CAMERA_TRIGGER        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2按键PC13
//#define KEY_SWITCHING_MODE       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0




//#define KEY_SCANNING_QR     HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11)     //PH11
#define KEY_CAMERA_TRIGGER  HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_10)  //PH10
#define KEY_CAMERA_FOCUS    HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11)      //PH9
#define KEY_SWITCHING_MODE  HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_9) //PH11

//#define PIN_SCANNING_QR GPIO_PIN_11
#define PIN_CAMERA_TRIGGER GPIO_PIN_10
#define PIN_CAMERA_FOCUS GPIO_PIN_11
#define PIN_SWITCHING_MODE GPIO_PIN_9



//#define KEY0_PRES 	1
//#define KEY1_PRES		2
//#define KEY2_PRES		3
//#define WKUP_PRES   4

void KEY_Init(void);
u8 KEY_Scan(u8 mode);
#endif
