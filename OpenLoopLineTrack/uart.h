#ifndef __UART_H
#define __UART_H

#include <Arduino.h>

//串口选择宏定义入口
#define WL_SERIAL 		Serial2	//串口2

void uart_Init();
void Read_Data(unsigned int *Data);

#endif


