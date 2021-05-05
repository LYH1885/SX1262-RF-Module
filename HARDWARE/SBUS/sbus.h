#ifndef _SBUS_H
#define _SBUS_H

#ifdef __cplusplus
 extern "C" {
#endif
	 

#include "stdio.h"	
#include "stm32f10x_conf.h"
#include "sys.h" 
#include "stdbool.h" 

#define ENDBYTEIDENT	//定义则判断尾字节是否正确
#define SENDINTERVAL	7 //发送时间间隔 毫秒
#define RESIVETIMEOUT 4//接收超时时间 4 * SENDINTERVAL = 28ms
typedef	struct CHS	//通道变量 C语言结构体可以直接赋值
{
	float ch[16]; //范围 -1~1
	struct
	{
		bool ch17 :1;				//二值通道17
		bool ch18 :1;				//二值通道18
		bool framlost :1;		//信号丢失标志，此时接收机（SBUS数据发送端）红灯亮
		bool failsafe	:1;		//故障保护 激活信号
		uint8_t :4;
	}flage;
}CHS;

extern CHS Wchs; 	//直接在这个结构体写入通道信号 0~15 共 16个通道 ，每个通道 范围 -1~1，用于发送SBUS信号
CHS ReadSBUS(void);	//读取信号
void SBUS_init(void);		
	 
#ifdef __cplusplus
 }
#endif

#endif
