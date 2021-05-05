/*
#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持

#define DMA_Rec_Len 25
#define DMA_Tx_Len 25

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);


#endif

*/

#ifndef _USART_H_
#define _USART_H_
#include "sys.h" 

#define DMA_Rec_Len 25
#define DMA_Tx_Len 25

#define    USART1_DMA                    1   //开启注意冲突
 
#define    USART1_MAX_TX_LEN           25   //最大发送缓存字节数
#define    USART1_MAX_RX_LEN           25  //最大接收缓存字节数
extern u8  USART1_TX_BUF[USART1_MAX_TX_LEN]; //发送缓冲区
extern u8  USART1_RX_BUF[USART1_MAX_RX_LEN]; //接收缓冲区
extern volatile u16 USART1_RX_STA;           //接收状态标记	    		             

__packed typedef struct
{
	//静态
	USART_TypeDef *                  USARTx ;                //串口
	uint32_t                         USARTx_RCC ;	         //串口时钟时能
	uint32_t                         GPIO_RCC_APBx;          //GPIO时钟时能
	GPIO_TypeDef  *                  GPIO_TX_RX_Pin_TypeDef; //GPIOx
	uint16_t                         GPIO_TX_Pin; 
	uint16_t                         GPIO_RX_Pin;  
	uint8_t                          USARTx_IRQn;            //串口中断线
#if USART1_DMA==1 
	uint8_t                          USART_TX_DMA_IRQn_NVIC; //DMA发送通道优先级
	DMA_Channel_TypeDef *            USART_TX_DMA;           //DMA发送通道  
	uint8_t                          USART_TX_DMA_IRQn;      //DMA发送中断线
	uint32_t                         USART_TX_FLAG_GL;       //DMA发送所有标记
	uint32_t                         USART_TX_FLAG_TC;       //DMA传输完成标记

	uint8_t                          USART_RX_DMA_IRQn_NVIC; //DMA接收通道优先级
	DMA_Channel_TypeDef *            USART_RX_DMA;           //DMA接收通道  
	uint8_t                          USART_RX_DMA_IRQn;      //DMA接收中断线
	uint32_t                         USART_RX_FLAG_GL;       //DMA接收所有标记
	uint32_t                         USART_RX_FLAG_TC;       //DMA传输完成标记
	//动态
	volatile u8                      Tx_flag;                //DMA忙碌标志,1	
	volatile u8                      Rx_flag;                //DMA接收完成标志,0
#endif	
}USART1_InitTypeDef ;        //串口DMA接收不定长数据结构体
extern USART1_InitTypeDef    USART1_InitStructure;
extern DMA_InitTypeDef       DMA_USART1_InitStructure;

void   USART1_Init(u32 bound);
void   USART1_printf (char *fmt, ...);
void   USART1_Send_Array (u8 *array,u16 num);
 
#if USART1_DMA==1
extern DMA_InitTypeDef DMA_USART1_InitStructure;//为了切换缓冲区修改地址，定义成全局的
void   DMA_USART1_Init(void);//DMA串口配置
void   DMA_USART1_Tx_Data(u16 size);
#endif
 
#endif
