/*
#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��

#define DMA_Rec_Len 25
#define DMA_Tx_Len 25

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);


#endif

*/

#ifndef _USART_H_
#define _USART_H_
#include "sys.h" 

#define DMA_Rec_Len 25
#define DMA_Tx_Len 25

#define    USART1_DMA                    1   //����ע���ͻ
 
#define    USART1_MAX_TX_LEN           25   //����ͻ����ֽ���
#define    USART1_MAX_RX_LEN           25  //�����ջ����ֽ���
extern u8  USART1_TX_BUF[USART1_MAX_TX_LEN]; //���ͻ�����
extern u8  USART1_RX_BUF[USART1_MAX_RX_LEN]; //���ջ�����
extern volatile u16 USART1_RX_STA;           //����״̬���	    		             

__packed typedef struct
{
	//��̬
	USART_TypeDef *                  USARTx ;                //����
	uint32_t                         USARTx_RCC ;	         //����ʱ��ʱ��
	uint32_t                         GPIO_RCC_APBx;          //GPIOʱ��ʱ��
	GPIO_TypeDef  *                  GPIO_TX_RX_Pin_TypeDef; //GPIOx
	uint16_t                         GPIO_TX_Pin; 
	uint16_t                         GPIO_RX_Pin;  
	uint8_t                          USARTx_IRQn;            //�����ж���
#if USART1_DMA==1 
	uint8_t                          USART_TX_DMA_IRQn_NVIC; //DMA����ͨ�����ȼ�
	DMA_Channel_TypeDef *            USART_TX_DMA;           //DMA����ͨ��  
	uint8_t                          USART_TX_DMA_IRQn;      //DMA�����ж���
	uint32_t                         USART_TX_FLAG_GL;       //DMA�������б��
	uint32_t                         USART_TX_FLAG_TC;       //DMA������ɱ��

	uint8_t                          USART_RX_DMA_IRQn_NVIC; //DMA����ͨ�����ȼ�
	DMA_Channel_TypeDef *            USART_RX_DMA;           //DMA����ͨ��  
	uint8_t                          USART_RX_DMA_IRQn;      //DMA�����ж���
	uint32_t                         USART_RX_FLAG_GL;       //DMA�������б��
	uint32_t                         USART_RX_FLAG_TC;       //DMA������ɱ��
	//��̬
	volatile u8                      Tx_flag;                //DMAæµ��־,1	
	volatile u8                      Rx_flag;                //DMA������ɱ�־,0
#endif	
}USART1_InitTypeDef ;        //����DMA���ղ��������ݽṹ��
extern USART1_InitTypeDef    USART1_InitStructure;
extern DMA_InitTypeDef       DMA_USART1_InitStructure;

void   USART1_Init(u32 bound);
void   USART1_printf (char *fmt, ...);
void   USART1_Send_Array (u8 *array,u16 num);
 
#if USART1_DMA==1
extern DMA_InitTypeDef DMA_USART1_InitStructure;//Ϊ���л��������޸ĵ�ַ�������ȫ�ֵ�
void   DMA_USART1_Init(void);//DMA��������
void   DMA_USART1_Tx_Data(u16 size);
#endif
 
#endif
