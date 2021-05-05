#ifndef _SBUS_H
#define _SBUS_H

#ifdef __cplusplus
 extern "C" {
#endif
	 

#include "stdio.h"	
#include "stm32f10x_conf.h"
#include "sys.h" 
#include "stdbool.h" 

#define ENDBYTEIDENT	//�������ж�β�ֽ��Ƿ���ȷ
#define SENDINTERVAL	7 //����ʱ���� ����
#define RESIVETIMEOUT 4//���ճ�ʱʱ�� 4 * SENDINTERVAL = 28ms
typedef	struct CHS	//ͨ������ C���Խṹ�����ֱ�Ӹ�ֵ
{
	float ch[16]; //��Χ -1~1
	struct
	{
		bool ch17 :1;				//��ֵͨ��17
		bool ch18 :1;				//��ֵͨ��18
		bool framlost :1;		//�źŶ�ʧ��־����ʱ���ջ���SBUS���ݷ��Ͷˣ������
		bool failsafe	:1;		//���ϱ��� �����ź�
		uint8_t :4;
	}flage;
}CHS;

extern CHS Wchs; 	//ֱ��������ṹ��д��ͨ���ź� 0~15 �� 16��ͨ�� ��ÿ��ͨ�� ��Χ -1~1�����ڷ���SBUS�ź�
CHS ReadSBUS(void);	//��ȡ�ź�
void SBUS_init(void);		
	 
#ifdef __cplusplus
 }
#endif

#endif
