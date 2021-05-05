#include "delay.h"
#include "sys.h"
#include "led.h"
#include "usart.h"
#include "sx1262.h"
#include "timer.h"
//������

extern int RF_sent_flag;
extern int TIM3_send_flag;
extern u8  USART1_TX_BUF[USART1_MAX_TX_LEN];    //���ͻ���,���USART1_MAX_TX_LEN�ֽ�
extern u8  USART1_RX_BUF[USART1_MAX_RX_LEN];    //���ջ���,���USART1_MAX_RX_LEN�ֽ�


 int main(void)
 {	
	u16 times=0;
	
	delay_init(); //��ʱ������ʼ��
	RFInit();	 //��Ƶģ���ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	LED_Init(); //LED�˿ڳ�ʼ��
	USART1_Init(100000); //��ʼ�����ڣ�DMA���ͺͿ����жϽ���ģʽ��
	TIM3_Int_Init(109,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms

	while(1)
	{
		
		if(TIM3_send_flag ==1)
		{

			RFSendData(USART1_RX_BUF,25 );
			LED0 = !LED0;//�仯LED�ƣ�ָʾ����
			TIM3_send_flag = 0;
		}
		
	}	 
}
 
