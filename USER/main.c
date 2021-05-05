#include "delay.h"
#include "sys.h"
#include "led.h"
#include "usart.h"
#include "sx1262.h"
#include "timer.h"
//主程序

extern int RF_sent_flag;
extern int TIM3_send_flag;
extern u8  USART1_TX_BUF[USART1_MAX_TX_LEN];    //发送缓冲,最大USART1_MAX_TX_LEN字节
extern u8  USART1_RX_BUF[USART1_MAX_RX_LEN];    //接收缓冲,最大USART1_MAX_RX_LEN字节


 int main(void)
 {	
	u16 times=0;
	
	delay_init(); //延时函数初始化
	RFInit();	 //射频模块初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	LED_Init(); //LED端口初始化
	USART1_Init(100000); //初始化串口（DMA发送和空闲中断接收模式）
	TIM3_Int_Init(109,7199);//10Khz的计数频率，计数到5000为500ms

	while(1)
	{
		
		if(TIM3_send_flag ==1)
		{

			RFSendData(USART1_RX_BUF,25 );
			LED0 = !LED0;//变化LED灯，指示发送
			TIM3_send_flag = 0;
		}
		
	}	 
}
 
