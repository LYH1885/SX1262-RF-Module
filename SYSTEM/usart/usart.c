#include "sys.h"
#include "usart.h"	
#include "sx1262.h"
#include "string.h"
#include "led.h"
#include <stdarg.h>

u8  USART1_TX_BUF[USART1_MAX_TX_LEN];    //发送缓冲,最大USART1_MAX_TX_LEN字节
u8  USART1_RX_BUF[USART1_MAX_RX_LEN];    //接收缓冲,最大USART1_MAX_RX_LEN字节
volatile u16 USART1_RX_STA=0;            //bit15:接收完成标志   bit14~0:接收到的有效字节数目    

int RF_sent_flag = 0;
int UART1_DMA_flag = 0;

//打包结构体主要为了看不同串口间需要修改那些参数
USART1_InitTypeDef  USART1_InitStructure  =
{
	//中断服务函数记得修改
	USART1,
	RCC_APB2Periph_USART1,
	RCC_APB2Periph_GPIOA,
	GPIOA,
	GPIO_Pin_9,
	GPIO_Pin_10,
	USART1_IRQn,
#if USART1_DMA==1
	0,//发送完成中断要高于调用发送函数的中断优先级(数值小于)，防止无法中断发送函数标记完成，或加超时处理
	DMA1_Channel4,              
	DMA1_Channel4_IRQn,
	DMA1_FLAG_GL4,
	DMA1_FLAG_TC4,
	#define TX_DMAx_Channelx_IRQHandler DMA1_Channel4_IRQHandler
	
	0,
	DMA1_Channel5,              
	DMA1_Channel5_IRQn,
	DMA1_FLAG_GL5,
	DMA1_FLAG_TC5,
	#define RX_DMAx_Channelx_IRQHandler DMA1_Channel5_IRQHandler
	//动态变量	
	0,
	0,
#endif
};

void USART1_Init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if USART1_DMA==1
	while(USART1_InitStructure.Tx_flag == 1);//防止DMA发送中设置，打断发送
#endif		 
	RCC_APB2PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);             //开始串口时钟
	if(USART1_InitStructure.USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) //用地址防止移植替换出问题
		RCC_APB2PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);         //使能串口1时钟
	else
		RCC_APB1PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);         //使能串口2,3时钟
	
	RCC_APB2PeriphClockCmd(USART1_InitStructure.GPIO_RCC_APBx, ENABLE);	 
	//TX   
	GPIO_InitStructure.GPIO_Pin   = USART1_InitStructure.GPIO_TX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;	     //复用推挽输出
	GPIO_Init(USART1_InitStructure.GPIO_TX_RX_Pin_TypeDef, &GPIO_InitStructure);

	//RX
	GPIO_InitStructure.GPIO_Pin  = USART1_InitStructure.GPIO_RX_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(USART1_InitStructure.GPIO_TX_RX_Pin_TypeDef, &GPIO_InitStructure);
 
	//Usart NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate            = bound;                         //串口波特率
	USART_InitStructure.USART_WordLength          = USART_WordLength_9b;           //字长为8位数据格式
	USART_InitStructure.USART_StopBits            = USART_StopBits_2;              //一个停止位
	USART_InitStructure.USART_Parity              =  USART_Parity_Even;               //偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //收发模式

	USART_Init(USART1_InitStructure.USARTx, &USART_InitStructure);                 //初始化串口
#if USART1_DMA==0
	USART_ITConfig(USART1_InitStructure.USARTx, USART_IT_RXNE, ENABLE);            //开启串口接受中断
#endif
	USART_ITConfig(USART1_InitStructure.USARTx, USART_IT_IDLE, ENABLE);            //开启串口空闲IDEL中断
	USART_Cmd(USART1_InitStructure.USARTx, ENABLE);                                //使能串口 
#if USART1_DMA==1
	DMA_USART1_Init();                                                             //串口 DMA 配置
	USART_DMACmd(USART1_InitStructure.USARTx, USART_DMAReq_Tx, ENABLE);            //开启串口DMA发送
	USART_DMACmd(USART1_InitStructure.USARTx, USART_DMAReq_Rx, ENABLE);            //开启串口DMA接收
#endif
	//USART1_printf("USART1 OK...\r\n");
}
 
#if USART1_DMA==1
DMA_InitTypeDef DMA_USART1_InitStructure;//为了切换缓冲区修改地址，定义成全局的
void DMA_USART1_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	
	if((u32)USART1_InitStructure.USART_TX_DMA  >   (u32)DMA2_BASE) //DMA2
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA2 , ENABLE );
	else                                                           //DMA1
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 , ENABLE );
		
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USART_TX_DMA_IRQn;      // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_InitStructure.USART_TX_DMA_IRQn_NVIC; // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#if USART1_DMA_RX_TC==1
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USART_RX_DMA1_Channelx_IRQn;      // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_InitStructure.USART_RX_DMA1_Channelx_IRQn_NVIC; // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
#endif		
	/*--- DMA_UART_Tx_DMA_Channel DMA Config ---*/ 
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);                                        //关DMA通道
    DMA_DeInit(USART1_InitStructure.USART_TX_DMA);                                              //恢复缺省值
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1_InitStructure.USARTx->DR));//注意这个地方取2次地址  设置串口发送数据寄存器
    DMA_USART1_InitStructure.DMA_MemoryBaseAddr     = (u32)USART1_TX_BUF;                       //设置发送缓冲区首地址
    DMA_USART1_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;                    //设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_USART1_InitStructure.DMA_BufferSize         = USART1_MAX_TX_LEN;                        //需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_USART1_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;                //外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_USART1_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                     //内存缓冲区地址增加调整
    DMA_USART1_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;              //外设数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;                  //内存数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_Mode               = DMA_Mode_Normal;                          //单次传输模式，不循环
    DMA_USART1_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;                    //优先级设置
    DMA_USART1_InitStructure.DMA_M2M                = DMA_M2M_Disable;                          //关闭内存到内存的DMA模式
    DMA_Init(USART1_InitStructure.USART_TX_DMA, &DMA_USART1_InitStructure);                     //写入配置
    DMA_ClearFlag(USART1_InitStructure.USART_TX_FLAG_GL);                                       //清除DMA所有标志    
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);                                        //关闭DMA
    DMA_ITConfig(USART1_InitStructure.USART_TX_DMA, DMA_IT_TC, ENABLE);                         //开启发送DMA通道中断
   
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);                                        //关DMA通道
    DMA_DeInit(USART1_InitStructure.USART_RX_DMA);                                              //恢复缺省值
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1_InitStructure.USARTx->DR));//注意这个地方取地址设置串口接收数据寄存器
 
	DMA_USART1_InitStructure.DMA_MemoryBaseAddr     = (u32)USART1_RX_BUF;                       //设置接收缓冲区首地址
   
    DMA_USART1_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                                   //DMA_DIR_PeripheralSRC:从外设读  DMA_DIR_PeripheralDST:从内存读
    DMA_USART1_InitStructure.DMA_BufferSize         = USART1_MAX_RX_LEN;                        //需要最大可能接收到的字节数
    DMA_USART1_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;                //外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_USART1_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                     //内存缓冲区地址增加调整
    DMA_USART1_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;              //外设数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;                  //内存数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_Mode               = DMA_Mode_Normal;                          //单次传输模式，不循环
    DMA_USART1_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;                    //优先级设置
    DMA_USART1_InitStructure.DMA_M2M                = DMA_M2M_Disable;                          //是否开启内存到内存传输(关闭表示只有事件产生才传输一次数据，开启表示一直传输)DMA_M2M_Disable:非存储器到存储器模式(关闭内存到内存模式)  DMA_M2M_Enable:启动存储器到存储器模式(开启内存到内存模式)
    DMA_Init(USART1_InitStructure.USART_RX_DMA, &DMA_USART1_InitStructure);                     //写入配置
		
    DMA_ClearFlag(USART1_InitStructure.USART_RX_FLAG_GL);                                       //清除DMA所有标志
    DMA_Cmd(USART1_InitStructure.USART_RX_DMA, ENABLE);                                         //开启接收DMA通道，等待接收数据
#if USART1_DMA_RX_TC==1 
	DMA_ITConfig(USART1_InitStructure.USART_RX_DMA1_Channelx, DMA_IT_TC, ENABLE);               //开启接收DMA通道中断
#endif
}
 
//DMA 发送应用源码
void DMA_USART1_Tx_Data(u16 size)
{
	USART1_InitStructure.Tx_flag = 1;
	USART1_InitStructure.USART_TX_DMA->CNDTR =size;                    //设置要发送的字节数目
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, ENABLE);                //开始DMA发送
}

void TX_DMAx_Channelx_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_InitStructure.USART_TX_FLAG_TC))
    {
		DMA_ClearFlag(USART1_InitStructure.USART_TX_FLAG_GL);          //清除标志
		DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);           //关闭DMA通道
		USART1_InitStructure.Tx_flag = 0;                              //发送完成
    }
}

void RX_DMAx_Channelx_IRQHandler(void)//DMA 接收，超过USART1_MAX_RX_LEN的舍弃
{
    if(DMA_GetITStatus(USART1_InitStructure.USART_RX_FLAG_TC))
    {
		DMA_ClearFlag(USART1_InitStructure.USART_RX_FLAG_GL);          //清除标志
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);           //关闭DMA ，防止干扰
    }
}
#endif
 
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1_InitStructure.USARTx , USART_IT_IDLE) != RESET)  //空闲中断
    {


			USART_ReceiveData(USART1_InitStructure.USARTx);                          //清除空闲中断标志
#if USART1_DMA==1
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);                     //关闭DMA ，防止干扰  		
		//注意，如果中断发送数据帧的速率很快，MCU来不及处理此次接收到的数据，中断又发来数据的话，这里不能开启，否则数据会被覆盖。有2种方式解决。
		//1. 在重新开启接收DMA通道之前，将DMA_Rx_Buf缓冲区里面的数据复制到另外一个数组中，然后再开启DMA，然后马上处理复制出来的数据。
		//2. 建立双缓冲，在DMA_Uart_DMA_Rx_Data函数中，重新配置DMA_MemoryBaseAddr 的缓冲区地址，那么下次接收到的数据就会保存到新的缓冲区中，不至于被覆盖。

		//重新开始下一次接收前先把上一次接受的数据长度，和数据读出来

		USART1_RX_STA = USART1_MAX_RX_LEN - DMA_GetCurrDataCounter(USART1_InitStructure.USART_RX_DMA); //获得接收到的字节数
		DMA_USART1_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_RX_BUF;
		DMA_Init(USART1_InitStructure.USART_RX_DMA, &DMA_USART1_InitStructure);
		USART1_InitStructure.USART_RX_DMA->CNDTR = USART1_MAX_RX_LEN;            // 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, ENABLE);                      // DMA 开启，等待数据。
#endif		
		//USART1_RX_BUF[USART1_RX_STA&0X7FFF]='\0';		                         //添加\0,防止字符串处理函数遇不见\0一直不结束
		USART1_RX_STA|=0x8000;	                                                //标记接收完成了
        //添加处理函数,最好主函数查询处理
			//LED0 = !LED0;
			//RF_sent_flag = 1;
			//RFSendData(USART1_RX_BUF,25 );
		}
    
	if(USART_GetFlagStatus(USART1_InitStructure.USARTx,USART_FLAG_ORE) == SET) // 检查 ORE 标志,防止开关总中断死机，放在接收中断前面
	{
		USART_ClearFlag(USART1_InitStructure.USARTx,USART_FLAG_ORE);
		USART_ReceiveData(USART1_InitStructure.USARTx);
	}
#if USART1_DMA==0
	if(USART_GetITStatus(USART1_InitStructure.USARTx, USART_IT_RXNE) != RESET)   
	{
		u8 res = USART_ReceiveData(USART1_InitStructure.USARTx);	          //读取接收到的数据	
		if((USART1_RX_STA&0X7FFF)<USART1_MAX_RX_LEN-1)                        //超过数组长度的舍弃,空闲中断后处理数据前来的数据会继续接上
		{
			USART1_RX_BUF[USART1_RX_STA&0X7FFF]=res;
			USART1_RX_STA++;
		}
	}
#endif

}
 
void USART1_printf (char *fmt, ...)
{
#if USART1_DMA==1
	u16 i = 0;
	while(USART1_InitStructure.Tx_flag==1);
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART1_TX_BUF, USART1_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);

	DMA_USART1_Tx_Data(strlen((char *)USART1_TX_BUF));

#else
	u16 i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART1_TX_BUF, USART1_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);

	while ((i < USART1_MAX_TX_LEN) && USART1_TX_BUF[i])
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USART1, (u8) USART1_TX_BUF[i++]);			
	}
#endif
}
 
void USART1_Send_Array (u8 *array,u16 num)
{
#if USART1_DMA==1
	while(USART1_InitStructure.Tx_flag==1);
	memcpy(USART1_TX_BUF,array,num);
	DMA_USART1_Tx_Data(num);
#else
	u16 i=0;
	while (i < num)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
		USART_SendData(USART1, (u8) array[i++]);
	}
#endif
}

//发送len个字节.
//buf:发送区首地址
//len:发送的字节数
void Usart1_Send(u8 *buf,u8 len)
{
	u8 t;
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART1,buf[t]);
	}	 
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
}



 