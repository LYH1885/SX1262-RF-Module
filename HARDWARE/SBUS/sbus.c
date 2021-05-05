#include "sbus.h"
#include "stdio.h"

typedef struct SBUS
{
	uint8_t head;
	uint8_t ch[22];		//16个通道编码
	struct 
	{
		bool ch17 :1;				//通道17
		bool ch18 :1;				//通道18
		bool framlost :1;
		bool failsafe	:1;
		uint8_t :4;
	}flage;
	uint8_t endbyte;	//尾字节
}SBUS;

typedef SBUS CHIN;	
CHIN code  = {.head = 0XFF, .endbyte = 0X66};	//接受侦
SBUS ocode = {.head = 0X0F, .endbyte = 0X40};	//发送帧

void SBUS_init(void)//初始化串口 DMA 定时器
{
	DMA_InitTypeDef  DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	USART_InitStructure.USART_BaudRate = 100000;//100K
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为9位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure);
  USART_Cmd(USART6, ENABLE); 
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);	//开启总线空闲中断
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);
	
	//usart6_tx
	while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE){}//等待DMAs2可配置 
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = NULL;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream6, DISABLE);
	
	//usatr6_rx
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&code;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = 25;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//最高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream1, DISABLE);	

  USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	RCC_ClocksTypeDef RCCClockStruct;
	RCC_GetClocksFreq(&RCCClockStruct);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Prescaler = (RCCClockStruct.PCLK1_Frequency / 10000) - 1; //定时器分频10Khz
	TIM_TimeBaseInitStructure.TIM_Period = (int)(SENDINTERVAL * 10) - 1;	//7ms重载	TIM10 7m定时发送
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器更新中断

	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM7,ENABLE); //使能定时器
}
//

//解调
CHS inchs;	//code
//返回 false 接收失败 true 成功

char loscnt = 0;//接收错误计数
static uint64_t rtimeover = 0;//接收定时器发送中断超计数
bool decode(void)//解码,只能由串口空闲中断调用
{
	#ifdef ENDBYTEIDENT
	static uint8_t endbyt =  0X04;
	static bool firstz = true;	//表示接收到的是第一帧数据
	loscnt = (loscnt >=120) ? 120: (loscnt + 1);
	if(firstz == false)	//上一帧接收成功
	{
		if(endbyt == code.endbyte && code.head == 0X0F)	//尾字节正确，和头字节正确
			endbyt = (code.endbyte == 0X34) ? (0X04): (code.endbyte + 0X10);
		else
		{
			firstz = true;
			inchs.flage.framlost = true;//使能信号丢失标志
			return false;
		}
	}
	else		//上一帧不成功或者头帧数据
	{
		if(code.head == 0X0F)	//头字节正确
		{
			switch(code.endbyte)
			{
				case 0X04 :	{endbyt =  0X14; break;}
				case 0X14	:	{endbyt =  0X24; break;}
				case 0X24 :	{endbyt =  0X34; break;}
				case 0X34	:	{endbyt =  0X04; break;}
				default:	
					{
						inchs.flage.failsafe |= (loscnt) > 10;	//连续丢失次数大于5 则使能保护	
						inchs.flage.framlost = true;//使能信号丢失标志
						return false;										//尾字节错误
					}
			}
			firstz = false;
		}
		else
		{
			inchs.flage.failsafe |= (loscnt) > 10;	//连续丢失次数大于5 则使能保护	
			inchs.flage.framlost = true;//使能信号丢失标志
			return false;
		}
	}
	#else
	if(code.head != 0X0F || code.endbyte == 0X66)		//尾字节为66说明没有接收完成
	{
		inchs.flage.failsafe |= (loscnt) > 10;	//连续丢失次数大于5 则使能保护	
		inchs.flage.framlost = true;//使能信号丢失标志
		return false;
	}
	#endif
	loscnt = 0;
	rtimeover = 0;
	float* _channels = inchs.ch;
	uint16_t buffer[22];
	uint8_t* tmp  =code.ch;
	taskENTER_CRITICAL();
	for(int i = 0;i<22;i++)
		buffer[i] = tmp[i];
	*_channels++  = (((buffer[0]		|buffer[1]<<8)           			 & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[1]>>3 |buffer[2]<<5)                 & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[2]>>6 |buffer[3]<<2 |buffer[4]<<10)  & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[4]>>1 |buffer[5]<<7)                 & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[5]>>4 |buffer[6]<<4)                 & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[6]>>7 |buffer[7]<<1 |buffer[8]<<9)   & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[8]>>2 |buffer[9]<<6)                 & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[9]>>5 |buffer[10]<<3)                & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[11]   |buffer[12]<<8)                & 0x07FF)) / 511.5 - 1;
	*_channels++   = (((buffer[12]>>3|buffer[13]<<5)                & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[13]>>6|buffer[14]<<2|buffer[15]<<10) & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[15]>>1|buffer[16]<<7)                & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[16]>>4|buffer[17]<<4)                & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[17]>>7|buffer[18]<<1|buffer[19]<<9)  & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[19]>>2|buffer[20]<<6)                & 0x07FF)) / 511.5 - 1;
	*_channels++  = (((buffer[20]>>5|buffer[21]<<3)                & 0x07FF)) / 511.5 - 1;
	*(uint8_t*)&inchs.flage = *(uint8_t*)&code.flage;
	taskEXIT_CRITICAL();
	code.endbyte = 0X66;
	code.head = 0XFF;

	return true;
}
//

CHS ReadSBUS(void)	//获取控制信号
{
	taskENTER_CRITICAL();
	CHS tmp = inchs;
	taskEXIT_CRITICAL();
	return tmp;
}
//


void USART6_IRQHandler(void) //接收中断 
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)	//串口空闲中断
	{
		DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN;
		decode();
		DMA2_Stream1->NDTR = 25;
		DMA2_Stream1->M0AR = (u32)&code;
		DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;
		USART6->SR;
		USART6->DR;
	}
}
//

CHS Wchs;		//输出的通道信号
void aecode(void)//调制
{
	taskENTER_CRITICAL();
	uint16_t channels[16];
	for(int i = 0;i<16;i++)
		channels[i] = (Wchs.ch[i] <-1 ? (uint16_t)0 : (  Wchs.ch[i] > 1 ? (uint16_t)1023 : ((uint16_t)( (Wchs.ch[i] + 1) * 511.5f))));
	*(uint8_t*)&ocode.flage = *(uint8_t*)&Wchs.flage;
	ocode.endbyte = (ocode.endbyte == 0X34)? 0X04 : (ocode.endbyte + 0X10);
	taskEXIT_CRITICAL();
	
	uint8_t* packet = ocode.ch;
	*packet++  = (uint8_t) ((channels[0] & 0x07FF));
	*packet++  = (uint8_t) ((channels[0] & 0x07FF)>>8   | (channels[1] & 0x07FF)<<3);
	*packet++  = (uint8_t) ((channels[1] & 0x07FF)>>5   | (channels[2] & 0x07FF)<<6);
	*packet++  = (uint8_t) ((channels[2] & 0x07FF)>>2);
	*packet++  = (uint8_t) ((channels[2] & 0x07FF)>>10  | (channels[3] & 0x07FF)<<1);
	*packet++  = (uint8_t) ((channels[3] & 0x07FF)>>7   | (channels[4] & 0x07FF)<<4);
	*packet++  = (uint8_t) ((channels[4] & 0x07FF)>>4   | (channels[5] & 0x07FF)<<7);
	*packet++  = (uint8_t) ((channels[5] & 0x07FF)>>1);
	*packet++  = (uint8_t) ((channels[5] & 0x07FF)>>9   | (channels[6] & 0x07FF)<<2);
	*packet++  = (uint8_t) ((channels[6] & 0x07FF)>>6   | (channels[7] & 0x07FF)<<5);
	*packet++  = (uint8_t) ((channels[7] & 0x07FF)>>3);
	*packet++  = (uint8_t) ((channels[8] & 0x07FF));
	*packet++  = (uint8_t) ((channels[8] & 0x07FF)>>8   | (channels[9] & 0x07FF)<<3);
	*packet++  = (uint8_t) ((channels[9] & 0x07FF)>>5   | (channels[10] & 0x07FF)<<6);  
	*packet++  = (uint8_t) ((channels[10] & 0x07FF)>>2);
	*packet++  = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
	*packet++  = (uint8_t) ((channels[11] & 0x07FF)>>7  | (channels[12] & 0x07FF)<<4);
	*packet++  = (uint8_t) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
	*packet++  = (uint8_t) ((channels[13] & 0x07FF)>>1);
	*packet++  = (uint8_t) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
	*packet++  = (uint8_t) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
	*packet++  = (uint8_t) ((channels[15] & 0x07FF)>>3);
	
}
//

void TIM7_IRQHandler(void)		//定时发送和接收超时中断
{
	if((TIM7->SR & TIM_IT_Update) != RESET)	//tim7 更新中断
	{
		if((TIM7->DIER & TIM_IT_Update) != RESET)
		{
			DMA2_Stream6->CR &= ~(uint32_t)DMA_SxCR_EN;
			aecode();
			DMA2_Stream6->NDTR = 25;
			DMA2_Stream6->M0AR = (u32)&ocode;
			DMA2_Stream6->CR |= (uint32_t)DMA_SxCR_EN;
		}
		TIM7->SR = ~TIM_IT_Update;
	}
	
	if(RESIVETIMEOUT <= rtimeover++)	//接收超时
	{
		DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN;
		USART6->SR;
		USART6->DR;
		DMA2_Stream1->NDTR = 25;
		DMA2_Stream1->M0AR = (u32)&code;
		DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;
		inchs.flage.failsafe = true;					//保护使能
		inchs.flage.framlost = true;					//使能信号丢失标志
		rtimeover = RESIVETIMEOUT;
	}
}
//
