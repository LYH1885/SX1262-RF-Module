#include "sys.h"
#include "usart.h"	
#include "sx1262.h"
#include "string.h"
#include "led.h"
#include <stdarg.h>

u8  USART1_TX_BUF[USART1_MAX_TX_LEN];    //���ͻ���,���USART1_MAX_TX_LEN�ֽ�
u8  USART1_RX_BUF[USART1_MAX_RX_LEN];    //���ջ���,���USART1_MAX_RX_LEN�ֽ�
volatile u16 USART1_RX_STA=0;            //bit15:������ɱ�־   bit14~0:���յ�����Ч�ֽ���Ŀ    

int RF_sent_flag = 0;
int UART1_DMA_flag = 0;

//����ṹ����ҪΪ�˿���ͬ���ڼ���Ҫ�޸���Щ����
USART1_InitTypeDef  USART1_InitStructure  =
{
	//�жϷ������ǵ��޸�
	USART1,
	RCC_APB2Periph_USART1,
	RCC_APB2Periph_GPIOA,
	GPIOA,
	GPIO_Pin_9,
	GPIO_Pin_10,
	USART1_IRQn,
#if USART1_DMA==1
	0,//��������ж�Ҫ���ڵ��÷��ͺ������ж����ȼ�(��ֵС��)����ֹ�޷��жϷ��ͺ��������ɣ���ӳ�ʱ����
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
	//��̬����	
	0,
	0,
#endif
};

void USART1_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if USART1_DMA==1
	while(USART1_InitStructure.Tx_flag == 1);//��ֹDMA���������ã���Ϸ���
#endif		 
	RCC_APB2PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);             //��ʼ����ʱ��
	if(USART1_InitStructure.USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) //�õ�ַ��ֹ��ֲ�滻������
		RCC_APB2PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);         //ʹ�ܴ���1ʱ��
	else
		RCC_APB1PeriphClockCmd(USART1_InitStructure.USARTx_RCC, ENABLE);         //ʹ�ܴ���2,3ʱ��
	
	RCC_APB2PeriphClockCmd(USART1_InitStructure.GPIO_RCC_APBx, ENABLE);	 
	//TX   
	GPIO_InitStructure.GPIO_Pin   = USART1_InitStructure.GPIO_TX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;	     //�����������
	GPIO_Init(USART1_InitStructure.GPIO_TX_RX_Pin_TypeDef, &GPIO_InitStructure);

	//RX
	GPIO_InitStructure.GPIO_Pin  = USART1_InitStructure.GPIO_RX_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(USART1_InitStructure.GPIO_TX_RX_Pin_TypeDef, &GPIO_InitStructure);
 
	//Usart NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate            = bound;                         //���ڲ�����
	USART_InitStructure.USART_WordLength          = USART_WordLength_9b;           //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits            = USART_StopBits_2;              //һ��ֹͣλ
	USART_InitStructure.USART_Parity              =  USART_Parity_Even;               //żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //�շ�ģʽ

	USART_Init(USART1_InitStructure.USARTx, &USART_InitStructure);                 //��ʼ������
#if USART1_DMA==0
	USART_ITConfig(USART1_InitStructure.USARTx, USART_IT_RXNE, ENABLE);            //�������ڽ����ж�
#endif
	USART_ITConfig(USART1_InitStructure.USARTx, USART_IT_IDLE, ENABLE);            //�������ڿ���IDEL�ж�
	USART_Cmd(USART1_InitStructure.USARTx, ENABLE);                                //ʹ�ܴ��� 
#if USART1_DMA==1
	DMA_USART1_Init();                                                             //���� DMA ����
	USART_DMACmd(USART1_InitStructure.USARTx, USART_DMAReq_Tx, ENABLE);            //��������DMA����
	USART_DMACmd(USART1_InitStructure.USARTx, USART_DMAReq_Rx, ENABLE);            //��������DMA����
#endif
	//USART1_printf("USART1 OK...\r\n");
}
 
#if USART1_DMA==1
DMA_InitTypeDef DMA_USART1_InitStructure;//Ϊ���л��������޸ĵ�ַ�������ȫ�ֵ�
void DMA_USART1_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	
	if((u32)USART1_InitStructure.USART_TX_DMA  >   (u32)DMA2_BASE) //DMA2
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA2 , ENABLE );
	else                                                           //DMA1
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 , ENABLE );
		
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USART_TX_DMA_IRQn;      // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_InitStructure.USART_TX_DMA_IRQn_NVIC; // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#if USART1_DMA_RX_TC==1
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_InitStructure.USART_RX_DMA1_Channelx_IRQn;      // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_InitStructure.USART_RX_DMA1_Channelx_IRQn_NVIC; // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
#endif		
	/*--- DMA_UART_Tx_DMA_Channel DMA Config ---*/ 
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);                                        //��DMAͨ��
    DMA_DeInit(USART1_InitStructure.USART_TX_DMA);                                              //�ָ�ȱʡֵ
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1_InitStructure.USARTx->DR));//ע������ط�ȡ2�ε�ַ  ���ô��ڷ������ݼĴ���
    DMA_USART1_InitStructure.DMA_MemoryBaseAddr     = (u32)USART1_TX_BUF;                       //���÷��ͻ������׵�ַ
    DMA_USART1_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;                    //��������λĿ�꣬�ڴ滺���� -> ����Ĵ���
    DMA_USART1_InitStructure.DMA_BufferSize         = USART1_MAX_TX_LEN;                        //��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
    DMA_USART1_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;                //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART1_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                     //�ڴ滺������ַ���ӵ���
    DMA_USART1_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;              //�������ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;                  //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_Mode               = DMA_Mode_Normal;                          //���δ���ģʽ����ѭ��
    DMA_USART1_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;                    //���ȼ�����
    DMA_USART1_InitStructure.DMA_M2M                = DMA_M2M_Disable;                          //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USART1_InitStructure.USART_TX_DMA, &DMA_USART1_InitStructure);                     //д������
    DMA_ClearFlag(USART1_InitStructure.USART_TX_FLAG_GL);                                       //���DMA���б�־    
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);                                        //�ر�DMA
    DMA_ITConfig(USART1_InitStructure.USART_TX_DMA, DMA_IT_TC, ENABLE);                         //��������DMAͨ���ж�
   
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);                                        //��DMAͨ��
    DMA_DeInit(USART1_InitStructure.USART_RX_DMA);                                              //�ָ�ȱʡֵ
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1_InitStructure.USARTx->DR));//ע������ط�ȡ��ַ���ô��ڽ������ݼĴ���
 
	DMA_USART1_InitStructure.DMA_MemoryBaseAddr     = (u32)USART1_RX_BUF;                       //���ý��ջ������׵�ַ
   
    DMA_USART1_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                                   //DMA_DIR_PeripheralSRC:�������  DMA_DIR_PeripheralDST:���ڴ��
    DMA_USART1_InitStructure.DMA_BufferSize         = USART1_MAX_RX_LEN;                        //��Ҫ�����ܽ��յ����ֽ���
    DMA_USART1_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;                //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART1_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                     //�ڴ滺������ַ���ӵ���
    DMA_USART1_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;              //�������ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;                  //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_Mode               = DMA_Mode_Normal;                          //���δ���ģʽ����ѭ��
    DMA_USART1_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;                    //���ȼ�����
    DMA_USART1_InitStructure.DMA_M2M                = DMA_M2M_Disable;                          //�Ƿ����ڴ浽�ڴ洫��(�رձ�ʾֻ���¼������Ŵ���һ�����ݣ�������ʾһֱ����)DMA_M2M_Disable:�Ǵ洢�����洢��ģʽ(�ر��ڴ浽�ڴ�ģʽ)  DMA_M2M_Enable:�����洢�����洢��ģʽ(�����ڴ浽�ڴ�ģʽ)
    DMA_Init(USART1_InitStructure.USART_RX_DMA, &DMA_USART1_InitStructure);                     //д������
		
    DMA_ClearFlag(USART1_InitStructure.USART_RX_FLAG_GL);                                       //���DMA���б�־
    DMA_Cmd(USART1_InitStructure.USART_RX_DMA, ENABLE);                                         //��������DMAͨ�����ȴ���������
#if USART1_DMA_RX_TC==1 
	DMA_ITConfig(USART1_InitStructure.USART_RX_DMA1_Channelx, DMA_IT_TC, ENABLE);               //��������DMAͨ���ж�
#endif
}
 
//DMA ����Ӧ��Դ��
void DMA_USART1_Tx_Data(u16 size)
{
	USART1_InitStructure.Tx_flag = 1;
	USART1_InitStructure.USART_TX_DMA->CNDTR =size;                    //����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(USART1_InitStructure.USART_TX_DMA, ENABLE);                //��ʼDMA����
}

void TX_DMAx_Channelx_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_InitStructure.USART_TX_FLAG_TC))
    {
		DMA_ClearFlag(USART1_InitStructure.USART_TX_FLAG_GL);          //�����־
		DMA_Cmd(USART1_InitStructure.USART_TX_DMA, DISABLE);           //�ر�DMAͨ��
		USART1_InitStructure.Tx_flag = 0;                              //�������
    }
}

void RX_DMAx_Channelx_IRQHandler(void)//DMA ���գ�����USART1_MAX_RX_LEN������
{
    if(DMA_GetITStatus(USART1_InitStructure.USART_RX_FLAG_TC))
    {
		DMA_ClearFlag(USART1_InitStructure.USART_RX_FLAG_GL);          //�����־
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);           //�ر�DMA ����ֹ����
    }
}
#endif
 
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1_InitStructure.USARTx , USART_IT_IDLE) != RESET)  //�����ж�
    {


			USART_ReceiveData(USART1_InitStructure.USARTx);                          //��������жϱ�־
#if USART1_DMA==1
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, DISABLE);                     //�ر�DMA ����ֹ����  		
		//ע�⣬����жϷ�������֡�����ʺܿ죬MCU����������˴ν��յ������ݣ��ж��ַ������ݵĻ������ﲻ�ܿ������������ݻᱻ���ǡ���2�ַ�ʽ�����
		//1. �����¿�������DMAͨ��֮ǰ����DMA_Rx_Buf��������������ݸ��Ƶ�����һ�������У�Ȼ���ٿ���DMA��Ȼ�����ϴ����Ƴ��������ݡ�
		//2. ����˫���壬��DMA_Uart_DMA_Rx_Data�����У���������DMA_MemoryBaseAddr �Ļ�������ַ����ô�´ν��յ������ݾͻᱣ�浽�µĻ������У������ڱ����ǡ�

		//���¿�ʼ��һ�ν���ǰ�Ȱ���һ�ν��ܵ����ݳ��ȣ������ݶ�����

		USART1_RX_STA = USART1_MAX_RX_LEN - DMA_GetCurrDataCounter(USART1_InitStructure.USART_RX_DMA); //��ý��յ����ֽ���
		DMA_USART1_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_RX_BUF;
		DMA_Init(USART1_InitStructure.USART_RX_DMA, &DMA_USART1_InitStructure);
		USART1_InitStructure.USART_RX_DMA->CNDTR = USART1_MAX_RX_LEN;            // ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
		DMA_Cmd(USART1_InitStructure.USART_RX_DMA, ENABLE);                      // DMA �������ȴ����ݡ�
#endif		
		//USART1_RX_BUF[USART1_RX_STA&0X7FFF]='\0';		                         //���\0,��ֹ�ַ���������������\0һֱ������
		USART1_RX_STA|=0x8000;	                                                //��ǽ��������
        //��Ӵ�����,�����������ѯ����
			//LED0 = !LED0;
			//RF_sent_flag = 1;
			//RFSendData(USART1_RX_BUF,25 );
		}
    
	if(USART_GetFlagStatus(USART1_InitStructure.USARTx,USART_FLAG_ORE) == SET) // ��� ORE ��־,��ֹ�������ж����������ڽ����ж�ǰ��
	{
		USART_ClearFlag(USART1_InitStructure.USARTx,USART_FLAG_ORE);
		USART_ReceiveData(USART1_InitStructure.USARTx);
	}
#if USART1_DMA==0
	if(USART_GetITStatus(USART1_InitStructure.USARTx, USART_IT_RXNE) != RESET)   
	{
		u8 res = USART_ReceiveData(USART1_InitStructure.USARTx);	          //��ȡ���յ�������	
		if((USART1_RX_STA&0X7FFF)<USART1_MAX_RX_LEN-1)                        //�������鳤�ȵ�����,�����жϺ�������ǰ�������ݻ��������
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

//����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���
void Usart1_Send(u8 *buf,u8 len)
{
	u8 t;
  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART1,buf[t]);
	}	 
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
}



 