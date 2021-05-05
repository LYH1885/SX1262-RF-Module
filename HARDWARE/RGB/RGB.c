#include "config.h"
#include "target.h"
/*
#define RGBLedNum 1
#define RGBLedDMA
#define RGB_PIN                 GPIO_Pin_10
#define RGB_PORT                GPIOB
#define RGBLedTIMx TIM2
#define RGBLedTimxCh            3
#define RGBLedTimxChRemap       GPIO_FullRemap_TIM2
#define RGBLedTimxRCC           RCC_APB1Periph_TIM2
#define RGBLedDmaCh             DMA1_Channel1
#define RGBLedDmaTcFlag         DMA1_FLAG_TC1
#define RGBLedDmaBaseAddr       0x4000003C 
#define RGBLedDMASource         TIM_DMA_CC3
*/
u32 RgbLedState = GreenRGB;
#if ( RGBLedNum >0 )
#ifdef RGBLedDMA
u8 DataRGB[64];
#define Data0Time 18	// Period 1.25us,set 18:0.25us
#define Data1Time 60	// Period 1.25us,set 60:1.00us
#define RESTime    0
void RGBInit(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RGBLedTimxRCC, ENABLE);		//TIMx Clock init
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//DMA1 Clock init
    
#ifdef RGBLedTimxChRemap	
    GPIO_PinRemapConfig(RGBLedTimxChRemap, ENABLE);		// IO Remap
#endif
    
    GPIO_InitStructure.GPIO_Pin = RGB_PIN;				// IO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RGB_PORT, &GPIO_InitStructure);			// Init GPIO

	TIM_TimeBaseStructure.TIM_Period = 89;  			//  Period 90
	TIM_TimeBaseStructure.TIM_Prescaler = 0;			//  f=72Mhz/((89+1)*1)=800KHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// TIM Clock -> 72MHz(HCLK Clock)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(RGBLedTIMx, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		// Mode -> PWM1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//Out State signal
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	// Out State signal -> High
    if(RGBLedTimxCh == 4)				// Init TIM_CHx ,according to RGBLedTimxCh
    {
        TIM_OC4Init(RGBLedTIMx, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(RGBLedTIMx,TIM_OCPreload_Enable);
    }
    else if(RGBLedTimxCh == 3) 
    {
        TIM_OC3Init(RGBLedTIMx, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(RGBLedTIMx,TIM_OCPreload_Enable);
    }
    else if(RGBLedTimxCh == 2) 
    {
        TIM_OC2Init(RGBLedTIMx, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(RGBLedTIMx,TIM_OCPreload_Enable);
    }
    else if(RGBLedTimxCh == 1) 
    {
        TIM_OC1Init(RGBLedTIMx, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(RGBLedTIMx,TIM_OCPreload_Enable);
    }
    
    TIM_ARRPreloadConfig(RGBLedTIMx,ENABLE);	// ENABLE auto Preload TIM_Period
	
	DMA_DeInit(RGBLedDmaCh);					// RGBLedDmaCh Enabled default default reset values.
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)RGBLedDmaBaseAddr; //TIMx_CCRx Addr + TIMx Addr,
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DataRGB;				// data Array
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMA Transmission direction
	DMA_InitStructure.DMA_BufferSize = 64;									//Transmission byte length
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//Peripheral added, Disable
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// memory to added, Enable
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// Peripherals byte length
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			// memory byte length,The same type as the Data Array
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//normal mode
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(RGBLedDmaCh, &DMA_InitStructure);		//Init DMA
	TIM_DMACmd(RGBLedTIMx, RGBLedDMASource, ENABLE);//DMA Request sources,TIM_DMA_CCx,x:1 2 3 4
}
void SetRGBLed(int data)
{
    u8 i,RGB_G=0,RGB_R=0,RGB_B=0;
    // separation  RGB
    RGB_G = data>>16 & 0XFF;
    RGB_R = data>>8  & 0XFF;
    RGB_B = data     & 0XFF;
    // Calculate the detached RGB value and map it to the timer
    for(i=0;i<8;i++)		//Loop 8 times to get a 24bit RGB value for an LED,
    {
        (RGB_G & 0x80)? (DataRGB[i] = Data1Time):(DataRGB[i] = Data0Time);
        RGB_G =RGB_G<<1;
        (RGB_R & 0x80)? (DataRGB[i+8] = Data1Time):(DataRGB[i+8] = Data0Time);
        RGB_R =RGB_R<<1;
        (RGB_B & 0x80)? (DataRGB[i+16] = Data1Time):(DataRGB[i+16] = Data0Time);
        RGB_B =RGB_B<<1;
    }
    u8 RGBLedDmaLength = 24;	//An RGB LED with 24 bits
    DataRGB[RGBLedDmaLength] = 0;
    RGBLedDmaLength++;	
    while(RGBLedDmaLength < 64)//RES Time
    {
        DataRGB[RGBLedDmaLength] = 0;
        RGBLedDmaLength++;
    }
    DMA_SetCurrDataCounter(RGBLedDmaCh, 64);//set Transmission length
    switch(RGBLedTimxCh)					//prevent confusion
    {
        case 1:TIM_SetCompare1(RGBLedTIMx,0);break;
        case 2:TIM_SetCompare2(RGBLedTIMx,0);break;
        case 3:TIM_SetCompare3(RGBLedTIMx,0);break;
        case 4:TIM_SetCompare4(RGBLedTIMx,0);break;
    }
    DMA_Cmd(RGBLedDmaCh, ENABLE);	// ENABLE DMA
    TIM_Cmd(RGBLedTIMx, ENABLE);	// ENABLE TIMx
    while(!DMA_GetFlagStatus(RGBLedDmaTcFlag)) ;//waiting Transmission completed
    DMA_Cmd(RGBLedDmaCh, DISABLE);	//Off
    DMA_ClearFlag(RGBLedDmaTcFlag);
    TIM_Cmd(RGBLedTIMx, DISABLE);	//Off
}


#else	

#define		RGB_LED_HIGH	(GPIO_SetBits(RGB_PORT,RGB_PIN))
#define 	RGB_LED_LOW		(GPIO_ResetBits(RGB_PORT,RGB_PIN))

void RGBInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	GPIO_InitStructure.GPIO_Pin = RGB_PIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(RGB_PORT, &GPIO_InitStructure);					
	GPIO_SetBits(RGB_PORT,RGB_PIN);						 
}

void RGB_LED_Write0(void)
{
	RGB_LED_HIGH;
	__nop();__nop();__nop();__nop();__nop();__nop();
	RGB_LED_LOW;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
}

void RGB_LED_Write1(void)
{
	RGB_LED_HIGH;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
	RGB_LED_LOW;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
}
void SetRGBLed(int data)
{
	for ( int i =23 ; i >=0 ; i--)
	{
		if (  (data>>i)&1  ) 
		{
			RGB_LED_Write1();
		}
		else 
		{
			RGB_LED_Write0();
		}
	}
}
#endif

#endif



