#include "delay.h"
#include "stm32f10x_gpio.h"
#include  "stm32f10x_tim.h"
#include "sx1262.h"
#include "led.h"
FlagType  Sx1262_Flag;
//30Hz中断时间
void timerx_init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure; 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=3333;		 								/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);																		/* 开启时钟 */
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)  //30Hz，33ms一个中断
{
	if( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
		Sx1262_Flag.rf_timeout++;
		if(Sx1262_Flag.rf_timeout == 120) //4秒钟超时
		{
			Sx1262_Flag.rf_timeout=0;
			Sx1262_Flag.rf_reach_timeout = 1;
		}
		Sx1262_Flag.busy_timeout_cnt++;
		if(Sx1262_Flag.busy_timeout_cnt == 120) //4秒钟超时
		{
			Sx1262_Flag.busy_timeout_cnt=0;
			Sx1262_Flag.busy_timeout = 1;
		}
	}		 	
}

void Sx1262_SPI_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(SX1262_SPI_NSS_PORT_RCC, ENABLE );
  RCC_APB2PeriphClockCmd(SX1262_SPI_SCK_PORT_RCC, ENABLE );
  RCC_APB2PeriphClockCmd(SX1262_SPI_MOSI_PORT_RCC, ENABLE );
	RCC_APB2PeriphClockCmd(SX1262_SPI_MISO_PORT_RCC, ENABLE );
	RCC_APB2PeriphClockCmd(SX1262_RF_NRST_PORT_RCC, ENABLE );
	RCC_APB2PeriphClockCmd(SX1262_RF_BUSY_PORT_RCC, ENABLE );
	RCC_APB2PeriphClockCmd(SX1262_RF_IRQ_PORT_RCC, ENABLE );

  GPIO_InitStructure.GPIO_Pin = SX1262_SPI_NSS_PIN;// SPI_NSS
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SX1262_SPI_NSS_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SX1262_SPI_SCK_PIN;//SPI_SCK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SX1262_SPI_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SX1262_SPI_MOSI_PIN;//SPI_MOSI
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SX1262_SPI_MOSI_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = SX1262_SPI_MISO_PIN;//SPI_MISO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SX1262_SPI_MISO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SX1262_RF_NRST_PIN;//RF_NRESET
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SX1262_RF_NRST_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SX1262_RF_BUSY_PIN;//RF BUSY
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SX1262_RF_BUSY_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SX1262_RF_IRQ_PIN;//RF IRQ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
  GPIO_Init(SX1262_RF_BUSY_PORT, &GPIO_InitStructure);
}

uint8_t spi_rw(uint8_t data) 
{
	uint8_t i,rdata=0x0;
	for (i = 0; i < 8; i++)		
	{				
		if (data & 0x80)
			SPI_MOSI(HIGH);
		else
			SPI_MOSI(LOW);
		data <<= 1;
		rdata <<= 1;
		SPI_SCK(HIGH);
		if (SPI_MISO())
			rdata |= 0x01;		
		SPI_SCK(LOW);
	}	
	SPI_MOSI(HIGH);
	return (rdata);	
}

/***************sx1262*****************/
void check_busy(void)
{
	Sx1262_Flag.busy_timeout_cnt = 0;								
	Sx1262_Flag.busy_timeout = 0;			
	while(RF_BUSY())
	{
		if(Sx1262_Flag.busy_timeout)			
		{
			SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
			reset_sx1262();		//reset RF
			sx1262_Config();
			Rx_Init();
			break;		
		}
	}
}

void reset_sx1262(void)
{
	RF_NRST(LOW);
	delay_ms(2);		//more thena 100us, delay 10ms
	RF_NRST(HIGH);
	delay_ms(20);		//delay 10ms
} 

void SetSleep(void)
{
	uint8_t Opcode,sleepConfig;
	
	check_busy();
	Opcode = SET_SLEEP;	//0x84
	sleepConfig = 0x04;	//bit2: 1:warm start; bit0:0: RTC timeout disable
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(sleepConfig);
	SPI_NSS(HIGH);
}

/*********************************************************************
函数名：SetStandby(uint8_t StdbyConfig)
功能：用于将设备设置为处于中等消耗级别的配置模式
参数：0:STDBY_RC; 1:STDBY_XOSC
*********************************************************************/

void SetStandby(uint8_t StdbyConfig)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = SET_STANDBY;	//0x80	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(StdbyConfig);
	SPI_NSS(HIGH);
}

/*********************************************************************
函数名：SetTx(uint32_t timeout)
功能：将设备设置为发送模式
参数：
*********************************************************************/
void SetTx(uint32_t timeout)
{
	uint8_t Opcode;
	uint8_t time_out[3];
	
	check_busy();
	Opcode = SET_TX;	//0x83
	time_out[0] = (timeout>>16)&0xFF;//MSB
	time_out[1] = (timeout>>8)&0xFF;
	time_out[2] = timeout&0xFF;//LSB
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(time_out[0]);
	spi_rw(time_out[1]);
	spi_rw(time_out[2]);
	SPI_NSS(HIGH);
}

/*********************************************************************
函数名：SetRx(uint32_t timeout)
功能：将设备设置为接收模式
参数：
*********************************************************************/
void SetRx(uint32_t timeout)
{
	uint8_t Opcode;
	uint8_t time_out[3];
	
	check_busy();
	
	Opcode = SET_RX;	//0x82
	time_out[0] = (timeout>>16)&0xFF;//MSB
	time_out[1] = (timeout>>8)&0xFF;
	time_out[2] = timeout&0xFF;//LSB
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(time_out[0]);
	spi_rw(time_out[1]);
	spi_rw(time_out[2]);
	SPI_NSS(HIGH);
}

/*********************************************************************
函数名：SetPacketType(uint8_t PacketType)
功能：在LoRa或FSK模式下设置SX1261无线电
参数：0:GFSK; 1:LORA
*********************************************************************/

void SetPacketType(uint8_t PacketType)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = SET_PACKET_TYPE;	//0x8A	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(PacketType);
	SPI_NSS(HIGH);
}
/*********************************************************************
函数名：GetPacketType(void)
功能：返回无线电的当前操作包类型
参数：
*********************************************************************/
uint8_t GetPacketType(void)
{
	uint8_t Opcode;
	uint8_t Status;
	uint8_t packetType;
	
	check_busy();
	Opcode = 0x11;	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	Status = spi_rw(0xFF);
	packetType = spi_rw(0xFF);
	SPI_NSS(HIGH);
	
	return packetType;
}

/*********************************************************************
函数名：SetRfFrequency(uint32_t RfFreq)
功能：设置RF频率模式的频率
参数：RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32
*********************************************************************/


void SetRfFrequency(uint32_t RfFreq)
{
	uint8_t Opcode;
	uint8_t Rf_Freq[4];
	
	check_busy();
	
	Opcode = SET_RF_FREQUENCY;	//0x86
	
	Rf_Freq[0] = (RfFreq>>24)&0xFF;//MSB
	Rf_Freq[1] = (RfFreq>>16)&0xFF;
	Rf_Freq[2] = (RfFreq>>8)&0xFF;
	Rf_Freq[3] = RfFreq&0xFF;//LSB
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(Rf_Freq[0]);
	spi_rw(Rf_Freq[1]);
	spi_rw(Rf_Freq[2]);
	spi_rw(Rf_Freq[3]);
	SPI_NSS(HIGH);
}
/*********************************************************************
函数名：SetPaConfig
功能：用于区分SX1261和SX1262的命令。 使用此命令时，用户选择设备要使用的功率放大器及其配置
参数：
*********************************************************************/

void SetPaConfig(void)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = 0x95;	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(0x04);	//paDutyCycle 
	spi_rw(0x07);	//hpMax:0x00~0x07; 7:22dbm
	spi_rw(0x00);	//deviceSel: 0: SX1262; 1: SX1261
	spi_rw(0x01);
	SPI_NSS(HIGH);
}

/*********************************************************************
函数名：SetRegulatorMode
功能：指定将DC-DC还是LDO用于功率调节
参数：
*********************************************************************/
void SetRegulatorMode(void)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = 0x96;	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(0x01);//regModeParam
	SPI_NSS(HIGH);
}

/*
@para 
power:
-17(0xEF) to +14(0x0E) dBm by step of 1 dB if low power PA is selected
-9(0xF7) to +22(0x16) dBm by step of 1 dB if high power PA is selected

RampTime:
-------------------------------------
RampTime 	  | Value | RampTime(祍)
-------------------------------------
SET_RAMP_10U    0x00    10
SET_RAMP_20U    0x01    20
SET_RAMP_40U 	0x02	40
SET_RAMP_80U 	0x03	80
SET_RAMP_200U 	0x04	200
SET_RAMP_800U 	0x05	800
SET_RAMP_1700U 	0x06	1700
SET_RAMP_3400U 	0x07	3400
*/

#define PACK_MAX_LEN 32

/*********************************************************************
函数名：SetTxParams
功能：设置TX输出功率，并使用参数RampTime设置TX斜坡时间
参数：
*********************************************************************/
void SetTxParams(uint8_t power,uint8_t RampTime)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = SET_TX_PARAMS;	//0x8E	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(power);
	spi_rw(RampTime);
	SPI_NSS(HIGH);
}


void SetBufferBaseAddress(uint8_t TX_base_addr,uint8_t RX_base_addr)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = SET_BUF_BASE_ADDR;	//0x8F	
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(TX_base_addr);
	spi_rw(RX_base_addr);
	SPI_NSS(HIGH);
}

void WriteRegister(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t Opcode;
	uint8_t addr_l,addr_h;
	uint8_t i;
	
	if(length<1)			
		return;
	
	check_busy();
	addr_l = address&0xff;
	addr_h = address>>8;
	Opcode = 0x0D;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(addr_h);//MSB
	spi_rw(addr_l);//LSB
	for(i=0;i<length;i++)
	{
		spi_rw(data[i]);
	}
	SPI_NSS(HIGH);
}

void ReadRegister(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t Opcode;
	uint8_t addr_l,addr_h;
	uint8_t i;
	
	if(length<1)			
		return;
	check_busy();
	
	addr_l = address&0xff;
	addr_h = address>>8;
	
	Opcode = 0x1D;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(addr_h);//MSB
	spi_rw(addr_l);//LSB
	for(i=0;i<length;i++)
	{
		data[i]=spi_rw(0xFF);
	}
	SPI_NSS(HIGH);
}

void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
	uint8_t Opcode;
	uint8_t i;
	
	if(length<1)			
		return;
	
	check_busy();
	Opcode = 0x0E;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(offset);
	for(i=0;i<length;i++)
	{
		spi_rw(data[i]);
	}
	SPI_NSS(HIGH);
}

void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
	uint8_t Opcode;
	uint8_t i;
	
	if(length<1)			
		return;
	check_busy();
	
	Opcode = 0x1E;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(offset);
	spi_rw(0xFF);
	for(i=0;i<length;i++)
	{
		data[i]=spi_rw(0xFF);
	}
	SPI_NSS(HIGH);
}

void GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer)
{
	uint8_t Opcode;
	uint8_t Status;
	
	check_busy();
	
	Opcode = 0x13;
	SPI_NSS(LOW);
	spi_rw(Opcode);

	Status = spi_rw(0xFF);
	*payload_len = spi_rw(0xFF);
	*buf_pointer = spi_rw(0xFF);
	SPI_NSS(HIGH);
}

void SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = 0x8B;
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	
	spi_rw(sf);//SF=5~12
	spi_rw(bw);//BW
	spi_rw(cr);//CR
	spi_rw(ldro);//LDRO LowDataRateOptimize 0:OFF; 1:ON;
	
	spi_rw(0XFF);//
	spi_rw(0XFF);//
	spi_rw(0XFF);//
	spi_rw(0XFF);//
	
	SPI_NSS(HIGH);
}

void SetPacketParams(uint8_t payload_len)
{
	uint8_t Opcode;
	uint16_t prea_len;
	uint8_t prea_len_h,prea_len_l;
	
	check_busy();
	
	Opcode = 0x8C;
	
	prea_len = 8;
	prea_len_h = prea_len>>8;
	prea_len_l = prea_len&0xFF;
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	
	spi_rw(prea_len_h);//PreambleLength MSB
	spi_rw(prea_len_l);//PreambleLength LSB
	spi_rw(0x00);//HeaderType 0:Variable,explicit 1:Fixed,implicit
	//spi_rw(0x01);
	spi_rw(payload_len);//PayloadLength: 0x00 to 0xFF
	
	spi_rw(0X01);//CRCType 0:OFF 1:ON
	spi_rw(0X00);//InvertIQ 0:Standard 1:Inverted
	spi_rw(0XFF);//
	spi_rw(0XFF);//
	spi_rw(0XFF);//
	
	SPI_NSS(HIGH);
}

void SetDioIrqParams(uint16_t irq)
{
	uint8_t Opcode;
	uint16_t Irq_Mask;
	uint8_t Irq_Mask_h,Irq_Mask_l;
	uint16_t DIO1Mask;
	uint8_t DIO1Mask_h,DIO1Mask_l;
	uint16_t DIO2Mask;
	uint8_t DIO2Mask_h,DIO2Mask_l;
	uint16_t DIO3Mask;
	uint8_t DIO3Mask_h,DIO3Mask_l;
	
	Irq_Mask = irq;
	DIO1Mask = irq;
	DIO2Mask = 0;
	DIO3Mask = 0;
	
	Irq_Mask_h = Irq_Mask>>8;
	Irq_Mask_l = Irq_Mask&0xFF;
	DIO1Mask_h = DIO1Mask>>8;
	DIO1Mask_l = DIO1Mask&0xFF;
	DIO2Mask_h = DIO2Mask>>8;
	DIO2Mask_l = DIO2Mask&0xFF;
	DIO3Mask_h = DIO3Mask>>8;
	DIO3Mask_l = DIO3Mask&0xFF;
	Opcode = 0x08;
	
	check_busy();
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	
	spi_rw(Irq_Mask_h);//Irq_Mask MSB
	spi_rw(Irq_Mask_l);//Irq_Mask LSB
	spi_rw(DIO1Mask_h);//
	spi_rw(DIO1Mask_l);//
	
	spi_rw(DIO2Mask_h);//
	spi_rw(DIO2Mask_l);//
	spi_rw(DIO3Mask_h);//
	spi_rw(DIO3Mask_l);//
	
	SPI_NSS(HIGH);
}

uint16_t GetIrqStatus(void)
{
	uint8_t Opcode;
	uint8_t Status;
	uint16_t IrqStatus;
	uint8_t temp;
	
	check_busy();
	
	Opcode = 0x12;
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	Status = spi_rw(0xFF);
	temp = spi_rw(0xFF);
	IrqStatus = temp;
	IrqStatus = IrqStatus<<8;
	temp = spi_rw(0xFF);
	IrqStatus = IrqStatus|temp;
	SPI_NSS(HIGH);
	
	return IrqStatus;
}
void ClearIrqStatus(uint16_t irq)
{
	uint8_t Opcode;
	uint16_t irq_h,irq_l;
	
	check_busy();
	
	irq_h = irq>>8;
	irq_l = irq&0xFF;
	
	Opcode = 0x02;
	
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(irq_h);
	spi_rw(irq_l);
	SPI_NSS(HIGH);
}

void SetDIO2AsRfSwitchCtrl(void)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = 0x9D;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(0x01);   //DIO2 is selected to be used to control an RF switch; DIO2 = 1 in TX mode
	SPI_NSS(HIGH);
}

void SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage)
{
	uint8_t Opcode;
	
	check_busy();
	Opcode = 0x97;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	spi_rw(tcxoVoltage);   //
	spi_rw(0x00);		   //Timeout MSB ; Timeout duration = Timeout *15.625 祍
	spi_rw(0x00);
	spi_rw(0x64);          //Timeout LSB
	
	SPI_NSS(HIGH);
}

uint8_t buf[4]={0};
uint8_t GetRssiInst(void)
{
	uint8_t Opcode;
	uint8_t rssi;
	check_busy();
	Opcode = 0x14;
	SPI_NSS(LOW);
	spi_rw(Opcode);
	buf[0] = spi_rw(0xFF);
	buf[1] = spi_rw(0xFF);
	buf[2] = spi_rw(0xFF);
	buf[3] = spi_rw(0xFF);
	SPI_NSS(HIGH);
	if(Sx1262_Flag.RFCtrl.RFmode)
		rssi = buf[1];
	else
		rssi = buf[3];
	return rssi;
}


void Tx_Start(uint8_t *txbuf,uint8_t payload_length)
{
	SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
	SetBufferBaseAddress(0,0);//(TX_base_addr,RX_base_addr)
	WriteBuffer(0,txbuf,payload_length);//(offset,*data,length)
	SetPacketParams(payload_length);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
	
	SetDioIrqParams(TxDone_IRQ);//TxDone IRQ

	//Define Sync Word value
	SetTx(0);//timeout = 0
	
	Sx1262_Flag.is_tx = 1;						//发射标志和发射指示灯

	Sx1262_Flag.rf_timeout = 0;								
	Sx1262_Flag.rf_reach_timeout = 0;				//发送超时标志清零
	
	//Wait for the IRQ TxDone or Timeout
	while(!RF_IRQ())
	{
		if(Sx1262_Flag.rf_reach_timeout)			//发送超时则复位模块
		{
			ClearIrqStatus(TxDone_IRQ);//Clear the IRQ TxDone flag
			SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
			reset_sx1262();		//reset RF
			sx1262_Config();
			break;		
		}
		Sx1262_Flag.Irq_Status = GetIrqStatus();
		if(TxDone_IRQ&Sx1262_Flag.Irq_Status)break;
	}
	Sx1262_Flag.Irq_Status = GetIrqStatus();
	ClearIrqStatus(TxDone_IRQ);//Clear the IRQ TxDone flag
}

void Rx_Init(void)
{
	Sx1262_Flag.is_tx = 0;
	SetBufferBaseAddress(0,0);//(TX_base_addr,RX_base_addr)
	//SetPacketParams(PACK_MAX_LEN);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
	SetDioIrqParams(RxDone_IRQ);//RxDone IRQ
	
	//Define Sync Word value
	SetRx(0);//timeout = 0
}

void sx1262_Config(void)
{
	SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
	SetRegulatorMode();
	SetPaConfig();
	SetDIO3AsTCXOCtrl(DIO3_3_3V);
	SetDIO2AsRfSwitchCtrl();
	
	SetPacketType(Sx1262_Flag.RFCtrl.RFmode);	//0:GFSK; 1:LORA
	SetRfFrequency(872611999);		//434M ; RF_Freq = freq_reg*32M/(2^25)
	SetTxParams(22,SET_RAMP_10U);	//set power and ramp_time
	
	SetModulationParams(SF5, LORA_BW_250, LORA_CR_4_5, Sx1262_Flag.RFCtrl.RFmode);//最大通信速率，与1276demo一样
	//SetModulationParams(SF5, Sx1262_Flag.RFCtrl.Bandwidth, Sx1262_Flag.RFCtrl.CodeRate, Sx1262_Flag.RFCtrl.RFmode);

	SetPacketParams(Sx1262_Flag.payload_length);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
}



const char *rfName = "SX1262";
 


u16	iSend, iRev;	//无线发送 接收计数
u8	sendBuf[25];	//发送缓冲区
u8	revBuf[25];	//接收缓冲区



//射频模块初始化
void RFInit( void )
{
	Sx1262_Flag.RFCtrl.RFmode = LDRO_ON;        //LoRa 模式开关
	Sx1262_Flag.RFCtrl.Bandwidth = LORA_BW_41;  //LoRa调制带宽
	Sx1262_Flag.RFCtrl.CodeRate = LORA_CR_4_5;  //传输码率
	Sx1262_Flag.RFCtrl.SpreadingFactor = SF5;   //扩频因子
	
	//timerx_init();        ///30Hz的中断频率
	Sx1262_SPI_Init();
	reset_sx1262();		//reset RF
	sx1262_Config();
	RFRxMode(); //初始化进入接收模式
}

//射频模块进入接收模式
void RFRxMode( void )
{
   Rx_Init();
}


//射频模块接收数据
u8 RFRevData(u8 *buf)
{
	u8 rbuf[64]={0},length,i;
  u8 Irq_Status;
	u8 packet_size,buf_offset;
  if(!Sx1262_Flag.is_tx)	
	{
		//Wait for the IRQ  RxDone or Timeout
		if(RF_IRQ())//等待接收中断
		{
			Irq_Status = GetIrqStatus();//读取中断状态
			if((Irq_Status&0x02) == RxDone_IRQ)
			{
				//LedGreen( On );
				GetRxBufferStatus(&packet_size, &buf_offset);
				ReadBuffer(buf_offset, rbuf, packet_size);
				Sx1262_Flag.RxRssi = 0-(GetRssiInst())/2;
				ClearIrqStatus(RxDone_IRQ);//Clear the IRQ RxDone flag
				Irq_Status = GetIrqStatus();// 
				RFRxMode();
				length = rbuf[0];
				for(i=0;i<length;i++)
				 *(buf+i) = rbuf[i+1];
				if(length)
			    iRev++;
				//LedGreen( Off );
				return length;
			} 
		}
	}
	return (0x0);
}

//通过无线模块发送大小为size的数据
u8 RFSendData( u8 *buf, u8 size )
{
	u8 sendbuf[25];
	u8 ret = 1,i=0;
	
	sendbuf[0] = size;
	for(i=0;i<size;i++)
	  sendbuf[i+1] = *(buf+i);
	Tx_Start(sendbuf,size+1);
	iSend++;
	RFRxMode();
 // LedRed( Off );
	//LED0 = !LED0;//变化LED灯，指示发送
	return ret;
}






