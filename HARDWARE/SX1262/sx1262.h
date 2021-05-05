#ifndef  __SX1262_H__
#define  __SX1262_H__
#include "stm32f10x.h"
//IO define
/*******************************************************
*****************SPI端口管脚定义**********************
********************************************************/
//SX1262_SPI_NSS;  //SPI片选  SPI_NSS------>PA4
#define SX1262_SPI_NSS_PIN   GPIO_Pin_4
#define SX1262_SPI_NSS_PORT  GPIOA
#define SX1262_SPI_NSS_PORT_RCC  RCC_APB2Periph_GPIOA

//SX1262_SPI_SCLK;  //SPI串行时钟 ,SPI_SCK------>PA5
#define SX1262_SPI_SCK_PIN   GPIO_Pin_5
#define SX1262_SPI_SCK_PORT  GPIOA
#define SX1262_SPI_SCK_PORT_RCC  RCC_APB2Periph_GPIOA

//SX1262_SPI_MOSI ;  //SPI串行数据  SPI_MOSI------>PA7
#define SX1262_SPI_MOSI_PIN   GPIO_Pin_7
#define SX1262_SPI_MOSI_PORT  GPIOA
#define SX1262_SPI_MOSI_PORT_RCC  RCC_APB2Periph_GPIOA

//SX1262_SPI_MISO ;  //SPI串行数据  SPI_MISO------>PA6
#define SX1262_SPI_MISO_PIN   GPIO_Pin_6
#define SX1262_SPI_MISO_PORT  GPIOA
#define SX1262_SPI_MISO_PORT_RCC  RCC_APB2Periph_GPIOA

//SX1262_RF_NRESET ;  //RF模块复位 RF_NRESET------>PB1
#define SX1262_RF_NRST_PIN   GPIO_Pin_1
#define SX1262_RF_NRST_PORT  GPIOB
#define SX1262_RF_NRST_PORT_RCC  RCC_APB2Periph_GPIOB

//SX1262_RF_BUSY ;  //RF模块忙   RF_BUSY------>PB5
#define SX1262_RF_BUSY_PIN   GPIO_Pin_5
#define SX1262_RF_BUSY_PORT  GPIOB
#define SX1262_RF_BUSY_PORT_RCC  RCC_APB2Periph_GPIOB

//SX1262_RF_DIO1 ;  //RF模块DIO1，中断管脚  RF_IRQ------>PB10
#define SX1262_RF_IRQ_PIN   GPIO_Pin_10
#define SX1262_RF_IRQ_PORT  GPIOB
#define SX1262_RF_IRQ_PORT_RCC  RCC_APB2Periph_GPIOB

#define  HIGH  Bit_SET
#define  LOW   Bit_RESET


#define  SPI_NSS(LEVEL)   GPIO_WriteBit(SX1262_SPI_NSS_PORT,SX1262_SPI_NSS_PIN,LEVEL)
#define  SPI_SCK(LEVEL)   GPIO_WriteBit(SX1262_SPI_SCK_PORT,SX1262_SPI_SCK_PIN,LEVEL)
#define  SPI_MOSI(LEVEL)  GPIO_WriteBit(SX1262_SPI_MOSI_PORT,SX1262_SPI_MOSI_PIN,LEVEL)
#define  SPI_MISO()       (SX1262_SPI_MISO_PORT->IDR & SX1262_SPI_MISO_PIN)    //读取MOSI的电平
#define  RF_NRST(LEVEL)   GPIO_WriteBit(SX1262_RF_NRST_PORT,SX1262_RF_NRST_PIN,LEVEL)
#define  RF_BUSY()        (SX1262_RF_BUSY_PORT->IDR & SX1262_RF_BUSY_PIN)    //读取RF BUSY管脚电平
#define  RF_IRQ()         (SX1262_RF_IRQ_PORT->IDR & SX1262_RF_IRQ_PIN)    //读取RF IRQ管脚电平

/**********************************************/
#define  SF5   0x05
#define  SF6   0x06 
#define  SF7   0x07
#define  SF8   0x08
#define  SF9   0x09
#define  SF10  0x0A 
#define  SF11  0x0B 
#define  SF12  0x0C 

#define  LORA_BW_7	  0x00
#define  LORA_BW_10	  0x08
#define  LORA_BW_15	  0x01 
#define  LORA_BW_20	  0x09 
#define  LORA_BW_31	  0x02
#define  LORA_BW_41	  0x0A 
#define  LORA_BW_62	  0x03 
#define  LORA_BW_125  0x04 
#define  LORA_BW_250  0x05 
#define  LORA_BW_500  0x06

#define  LORA_CR_4_5  0x01
#define  LORA_CR_4_6  0x02
#define  LORA_CR_4_7  0x03
#define  LORA_CR_4_8  0x04

#define  LDRO_ON	  0x01
#define  LDRO_OFF     0x00
/**********************************************/
#define SET_SLEEP 			   0x84
#define SET_STANDBY			   0x80
#define SET_TX				     0x83
#define SET_RX				     0x82
#define SET_PACKET_TYPE		 0x8A
#define SET_RF_FREQUENCY 	 0x86
#define SET_TX_PARAMS 		 0x8E
#define SET_BUF_BASE_ADDR  0x8F

#define SET_RAMP_10U  	   0x00
#define SET_RAMP_20U  	   0x01
#define SET_RAMP_40U 	     0x02
#define SET_RAMP_80U 	     0x03
#define SET_RAMP_200U 	   0x04
#define SET_RAMP_800U 	   0x05
#define SET_RAMP_1700U 	   0x06
#define SET_RAMP_3400U 	   0x07

#define  TxDone_IRQ  	     0x01
#define  RxDone_IRQ  	     0x02
#define  Pream_IRQ 		     0x04
#define  SyncWord_IRQ 	   0x08
#define  Header_IRQ  	     0x10
#define  HeaderErr_IRQ 	   0x20
#define  CrcErr_IRQ		     0x40
#define  CadDone_IRQ	     0x80
#define  CadDetec_IRQ	     0x0100
#define  Timeout_IRQ	     0x0200

#define  DIO3_1_6V  0x00
#define  DIO3_1_7V  0x01
#define  DIO3_1_8V  0x02
#define  DIO3_2_2V  0x03
#define  DIO3_2_4V  0x04
#define  DIO3_2_7V  0x05
#define  DIO3_3_0V  0x06
#define  DIO3_3_3V  0x07

typedef struct 
{
	uint8_t reach_tx;	
	uint8_t is_tx	;
	uint8_t rf_reach_timeout;
	uint8_t busy_timeout;
	uint8_t busy_timeout_cnt;
	uint8_t tx_buff_flag;
	uint8_t txbuf[128];
	uint8_t payload_length;
	uint8_t rf_timeout;
	uint16_t Irq_Status;
	struct RFCtrl
	{
	  uint8_t  RFmode;    //0:FSK,1:LoRa
	  uint8_t  SpreadingFactor;   //扩频因子,SF5~SF12
		uint8_t  Bandwidth; //LORA_BW_7~LORA_BW_500
		uint8_t  CodeRate;  //通信码率
	} RFCtrl;
	int8_t   RxRssi;
}FlagType;
extern FlagType  Sx1262_Flag;

void Sx1262_SPI_Init(void);
void reset_sx1262(void);

void sx1262_Config(void);
void Tx_Start(uint8_t *txbuf,uint8_t payload_length);
void Rx_Init(void);

void timerx_init(void);

void SetPacketType(uint8_t PacketType);
uint8_t GetPacketType(void);
uint16_t GetIrqStatus(void);
void ClearIrqStatus(uint16_t irq);
void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length);
void GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer);
void SetStandby(uint8_t StdbyConfig);
uint8_t GetRssiInst(void);



extern const char *rfName;
extern u16	iSend, iRev;

extern u8	sendBuf[25];
extern u8	revBuf[25];

u8 RFSendData( u8 *buf, u8 size );
u8 RFRevData( u8 *buf );
void RFRxMode( void );
void RFInit( void );

#endif
