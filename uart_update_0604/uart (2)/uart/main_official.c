//official version
#include "stm32f4xx.h"
#include "system_timetick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#define		BUFF_SIZE			100
#define   BUFF_SIZE_rx  125
#define   BUFFER_MAX_SIZE 128
#define   x              8
#define BUFFER_FULL 2 // Buffer day
#define BUFFER_OK 1 // Ham thuc hien thanh cong
#define BUFFER_EMPTY 0 // Buffer rong
#define BUFFER_NO_CR -1 // Ðoc buffer khong tim duoc <CR>
#define bool int 
#define true  1
#define false 0
#define EEPROM_ADDR 0x50
/* Lenh doc 30 thanh ghi tu thanh ghi 0x00 theo Modbus RTU */
uint8_t txbuff[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x3C, 0xF0,0x1B};
char 	rxbuff[BUFF_SIZE];
char 	rxbuff_3[BUFF_SIZE_rx];
uint8_t receive_data;
uint16_t pre_NDTR;
uint16_t Timingdelay;
uint8_t receive_eeprom[125];


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

typedef struct
{
USART_TypeDef *gate;
/* Bien de nhan du lieu */
char rxChar;
} uart_t;

typedef struct
{
/* Tên Port cua GPIO trong thu vien STM32F2xx_HAL.h */
GPIO_TypeDef *port;
/* Thu tu Pin cua GPIO */
uint16_t pin;
/* Trang thai tich cuc cua GPIO */
int8_t activeState;
/* Trang thai hien tai cua GPIO */
int8_t status;
} gpio_t;

typedef struct
{
/* Mang de luu du lieu*/
char array[BUFFER_MAX_SIZE];
/* Con tro doc */
uint32_t iR;
/* Con tro ghi */
uint32_t iW;
} buffer_t;


/* Bien luu các giá tri ba pha */
typedef struct
{
float phase1;
float phase2;
float phase3;
} threePhase_t;


/* Bien luu các giá tri dòng dien */
typedef struct
{
float lineVolt;
float neutralVolt;
float current;
float kW;
float kVA;
float kVAr;
float PF;
float frequency;
float kWh;
} electricity_t;


/* Bien dành cho SELEC */
typedef struct
{
uart_t uart;
buffer_t buffer;
bool requestSent;
bool gotResponse;
char rxString[125];
gpio_t driveEnable;
electricity_t electricity;
threePhase_t lineVolt;
threePhase_t neutralVolt;
threePhase_t current;
threePhase_t kW;
threePhase_t kVA;
threePhase_t kVAr;
threePhase_t PF;
} selec_t;


typedef struct
{
I2C_TypeDef *gate;
// dia chi thiet bi
uint8_t devAddress;
// kich thuoc o nho thiet bi
uint8_t memAddSize;
} i2c_t;

typedef struct
{
/* Cong I2C tuong ung voi EEPROM */
i2c_t i2c;
gpio_t writeProtect;
} eeprom_t;

void delay_01ms(uint16_t period);
void init_main(void);
void print_f(USART_TypeDef* USARTx,char *text, uint16_t length);
//bool SELEC_sendReadCMD(selec_t *selec);
float SELEC_dataConvert(char *firstByteAdd);
void E_Read (uint8_t Address, uint8_t *buffer, uint8_t size);
void E_Write (uint8_t Address, uint8_t Data);
int main(void)
{
	init_main();
		
//	  print_f(USART2,"AT\r\n",4);
//        delay_01ms(10000);
//		print_f(USART2,"AT+CPIN?\r\n",10);
//        delay_01ms(10000);
//    print_f(USART2,"AT+CSQ?\r\n",9);
//       delay_01ms(10000);
//    print_f(USART2,"AT+CGREG\r\n",10); 
//	      delay_01ms(10000);
//    print_f(USART2,"AT+COPS\r\n",9); 
//		    delay_01ms(30000);
//		print_f(USART2,"AT+CMQTTSTART\r\n",16);
//		    delay_01ms(30000);
//		    
//	  print_f(USART2,"AT+CMQTTACCQ=0,\"CLIENT1\"\r\n",strlen("AT+CMQTTACCQ=0,\"CLIENT1\"\r\n"));
//	      delay_01ms(10000);
//	  print_f(USART2,"AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com\",60,1\r\n",strlen("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com\",60,1\r\n"));
//	      delay_01ms(30000);
//	  print_f(USART2,"AT+CMQTTTOPIC=0,7\r\n",strlen("AT+CMQTTTOPIC=0,7\r\n"));
//	      delay_01ms(30000);
//	  print_f(USART2,"vinh/sys",strlen("vinh/sys"));
//	      delay_01ms(30000);
//		print_f(USART2,"AT+CMQTTPAYLOAD=0,5\r\n",strlen("AT+CMQTTPAYLOAD=0,5\r\n"));
//		    delay_01ms(30000);
//		print_f(USART2,"abcdf",strlen("abcdf"));
//		    delay_01ms(30000);
//		print_f(USART2,"AT+CMQTTPUB=0,1,60\r\n",strlen("AT+CMQTTPUB=0,1,60\r\n"));
//		    delay_01ms(10000);
	printf("AT\r\n");
	delay_01ms(10000);
	printf("AT+CPIN?\r\n");
	delay_01ms(10000);
	printf("AT+CSQ?\r\n");
	delay_01ms(10000);
	printf("AT+CGREG\r\n");
	delay_01ms(10000);
	printf("AT+COPS\r\n");
	delay_01ms(30000);
	printf("AT+CMQTTSTART\r\n");
	delay_01ms(30000);
	
	printf("AT+CMQTTACCQ=0,\"CLIENT1\"\r\n");
	delay_01ms(10000);
	printf("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com\",60,1\r\n");
	delay_01ms(30000);
	printf("AT+CMQTTTOPIC=0,7\r\n");
	delay_01ms(30000);
	printf("vinh/sys");
	delay_01ms(30000);
	printf("AT+CMQTTPAYLOAD=0,5\r\n");
	delay_01ms(30000);
	printf("abcdf");
	delay_01ms(30000);
	printf("AT+CMQTTPUB=0,1,60\r\n");
	delay_01ms(10000);
	while(1){
	
		/* Gui lenh doc thanh ghi */
if(tick_count == 100){
			tick_count = 0;
			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
			DMA1_Stream3->NDTR = x;
			DMA_Cmd(DMA1_Stream3, ENABLE);
		}	//return true;	

		E_Write (EEPROM_ADDR,receive_data);
    E_Read (EEPROM_ADDR,receive_eeprom, 125);
}
}

/* Delay */
//void delay_ms_sys(uint16_t time) {
//	Timingdelay = time;
//	while(Timingdelay!=0);
//}
void delay_01ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock /2 /(PSC+1) = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

/* Detect and get message length function */
uint16_t get_mess_from_rx(void)
{
	if(pre_NDTR != DMA1_Stream5->NDTR) 
		{
			pre_NDTR = DMA1_Stream5->NDTR;
			return 0;
		}
		if((pre_NDTR == DMA1_Stream5->NDTR)&&(pre_NDTR == BUFF_SIZE))
		{	
			return 0;
		}
		DMA_Cmd(DMA1_Stream5, DISABLE);		
		DMA1_Stream5->NDTR = (uint32_t)BUFF_SIZE;
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_Cmd(DMA1_Stream5, ENABLE);
		return (uint16_t)(BUFF_SIZE - pre_NDTR);
}

/* Transmit multi bytes in Tx function*/
void print_f(USART_TypeDef* USARTx,char *text, uint16_t length)
{
	for(int i = 0; i<length; i++)
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET) 
		{}
		USART_SendData(USARTx, (uint8_t)text[i]);
	}
}


	float SELEC_dataConvert(char *firstByteAdd)
{ 
uint16_t uint16_high = (uint16_t) * (firstByteAdd + 0) << 8 |*(firstByteAdd + 1);
/* Hop 2 bien char 8 bit thanh bien int 16 bit thap */
uint16_t uint16_low = (uint16_t) * (firstByteAdd + 2) << 8 |*(firstByteAdd + 3);
/* Hop 2 bien int 16 bit thanh bien long int 32 bit cao */
uint32_t uint32 = (uint32_t)uint16_high << 16 | uint16_low;
/* Ép kieu du lieu sang float 32 bit */
float float32 = *(float *)(&uint32);
return float32;
}

void DMA1_Stream2_IRQHandler( selec_t*selec)
{
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF2);
	
	for(uint8_t i=0;i<BUFF_SIZE_rx;i++)	
	{ 
	selec->rxString[i] = rxbuff_3[i];
	
selec->neutralVolt.phase1 = SELEC_dataConvert(selec->rxString + 5);
selec->neutralVolt.phase2 = SELEC_dataConvert(selec->rxString + 9);
selec->neutralVolt.phase3 = SELEC_dataConvert(selec->rxString + 13);
selec->electricity.neutralVolt= SELEC_dataConvert(selec->rxString+ 17);
selec->lineVolt.phase1 = SELEC_dataConvert(selec->rxString + 21);
selec->lineVolt.phase2 = SELEC_dataConvert(selec->rxString + 25);
selec->lineVolt.phase3 = SELEC_dataConvert(selec->rxString + 29);
selec->electricity.lineVolt = SELEC_dataConvert(selec->rxString +33);
selec->current.phase1 = SELEC_dataConvert(selec->rxString + 37);
selec->current.phase2 = SELEC_dataConvert(selec->rxString + 41);
selec->current.phase3 = SELEC_dataConvert(selec->rxString + 45);
selec->electricity.current = SELEC_dataConvert(selec->rxString +49);
selec->kW.phase1 = SELEC_dataConvert(selec->rxString + 53);
selec->kW.phase2 = SELEC_dataConvert(selec->rxString + 57);
selec->kW.phase3 = SELEC_dataConvert(selec->rxString + 61);
selec->kVA.phase1 = SELEC_dataConvert(selec->rxString + 65);
selec->kVA.phase2 = SELEC_dataConvert(selec->rxString + 69);
selec->kVA.phase3 = SELEC_dataConvert(selec->rxString + 73);
selec->kVAr.phase1 = SELEC_dataConvert(selec->rxString + 77);
selec->kVAr.phase2 = SELEC_dataConvert(selec->rxString + 81);
selec->kVAr.phase3 = SELEC_dataConvert(selec->rxString + 85);
selec->electricity.kW = SELEC_dataConvert(selec->rxString + 89);
selec->electricity.kVA = SELEC_dataConvert(selec->rxString + 93);
selec->electricity.kVAr = SELEC_dataConvert(selec->rxString + 97);
selec->PF.phase1 = SELEC_dataConvert(selec->rxString + 101);
selec->PF.phase2 = SELEC_dataConvert(selec->rxString + 105);
selec->PF.phase3 = SELEC_dataConvert(selec->rxString + 109);
selec->electricity.PF = SELEC_dataConvert(selec->rxString + 113);
selec->electricity.frequency = SELEC_dataConvert(selec->rxString +117);
selec->electricity.kWh = SELEC_dataConvert(selec->rxString + 121);
	
uint8_t receive_data = selec->rxString[i];}
	DMA_Cmd(DMA1_Stream1, ENABLE);
}

void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure; 
	DMA_InitTypeDef   DMA_InitStructure;
	NVIC_InitTypeDef	NVIC_TIM_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* GPIOC Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* GPIOC Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Enable USART2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	/* Enable USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
	/* Enable UART4 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 , ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* TIM7 Peripheral clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 839; //clk = SystemCoreClock /2/(PSC+1)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 9999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Configure PD12, PD13, PD14, PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  /* Connect UASRT2 pins to AF7 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 
	/* Connect UASRT3 pins to AF7 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); 
	/* Connect UART4 pins to AF8 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 

  /* GPIO Configuration for USART2 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* GPIO Configuration for USART3 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	

   /* GPIO Configuration for USART2 Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	 /* GPIO Configuration for USART3 Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
       
  /* USART2 configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
	USART_Init(USART3, &USART_InitStructure);
	/* Enable USART2 */
  USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART3, ENABLE);	
	
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	
	USART_Init(UART4, &USART_InitStructure);
	
	/* Enable UART4 */
  USART_Cmd(UART4, ENABLE);
	
	/* Enable USART2 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	
	/* DMA1 Stream5 Channel4 for USART2 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;   
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream5, ENABLE);
	/* DMA1 Stream3 Channel4 for USART3 Tx configuration */
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);	
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = x;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;   
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream3, ENABLE);
	/* DMA1 Stream3 Channel4 for USART3 Rx configuration */
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);	
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff_3; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_rx;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;   
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream1, ENABLE);
	/* Enable TIM Interrupt */
	NVIC_TIM_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_TIM_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_TIM_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_TIM_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TIM_InitStructure);
	
	/* Enable TIM7 */
	TIM_Cmd(TIM7, ENABLE);
	
	/* Enable IT TIM7 */
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	
	/* I2C1 configuration function */
		 I2C_InitTypeDef    I2C_InitStructure;
/* Enable GPIOA clock */	 
/* Enable I2C1 clock state */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);		
		
// Cau hinh chan SDA va SDL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// Cau hinh I2C
	I2C_InitStructure.I2C_Mode= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1=0;
	I2C_InitStructure.I2C_ClockSpeed=100000;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1,&I2C_InitStructure);
		
	I2C_Cmd(I2C1,ENABLE);
}


void TIM7_IRQHandler(void) 
{
	/*Clear flag timer 7*/
	TIM_ClearFlag(TIM7, TIM_IT_Update); 
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	
	/*Detect message from Rx then return length of message*/
	uint16_t length_mess = get_mess_from_rx();
	
	if(length_mess)
	{
		/*Take message from rx usart2*/
		char *ptr;
		ptr = (char *)calloc(100,1); //allocate 100 btyes memory
		for(int i = 0; i<length_mess; i++)
		{
			*(ptr+i) = rxbuff[i]; 
			
		}
		
		/*Indicate to uart4*/
		print_f(UART4, ptr, length_mess);
		free(ptr);
	}
	else
	{
		//
	}

}
void i2c_start(void)
		{
			I2C1->CR1 |= (1<<10);  // Enable the ACK
      I2C1->CR1 |= (1<<8);  // Generate START
		}
void i2c_write(uint8_t data){
      while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
      I2C1->DR = data;
      while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}
void i2c_sendaddress(uint8_t devAddress){
      I2C1->DR = devAddress ;  //  send the address
      while (!(I2C1->SR1 & (1<<1))); 
      uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 

}
void I2C_Stop (void)
{
	   I2C1->CR1 |= (1<<9);  // Stop I2C
}
void I2C_WriteMulti (uint8_t *data, uint8_t size)
{
	   while (!(I2C1->SR1 & (1<<7)));  // wait for TXE
	   while (size)
	{
		 while (!(I2C1->SR1 & (1<<7)));  // wait for TXE 
		 I2C1->DR = (uint32_t )*data++;  // send data
		 size--;
	}
	
	   while (!(I2C1->SR1 & (1<<2)));  // wait for BTF 
}


void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{	
	int remaining = size;
	
	if (size == 1)
	{
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);  // Stop I2C	
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER
		
	}
	
	else 
	{
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
		
		while (remaining>2)
		{
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
			
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer			
			
			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}
		
		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		I2C1->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	}	
}
void E_Write (uint8_t Address, uint8_t Data)
{
	i2c_start ();
	i2c_sendaddress (Address);
//	i2c_write (Reg);
	i2c_write(Data);
	I2C_Stop ();
}

void E_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
	i2c_start ();
	i2c_sendaddress (Address);
	//i2c_write (Reg);
	i2c_start  ();  // repeated start
	I2C_Read (Address+0x01, buffer, size);
	I2C_Stop ();
}
/* Hàm chuyen kieu ASCII sang BCD */
uint8_t string2BCD(char tenDigit, char oneDigit)
{
uint8_t bcdOut = 0;
bcdOut = (tenDigit - '0') << 4;
bcdOut |= (oneDigit - '0');
return bcdOut;
}
/* Hàm chuyen kieu BCD sang ASCII */
bool BCD2String(uint8_t bcd, char *desString)
{
desString[0] = (bcd >> 4) + '0';
desString[1] = (bcd & 0x0F) + '0';
return true;
}
/* Hàm chuyen kieu BCD sang DEC */
uint8_t BCD2DEC(uint8_t data)
{
return ((data >> 4) * 10 + (data & 0x0f));
}
uint8_t DEC2BCD(uint8_t data)
{
return ((data / 10) << 4 | (data % 10));
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/