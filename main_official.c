//official version
#include "stm32f4xx.h"
#include "system_timetick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#define		BUFF_SIZE_UART2			128
#define   BUFF_SIZE_UART3  500
#define   BUFFER_MAX_SIZE 128
#define   x              8
#define BUFFER_FULL 2 // Buffer day
#define BUFFER_OK 1 // Ham thuc hien thanh cong
#define BUFFER_EMPTY 0 // Buffer rong
#define BUFFER_NO_CR -1 // �oc buffer khong tim duoc <CR>
#define true  1
#define false 0
//#define EEPROM_ADDR 0x50

#define TIME_STRUCT_SIZE 0x08
#define RAM_SIZE 55
#define BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#define BIN2BCD(val) ((((val)/10)<<4) + (val)%10)


#define RTC_DS1307_OK       0
#define RTC_DS1307_BAD     -1
#define eerom_OK            0
#define eerom_BAD           -1
#define DS1307_ADDRESS      0x68

#define SECOND_REGISTER    0x00
#define MINUTE_REGISTER    0x01
#define HOUR_REGISTER      0x02
#define DAY_REGISTER       0x03
#define DATE_REGISTER      0x04
#define MONTH_REGISTER     0x05
#define YEAR_REGISTER      0x06
#define CONTROL_REGISTER   0x07

#define SECOND_MASK        0x7F
#define MINUTE_MASK        0x7F
#define HOUR_MASK          0x3F
#define DAY_MASK           0x07
#define DATE_MASK          0x3F
#define MONTH_MASK         0x1F
#define YEAR_MASK          0xFF
#define FLAG_MASK          0xFF

#define RESET_FLAG_ON      0x80
#define RESET_FLAG_OFF     0x00
#define TRICKLE_SETTING    0xA7


#define MAX_STR_LEN 128
#define DEFAULT_WAIT_TIME 5000000u

#define EEPROMADDR          0x50 <<1
#define EEP24CXX_ADDR_WRITE	0xA0
#define EEP24CXX_ADDR_READ	0xA1
#define EEPROM_ADDRESS      0xA0
uint8_t I2c_Buf_Write[256];
uint8_t I2c_Buf_Read[256];
#define I2CT_FLAG_TIMEOUT  ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT  ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define  EEP_Firstpage      0x00
#define I2C_PageSize        8

/* Lenh doc 30 thanh ghi tu thanh ghi 0x00 theo Modbus RTU */
//const char txbuff[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x3C, 0xF0,0x1B};

  
unsigned int g_return_value;

float	neutralVolt_phase1;
float neutralVolt_phase2;
float neutralVolt_phase3;
float electricity_neutralVolt;
float lineVolt_phase1 ;
float	lineVolt_phase2;
float	lineVolt_phase3 ;
float electricity_lineVolt;
float	current_phase1 ;
float current_phase2 ;
float current_phase3 ;
float electricity_current ;
float kW_phase1 ;
float kW_phase2 ;
float kW_phase3 ;
float kVA_phase1 ;
float kVA_phase2 ;
float kVA_phase3;
float kVAr_phase1 ;
float kVAr_phase2 ;
float kVAr_phase3 ;
float electricity_kW ;
float electricity_kVA ;
float electricity_kVAr;
float PF_phase1 ;
float PF_phase2;
float PF_phase3 ;
float electricity_PF ;
float electricity_frequency;
float electricity_kWh ;


typedef struct {
    uint8_t year;       // year (0 - 99)
    uint8_t month;      // month (01 - 12)
    uint8_t date;       // date (01 - 31)
    uint8_t day;        // day (01 - 07)
    uint8_t hour;       // hour (00 - 23)
    uint8_t minutes;    // minutes (00 - 59)
    uint8_t seconds;    // seconds (00 - 59)
} rtc_ds1307_datetime_t;
/* Bien d�nh cho RTC */
typedef struct
{

I2C_TypeDef i2c;

rtc_ds1307_datetime_t time;

char timeStr[20];

char dateStr[20];

char dateTimeStr[20];
} rtc_t;

typedef struct
{
USART_TypeDef *gate;
/* Bien de nhan du lieu */
char rxChar;
} uart_t;

typedef struct
{

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


/* Bien luu c�c gi� tri ba pha */
typedef struct
{
float phase1;
float phase2;
float phase3;
} threePhase_t;


/* Bien luu c�c gi� tri d�ng dien */
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


/* Bien d�nh cho SELEC */
typedef struct
{
uart_t uart;
buffer_t buffer;
bool requestSent;
bool gotResponse;
char rxString[128];
gpio_t driveEnable;
electricity_t electricity;
threePhase_t lineVolt;
threePhase_t neutralVolt;
threePhase_t current;
threePhase_t kW;
threePhase_t kVA;
threePhase_t kVAr;
threePhase_t PF;
	float tt;
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

char echo[100];
bool m=0;
bool test=0;
bool relay=0;
/* var control tasks */
	bool enable_start_up=true;
	bool enable_start_mqtt=false;
	bool enable_pub_data = false;
/**/
char 	rxbuff_uart2[BUFF_SIZE_UART2];
char 	rxbuff_3[BUFF_SIZE_UART3];
char EEROM[500];
uint8_t ds_data;
/*  */
uint16_t pre_NDTR_USART2;
uint16_t pre_NDTR_USART3;
uint16_t Timingdelay;
char buffer[MAX_STR_LEN];
char buffer_selec[32];
uint32_t count;
uint16_t time_second=0;
void delay_01ms(uint16_t period);
void init_main(void);



void receive(USART_TypeDef* USARTx,char *text, uint16_t length);
float SELEC_dataConvert(char *firstByteAdd);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);

void display_rtc(rtc_ds1307_datetime_t *rtc_datetime);
unsigned int ds1307_bcd2bin (unsigned int bcd_value);
unsigned int ds1307_bin2bcd (unsigned int binary_value);
int ds1307_set_rtc_data (char register_value, char data);
int ds1307_get_rtc_data (char register_value, char register_mask,unsigned int *return_value);
void ds1307_set_rtc_second (unsigned int value);
void ds1307_set_rtc_minute (unsigned int value);
void ds1307_set_rtc_hour (unsigned int value);
void ds1307_set_rtc_day (unsigned int value);
void ds1307_set_rtc_date (unsigned int value);
void ds1307_set_rtc_month (unsigned int value);
void ds1307_set_rtc_year (unsigned int value);
void ds1307_reset_osf (void);
int ds1307_set_rtc_datetime (rtc_ds1307_datetime_t* datetime);
int ds1307_get_rtc_second (void);
int ds1307_get_rtc_minute (void);
int ds1307_get_rtc_hour (void);
int ds1307_get_rtc_day (void);
int ds1307_get_rtc_date (void);
int ds1307_get_rtc_month (void);
int ds1307_get_rtc_year (void);
int ds1307_get_rtc_datetime (rtc_ds1307_datetime_t *datetime);
int ds1307_init_rtc (int first_run_flag);
void USART_put(USART_TypeDef* USARTx, volatile char *s);
bool checkAddreses(const uint8_t start_address, const uint8_t allocated_bytes);
void write_eeprom (uint16_t Address, char *data,const uint8_t bytes);
void eeprom_writeData(rtc_ds1307_datetime_t *datetime,char register_value, const uint8_t bytes);
uint8_t* read_eeprom(uint16_t Address,const uint8_t allocated_bytes);
uint8_t* encodeData(const rtc_ds1307_datetime_t *datetime);
uint8_t* readTime(void);
void decodeTime(const uint8_t *data, rtc_ds1307_datetime_t *rtc_datetime);
//void write_24CXX (uint16_t Addr, uint16_t data);
//uint8_t read_24CXX(uint16_t Addr);
//void EEP24CXX_WriteMultiData(uint16_t Addr, uint16_t Quantity, uint8_t *Buff_Data);
//void EEP24CXX_ReadMultiData(uint16_t Addr, uint16_t Quantity, uint8_t *Buff_Data);
//void Wait_for_EEPROM(void);
uint16_t DMA_rx(void);

uint8_t eeprom_at24cxx_read_multi(uint8_t* memaddress,uint16_t address,uint16_t num_of_byte);
static uint8_t eeprom_at24cxx_write_page(uint8_t* memaddress,	uint16_t pageaddress,uint16_t num_of_byte);
uint8_t eeprom_at24cxx_write_multi(uint8_t* memaddress,uint16_t romaddress,uint16_t num_of_byte);
uint32_t lengthString(char *string);
	bool clearString(char *srcString);
/* Declare Func */	
void pwr(bool isSet);
void start_mqtt(uint16_t timeDelay);
void stop_mqtt(uint16_t timeDelay);
void subcribe_mqtt_topic(uint16_t timeDelay, char topic[]);
void public_data(uint16_t timeDelay, char topic[], uint8_t data[]);
bool detect_string(char stringIn[], char detectString[]);
void print_uart2(char *text, uint16_t length);
void print_uart4(char *text, uint16_t length);
void print_uart3(char *text, uint16_t length);
void send_SELEC(void);
uint8_t bcd2asc(uint8_t bcd);
int main(void){
//SysTick_Config(SystemCoreClock/1000);
	init_main();
	
	uint8_t Buff_Str[89];
  uint8_t Buff_Str1[89];
	uint8_t Buff_Str2[126];
	rtc_ds1307_datetime_t  rtc_datetime;
	 char buffer[MAX_STR_LEN];
    uint32_t count;   

	bool m=0;
	bool stop=0;
     ds1307_init_rtc(1);  // init the ds1307 RTC for first time
	//GPIO_SetBits(GPIOB,GPIO_Pin_2);
	while(1){
		/* Start Up */
		pwr(enable_start_up);
		
		/* Task 1: Check NET and start MQTT service */	
		if(enable_start_mqtt){
				start_mqtt(20000);
				/* Detect ERROR when connecting to MQTT Broker */
				while(detect_string(echo,"ERROR"))
				{				
					stop_mqtt(20000);
					start_mqtt(20000);
					/* Delete Received String after detecting error */
					clearString(echo);
				}	
				subcribe_mqtt_topic(20000,"VINH_1");
				/* Disable Task 1 and Enable Task 2 */
				enable_start_mqtt=false;
				enable_pub_data = true;
		}
		
		/* Task 2: Get data from cabinet and PUBLIC to SERVER */
		if(enable_pub_data){
			send_SELEC();
			public_data(20000, "VINH", Buff_Str1);
	}

//USART_put(USART3, "Main loop entered.\r\n");
        if (ds1307_get_rtc_datetime(&rtc_datetime) != RTC_DS1307_OK) {
            //USART_put(USART3, "!!!! not get ds1307 RTC datetime\n");
        }
       //display_rtc(&rtc_datetime);
           
				sprintf((char *) &Buff_Str,"{\"date\":\"%d\",\"month\":\"%d\",\"year\":\"%d\",\"day\":\"%d\",\"hour\":\"%d\",\"minutes\":\"%d\",\"seconds\":\"%d\"}",rtc_datetime.date, rtc_datetime.month,rtc_datetime.year,rtc_datetime.day,
 rtc_datetime.hour, rtc_datetime.minutes,rtc_datetime.seconds);
//				sprintf((char *) &Buff_Str,"%d-%d-%d-%d %d:%d:%d\r\n",rtc_datetime.date, rtc_datetime.month,rtc_datetime.year,rtc_datetime.day,
//  rtc_datetime.hour, rtc_datetime.minutes,rtc_datetime.seconds);
				eeprom_at24cxx_write_multi(Buff_Str,0x00,89);
//				
//       //delay_01ms(1000);
			  eeprom_at24cxx_read_multi((uint8_t*)Buff_Str1,0x00,89);
				
//				eeprom_at24cxx_write_multi(buffer_selec,0x91,126);
//			eeprom_at24cxx_read_multi((uint8_t*)Buff_Str2,0x91,126);
	count++;
//	print_f(USART3,(char *)&Buff_Str1,110);	
//	print_f(USART3,"\r\n",2);
//print_f(USART3,(char *)txbuff,8);
  delay_01ms(5000);
}

}
/* Func Start Up */
void pwr(bool isSet) {
	if(isSet) {
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
	}
	else {
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	}
}
		/* Execute AT Function */
/* Func Start MQTT Service - Delay time is ms and uint16_t max is 65535 */
void start_mqtt(uint16_t timeDelay) {
		print_uart2("AT+CMQTTSTART\r\n",strlen("AT+CMQTTSTART\r\n"));
		delay_01ms(timeDelay);
		print_uart2("AT+CMQTTACCQ=0,\"CLIENT1\"\r\n",strlen("AT+CMQTTACCQ=0,\"CLIENT1\"\r\n"));
	  delay_01ms(timeDelay);
		print_uart2("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com\",60,1\r\n",strlen("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com\",60,1\r\n"));
		delay_01ms(timeDelay);
}

/* Func Stop MQTT Service - Delay time is ms and uint16_t max is 65535 */
void stop_mqtt(uint16_t timeDelay) {
		print_uart2("AT+CMQTTREL=0\r\n",strlen("AT+CMQTTREL=0\r\n"));
		delay_01ms(timeDelay);
		print_uart2("AT+CMQTTSTOP\r\n",strlen("AT+CMQTTSTOP\r\n"));
		delay_01ms(timeDelay);
}

/* Func Subscribe a TOPIC  */
void subcribe_mqtt_topic(uint16_t timeDelay, char topic[]){
		print_uart2("AT+CMQTTSUBTOPIC=0,6,1\r\n",strlen("AT+CMQTTSUBTOPIC=0,6,1\r\n"));
	  delay_01ms(timeDelay);
		print_uart2(topic,strlen(topic));
	  delay_01ms(timeDelay);
		print_uart2("AT+CMQTTSUB=0,6,1\r\n",strlen("AT+CMQTTSUB=0,6,1\r\n"));
		delay_01ms(timeDelay);
		print_uart2(topic,strlen(topic));
	  delay_01ms(timeDelay);
}
/* Func Request Data From SELEC */
void send_SELEC(void){
		uint8_t txbuff[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x3C, 0xF0,0x1B};
    print_uart3((char *)txbuff, 8);	
}

/* Func Public Cabinet Data To Server */
void public_data(uint16_t timeDelay, char topic[], uint8_t data[]){
		print_uart2("AT+CMQTTTOPIC=0,4\r\n",strlen("AT+CMQTTTOPIC=0,4\r\n"));
	  delay_01ms(timeDelay);
	  print_uart2(topic,strlen(topic));
	  delay_01ms(timeDelay);
		print_uart2("AT+CMQTTPAYLOAD=0,89\r\n",strlen("AT+CMQTTPAYLOAD=0,89\r\n"));
		delay_01ms(timeDelay);
		print_uart2((char*)data,strlen((char*)data));
		delay_01ms(timeDelay);
		print_uart2("AT+CMQTTPUB=0,1,60\r\n",strlen("AT+CMQTTPUB=0,1,60\r\n"));
		delay_01ms(timeDelay);
}
		/* Execute Process Function */
/* Func Delay  */
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
/* Fuc Check String In String */
bool detect_string(char stringIn[], char detectString[]) {
		char *isDetected;
		isDetected = strstr(stringIn, detectString);
		//print_uart4(isDetected,1);
		if(isDetected == NULL)
		{		
			return false;
		}
		return true;		
}
/* Func Detect and get message length function */
uint16_t get_mess_from_rx(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t *pre_NDTRx, uint32_t buff_size, uint32_t flag){ 
	if(*pre_NDTRx != DMAy_Streamx->NDTR) 
		{
			*pre_NDTRx = DMAy_Streamx->NDTR;
			return 0;
		}
		if((*pre_NDTRx == DMAy_Streamx->NDTR)&&(*pre_NDTRx == buff_size))
		{	
			return 0;
		}
		DMA_Cmd(DMAy_Streamx, DISABLE);		
		DMAy_Streamx->NDTR = (uint32_t)buff_size;
		DMA_ClearFlag(DMAy_Streamx, flag);
		DMA_Cmd(DMAy_Streamx, ENABLE);
		return (uint16_t)(buff_size - *pre_NDTRx);
}
/* Func Convert BCD To ASCII */
uint8_t bcd2asc(uint8_t bcd){
	return bcd + 0x30;
}
		/* UART */
/* Func Transmit multi bytes in Tx in USART2 */
void print_uart2(char *text, uint16_t length){
	for(int i = 0; i<length; i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) 
		{}
		USART_SendData(USART2, (uint8_t)text[i]);
	}
}
/* Func Transmit multi bytes in Tx in USART3 */
void print_uart3(char *text, uint16_t length){
	for(int j = 0; j<length; j++)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) 
		{}
		USART_SendData(USART3, (uint8_t)text[j]);
	}
}
/* Func Transmit multi bytes in Tx in UART4 */
void print_uart4(char *text, uint16_t length){
	for(int j = 0; j<length; j++)
	{
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET) 
		{}
		USART_SendData(UART4, (uint8_t)text[j]);
	}
}
/* Func RTC Transmit */
void USART_put(USART_TypeDef* USARTx, volatile char *s) {
    while(*s){
		  // wait until data register is empty
		  while( !(USARTx->SR & 0x00000040) )
				;
		      USART_SendData(USARTx, *s);
		  *s++;
    }
}
/* */
void display_rtc(rtc_ds1307_datetime_t *rtc_datetime) {
    char buffer[MAX_STR_LEN];

//    sprintf( (char *) &buffer[0], "rtc_datetime\r\n\t->seconds = %d\r\n" \
//            "\t->minutes = %d\r\n\t->hour = %d\r\n\t->day = %d\r\n" \
//            "\t->date = %d\r\n\t->month = %d\r\n\t->year = %d\r\n\r\n", 
//            rtc_datetime->seconds, rtc_datetime->minutes, rtc_datetime->hour, 
//            rtc_datetime->day, rtc_datetime->date, rtc_datetime->month, 
//            rtc_datetime->year);

		sprintf((char *) &buffer[0],"%d-%d-%d-%d %d:%d:%d\r\n",rtc_datetime->date, rtc_datetime->month,rtc_datetime->year,rtc_datetime->day,
    rtc_datetime->hour, rtc_datetime->minutes,rtc_datetime->seconds);
		//USART_put(USART3, (char *) &buffer[0]);

    return;
}
/* */
float SELEC_dataConvert(char *firstByteAdd){ 
uint16_t uint16_high = (uint16_t) * (firstByteAdd + 0) << 8 |*(firstByteAdd + 1);
/* Hop 2 bien char 8 bit thanh bien int 16 bit thap */
uint16_t uint16_low = (uint16_t) * (firstByteAdd + 2) << 8 |*(firstByteAdd + 3);
/* Hop 2 bien int 16 bit thanh bien long int 32 bit cao */
uint32_t uint32 = (uint32_t)uint16_high << 16 | uint16_low;
/* �p kieu du lieu sang float 32 bit */
float float32 = *(float *)(&uint32);
return float32;
}




		/* Config STM32F407 */
void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure; 
	DMA_InitTypeDef   DMA_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	I2C_InitTypeDef    I2C_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
  
  /* GPIOA Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* GPIOB Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* GPIOC Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable I2C1 clock state */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	

	/* Enable USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
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
	
	/* Configure GPIOA in OUTPUT pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure GPIOB in OUTPUT pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	/* Configure GPIOD in OUTPUT pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
	/* GPIOA Configuration for USART1 & USART2 - (Pin_9 & Pin_10 for USART1, Pin_2 & Pin_3 for UART2, Pin_0 for UART4) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_9 | GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* GPIO Configuration for USART3 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Connect UASRT1 pins to AF7 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
  /* Connect UASRT2 pins to AF7 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 
	
	/* Connect UASRT3 pins to AF7 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	/* Connect UART4 pins to AF8 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
	
  /* 
		USART1, USART2 & UART4 configured as follow:
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
  USART_Init(USART1, &USART_InitStructure);
  USART_Init(USART2, &USART_InitStructure);
	
	/* Enable USART2 */
	USART_Cmd(USART1, ENABLE);
  USART_Cmd(USART2, ENABLE);
	
	/* Only Tx in UART4 */
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	
	USART_Init(UART4, &USART_InitStructure);
	
	/* Enable UART4 */
  USART_Cmd(UART4, ENABLE);
	
	/* DMA1 Stream5 Channel4 for USART2 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff_uart2; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_UART2;
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
 
	/* Enable USART2 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	
	/* 
		USART3 configured as follow:
		- BaudRate = 9600 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
	
	USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
	
	/* Enable USART3 */
  USART_Cmd(USART3, ENABLE);
		
	/* DMA1 Stream1 Channel4 for USART3 Rx configuration */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff_3; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_UART3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;   
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream1, ENABLE);	
	
	/* Enable USART3 DMA */
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	
  /* Config SDA and SCL pin for I2C1 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

  /* Config I2C1 */
	I2C_InitStructure.I2C_Mode= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_InitStructure.I2C_ClockSpeed=100000;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1,&I2C_InitStructure);
	
	/* Enable I2C1 */
	I2C_Cmd(I2C1,ENABLE);
	
	/* Timer 7: Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 839; //clk = SystemCoreClock /2/(PSC+1)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 9999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	/* Timer 7: Enable TIM Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable TIM7 */
	TIM_Cmd(TIM7, ENABLE);
	
	/* Enable IT TIM7 */
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	/* Configure PD0/PD1/PD2/PD3 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Connect EXTI Line0 to PD0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
	/* Connect EXTI Line1 to PD1 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	/* Connect EXTI Line2 to PD2 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);
	/* Connect EXTI Line3 to PD3 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
	
	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable and set EXTI Line1 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable and set EXTI Line2 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable and set EXTI Line3 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Configure EXTI Line0 - Line1 - Line2 - Line3 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	
}

	

		/* IRQ TIM7 */
void TIM7_IRQHandler(void) 
{
	/*Clear flag timer 7*/
	TIM_ClearFlag(TIM7, TIM_IT_Update); 
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	
	/*Detect message from Rx then return length of message*/
	uint16_t length_mess = get_mess_from_rx(DMA1_Stream5, &pre_NDTR_USART2, BUFF_SIZE_UART2, DMA_FLAG_TCIF5);
	
	if(length_mess)
	{
		/*Take message from rx usart2*/
		char *ptr;
		ptr = (char *)calloc(length_mess,1); 
		for(int i = 0; i<length_mess; i++)
		{
			*(ptr+i) = rxbuff_uart2[i];
			/**/
			echo[i] =rxbuff_uart2[i];
		}
		
		/* Check Net Light */
		if(enable_start_up){
			if(detect_string(echo,"DONE")){
				/* Disable Start Up and Enable Task 1 */
				enable_start_up = false;
				enable_start_mqtt=true;
			}
		}
		/* Check DO */
		else if (enable_start_mqtt | enable_pub_data){
			if(detect_string(echo,"ON1")){
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
			}
			else if(detect_string(echo,"OFF1")){
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			}
			else if(detect_string(echo,"ON2")){
				GPIO_SetBits(GPIOD, GPIO_Pin_13);
			}
			else if(detect_string(echo,"OFF2")){
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
			}
			else if(detect_string(echo,"ON3")){
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
			}
			else if(detect_string(echo,"OFF3")){
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
			}
		}
		/*Indicate to uart4*/
		print_uart4(ptr, length_mess);
		free(ptr);
	}
	/*rx_Selec*/
	uint16_t length = get_mess_from_rx(DMA1_Stream1,&pre_NDTR_USART3, BUFF_SIZE_UART3, DMA_FLAG_TCIF1);
	if(length)
	{
		/*Take message from rx usart2*/
		char *ptrr;
		ptrr = (char *)calloc(128,1); //allocate 100 btyes memory
		for(int i = 0; i<length; i++)
		{
			*(ptrr+i) = rxbuff_3[i]; 
			
		}
	float neutralVolt_phase1=SELEC_dataConvert(ptrr+5);
  float neutralVolt_phase2=SELEC_dataConvert(ptrr+9);
  float neutralVolt_phase3=SELEC_dataConvert(ptrr+13);
	float electricity_neutralVolt=SELEC_dataConvert(ptrr+17);
	float lineVolt_phase1 = SELEC_dataConvert(ptrr + 21);
	float	lineVolt_phase2 = SELEC_dataConvert(ptrr + 25);
	float	lineVolt_phase3 = SELEC_dataConvert(ptrr + 29);
	float electricity_lineVolt = SELEC_dataConvert(ptrr +33);
	float	current_phase1 = SELEC_dataConvert(ptrr + 37);
float current_phase2 = SELEC_dataConvert(ptrr+ 41);
float current_phase3 = SELEC_dataConvert(ptrr+ 45);
float electricity_current = SELEC_dataConvert(ptrr +49);
float kW_phase1 = SELEC_dataConvert(ptrr + 53);
float kW_phase2 = SELEC_dataConvert(ptrr + 57);
float kW_phase3 = SELEC_dataConvert(ptrr + 61);
float kVA_phase1 = SELEC_dataConvert(ptrr + 65);
float kVA_phase2 = SELEC_dataConvert(ptrr + 69);
float kVA_phase3 = SELEC_dataConvert(ptrr+ 73);
float kVAr_phase1 = SELEC_dataConvert(ptrr+ 77);
float kVAr_phase2 = SELEC_dataConvert(ptrr+ 81);
float kVAr_phase3 = SELEC_dataConvert(ptrr + 85);
float electricity_kW = SELEC_dataConvert(ptrr + 89);
float electricity_kVA = SELEC_dataConvert(ptrr + 93);
float electricity_kVAr = SELEC_dataConvert(ptrr + 97);
float PF_phase1 = SELEC_dataConvert(ptrr + 101);
float PF_phase2 = SELEC_dataConvert(ptrr + 105);
float PF_phase3 = SELEC_dataConvert(ptrr + 109);
float electricity_PF = SELEC_dataConvert(ptrr + 113);
 float electricity_frequency = SELEC_dataConvert(ptrr +117);
float electricity_kWh = SELEC_dataConvert(ptrr+ 121);
free(ptrr);
		sprintf((char *) &EEROM,"{\"V1N\":\"%.3f\",\"V2N\":\"%.3f\",\"V3N\":\"%.3f\",\"AV_LN\":\"%.3f\",\"V12\":\"%.3f\",\"V23\":\"%.3f\",\"V31\":\"%.3f\""
		",\"AV_LL\":\"%.3f\",\"I1\":\"%.3f\",\"I2\":\"%.3f\",\"I3\":\"%.3f\",\"AI\":\"%.3f\",\"KW1\":\"%.3f\",\"KW2\":\"%.3f\",\"KW3\":\"%.3f\""
	",\"KVA1\":\"%.3f\",\"KVA2\":\"%.3f\",\"KVA3\":\"%.3f\",\"KVAR1\":\"%.3f\",\"KVAR2\":\"%.3f\",\"KVAR3\":\"%.3f\",\"Total_KW\":\"%.3f\""
	",\"Total_KVA\":\"%.3f\",\"Total_KVAR\":\"%.3f\",\"PF1\":\"%.3f\",\"PF2\":\"%.3f\",\"PF3\":\"%.3f\",\"A_VF\":\"%.3f\",\"f\":\"%.3f\",\"KWH\":\"%.3f\"}"
			,neutralVolt_phase1,neutralVolt_phase2,neutralVolt_phase3,electricity_neutralVolt,lineVolt_phase1,lineVolt_phase2,lineVolt_phase3,
		electricity_lineVolt,current_phase1,current_phase2,current_phase3,electricity_current,kW_phase1,kW_phase2,kW_phase3,kVA_phase1,
		kVA_phase2,kVA_phase3,kVAr_phase1,kVAr_phase2,kVAr_phase3,electricity_kW,electricity_kVA,electricity_kVAr,PF_phase1,PF_phase2,PF_phase3,
		electricity_PF,electricity_frequency,electricity_kWh);
//		sprintf((char *) &EEROM,"neutralVolt_phase1=%f,neutralVolt_phase2=%f,neutralVolt_phase3=%f,electricity_frequency=%f\r\n",
//			neutralVolt_phase1,neutralVolt_phase2,neutralVolt_phase3,electricity_frequency);
//	USART_put(UART4, (char *) &EEROM);
//		receive(UART4, ptrr, length_mess);
//		free(ptrr);
	}
	else
	{
		//
	}
		
}

/* define lai dia chi cua eeprom */
	/* EXTI PD0 */
void EXTI0_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line0);
	uint8_t di_0 = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0);
	uint8_t  DI0[]={'D','I','0','=',bcd2asc(di_0)};
	eeprom_at24cxx_write_multi(DI0,0x59,5);
}

	/* EXTI PD1 */
void EXTI1_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line1);
	uint8_t di_1 = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1);
	uint8_t  DI1[]={'D','I','1','=',bcd2asc(di_1)};
	eeprom_at24cxx_write_multi(DI1,0x5E,5);
}
	/* EXTI PD2 */
void EXTI2_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line2);
	uint8_t di_2 = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2);
	uint8_t  DI2[]={'D','I','2','=',bcd2asc(di_2)};
	eeprom_at24cxx_write_multi(DI2,0x63,5);
}
	/* EXTI PD3 */
void EXTI3_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line3);
	uint8_t di_3 = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);
	uint8_t  DI3[]={'D','I','3','=',bcd2asc(di_3)};
	eeprom_at24cxx_write_multi(DI3,0x68,5);
}
/* H�m chuyen kieu ASCII sang BCD */
uint8_t string2BCD(char tenDigit, char oneDigit)
{
uint8_t bcdOut = 0;
bcdOut = (tenDigit - '0') << 4;
bcdOut |= (oneDigit - '0');
return bcdOut;
}
/* H�m chuyen kieu BCD sang ASCII */
bool BCD2String(uint8_t bcd, char *desString)
{
desString[0] = (bcd >> 4) + '0';
desString[1] = (bcd & 0x0F) + '0';
return true;
}

/* H�m chuyen kieu BCD sang DEC */
uint8_t BCD2DEC(uint8_t data)
{
return ((data >> 4) * 10 + (data & 0x0f));
}
uint8_t  DEC2BCD(uint8_t data)
{
return ((data / 10) << 4 | (data % 10));
}


void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
    // wait until I2C1 is not busy any more
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
        ;

    // Send I2C1 START condition
    I2C_GenerateSTART(I2Cx, ENABLE);

    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    // Send slave Address for write
    I2C_Send7bitAddress(I2Cx, address, direction);

    /* wait for I2Cx EV6, check if
     * either Slave has acknowledged Master transmitter or
     * Master receiver mode, depending on the transmission
     * direction
     */
    if (direction == I2C_Direction_Transmitter) {
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
            ;
    } else if (direction == I2C_Direction_Receiver) {
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
            ;
    }
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx) {
    // enable acknowledge of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2Cx);
    return data;
}


uint8_t I2C_read_nack(I2C_TypeDef* I2Cx) {
    // disable acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait until one byte has been received
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2Cx);
    return data;
}

void I2C_stop(I2C_TypeDef* I2Cx) {
    // Wait until end of transission
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
        ;
    // Send I2C1 STOP Condition after last byte has been transmitted
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
    // wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
        ;
    I2C_SendData(I2Cx, data);
}

int ds1307_set_rtc_data (char register_value, char data) {
    USART_put(USART3, "ds1307_set_rtc_data A entered\r\n");
    I2C_start(I2C1, (DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    USART_put(USART3, "ds1307_set_rtc_data B entered\r\n");
    I2C_write(I2C1, register_value);
    //USART_put(USART2, "ds1307_set_rtc_data B entered\r\n");
    I2C_write(I2C1, data);
    //USART_put(USART2, "ds1307_set_rtc_data B entered\r\n");
    I2C_stop(I2C1);
   // USART_put(USART2, "ds1307_set_rtc_data C entered\r\n");

    // otherwise success
    return RTC_DS1307_OK;
}
int ds1307_get_rtc_data (char register_value, char register_mask,
    unsigned int *return_value) {
    //uint8_t out_buff[1];
    uint8_t in_buff[1];
    //uint32_t status;

    I2C_start(I2C1, (DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    I2C_write(I2C1, register_value);
    I2C_stop(I2C1);

    I2C_start(I2C1, (DS1307_ADDRESS << 1), I2C_Direction_Receiver);
    in_buff[0] = I2C_read_nack(I2C1);
    
    // otherwise success
    *return_value = BCD2BIN(in_buff[0] & register_mask);
    return RTC_DS1307_OK;
}
		
void ds1307_set_rtc_second (unsigned int value) {
    ds1307_set_rtc_data(SECOND_REGISTER, (BIN2BCD(value) & SECOND_MASK));
}

void ds1307_set_rtc_minute (unsigned int value) {
    ds1307_set_rtc_data(MINUTE_REGISTER, (BIN2BCD(value) & MINUTE_MASK));
}

void ds1307_set_rtc_hour (unsigned int value) {
    ds1307_set_rtc_data(HOUR_REGISTER, (BIN2BCD(value) & HOUR_MASK));
}

void ds1307_set_rtc_day (unsigned int value) {
    ds1307_set_rtc_data(DAY_REGISTER, (BIN2BCD(value) & DAY_MASK));
}

void ds1307_set_rtc_date (unsigned int value) {
    ds1307_set_rtc_data(DATE_REGISTER, (BIN2BCD(value) & DATE_MASK));
}
void ds1307_set_rtc_month (unsigned int value) {
    ds1307_set_rtc_data(MONTH_REGISTER, (BIN2BCD(value) & MONTH_MASK));
}

void ds1307_set_rtc_year (unsigned int value) {
    ds1307_set_rtc_data(YEAR_REGISTER, (BIN2BCD(value) & YEAR_MASK));
}

int ds1307_set_rtc_datetime (rtc_ds1307_datetime_t* datetime) {
    // Set RTC datetime, the write format is documented on ds1307 
    // byte0: word address = 0x00
    // byte1: (seconds)
    // byte2: (minutes)
    // 3: day
    // 4: date (day of month)
    // 5: month
    // 6: year (0-99)
    I2C_start(I2C1, (DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    I2C_write(I2C1, 0x00);
    I2C_write(I2C1, BIN2BCD(datetime->seconds) & SECOND_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->minutes) & MINUTE_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->hour) & HOUR_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->day) & DAY_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->date) & DATE_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->month) & MONTH_MASK);
    I2C_write(I2C1, BIN2BCD(datetime->year) & YEAR_MASK);
    I2C_stop(I2C1);

    // otherwise success
    return RTC_DS1307_OK;
}

int ds1307_get_rtc_second (void) {
    return ds1307_get_rtc_data(SECOND_REGISTER, SECOND_MASK, &g_return_value);
}

int ds1307_get_rtc_minute (void) {
    return ds1307_get_rtc_data(MINUTE_REGISTER, MINUTE_MASK, &g_return_value);
}

int ds1307_get_rtc_hour (void) {
    return ds1307_get_rtc_data(HOUR_REGISTER, HOUR_MASK, &g_return_value);
}

int ds1307_get_rtc_day (void) {
    return ds1307_get_rtc_data(DAY_REGISTER, DAY_MASK, &g_return_value);
}

int ds1307_get_rtc_date (void) {
    return ds1307_get_rtc_data(DATE_REGISTER, DATE_MASK, &g_return_value);
}

int ds1307_get_rtc_month (void) {
    return ds1307_get_rtc_data(MONTH_REGISTER, MONTH_MASK, &g_return_value);
}

int ds1307_get_rtc_year (void) {
    return ds1307_get_rtc_data(YEAR_REGISTER, YEAR_MASK, &g_return_value);
}

int ds1307_get_rtc_datetime (rtc_ds1307_datetime_t *datetime)
{
    // seconds
    if (ds1307_get_rtc_second() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->seconds = g_return_value;

    // minutes
    if (ds1307_get_rtc_minute() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->minutes = g_return_value;

    // hours
    if (ds1307_get_rtc_hour() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->hour = g_return_value;

    // day
    if (ds1307_get_rtc_day() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->day = g_return_value;

    // date
    if (ds1307_get_rtc_date() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->date = g_return_value;

    // month
    if (ds1307_get_rtc_month() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->month = g_return_value;

    // year
    if (ds1307_get_rtc_year() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->year = g_return_value;

    // otherwise, success
    return RTC_DS1307_OK;
}
int ds1307_init_rtc (int first_run_flag) {
    // build the default (zero) starting state for the RTC
    // datetime structure.
    rtc_ds1307_datetime_t init_datetime = {
        .year = 21,
        .month = 05,
        .date = 12,
        .day = 03,
        .hour = 13,
        .minutes = 10
			,
        .seconds = 00
    };

   // USART_put(USART3, "ds1307_init_rtc entered\r\n");
    if (ds1307_set_rtc_data(CONTROL_REGISTER, 0) == -1) {
        return RTC_DS1307_BAD;
    }

    if (first_run_flag) {
        return ds1307_set_rtc_datetime(&init_datetime);
        
    }
    // otherwise return 0 to indicate success
    return RTC_DS1307_OK;
}


uint8_t eeprom_at24cxx_read_multi(uint8_t* memaddress,
	uint16_t address,uint16_t num_of_byte)
{
	uint8_t timeout;
	uint8_t addhigh = address>>8;
	uint8_t addlow = address&0xff;
		
	// start
	I2C_GenerateSTART(I2C1,ENABLE);
	timeout = 100;
	while ((!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))&&(timeout))
	{
		delay_01ms(100);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2C1,ENABLE);
		return 0;
	}
	
	// send address + W
	I2C_Send7bitAddress(I2C1,EEPROMADDR,I2C_Direction_Transmitter);
	timeout = 100;
	while ((!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout))
	{
		delay_01ms(100);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2C1,ENABLE);
		return 0;
	}
	
	// send reg address (dummy write)
	I2C_SendData(I2C1,addhigh);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	I2C_SendData(I2C1,addlow);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	
	// start repeated
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
	}
	
	// send address + R
	I2C_Send7bitAddress(I2C1,EEPROMADDR,I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
	}
	
	// read multi byte
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	while(num_of_byte)
	{
		if (num_of_byte == 1)
		{
			I2C_AcknowledgeConfig(I2C1,DISABLE);
		}
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
		}
		*memaddress = I2C_ReceiveData(I2C1);
		memaddress++;
		num_of_byte--;
	}	
	
	// stop
	I2C_GenerateSTOP(I2C1,ENABLE);
	delay_01ms(500);
	return 1;	
}


static uint8_t eeprom_at24cxx_write_page(uint8_t* memaddress,
	uint16_t pageaddress,uint16_t num_of_byte)
{
	uint8_t timeout;
	uint8_t addhigh = pageaddress>>8;
	uint8_t addlow = pageaddress&0xff;
		
	// start
	I2C_GenerateSTART(I2C1,ENABLE);
	timeout = 10;
	while ((!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))&&(timeout))
	{
		delay_01ms(100);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2C1,ENABLE);
		return 0;
	}
	
	// send address + W
	I2C_Send7bitAddress(I2C1,EEPROMADDR,I2C_Direction_Transmitter);
	timeout = 30;
	while ((!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout))
	{
		delay_01ms(100);
		timeout--;
	}
	if (timeout == 0)
	{
		I2C_GenerateSTOP(I2C1,ENABLE);
		return 0;
	}
	
	// send reg address 
	I2C_SendData(I2C1,addhigh);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}
	I2C_SendData(I2C1,addlow);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	}

	// write multi byte
	while(num_of_byte)
	{
		I2C_SendData(I2C1,*memaddress);
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}
		memaddress++;
		num_of_byte--;
	}	
	
	// stop
	I2C_GenerateSTOP(I2C1,ENABLE);
	return 1;
}



uint8_t eeprom_at24cxx_write_multi(uint8_t* memaddress,
	uint16_t romaddress,uint16_t num_of_byte)
{
	uint16_t num_of_byte_write;
	uint8_t write_result;
	uint8_t first_page_remain = 32 - romaddress&0x001F;
	uint8_t first_page = 1;
	
	while(num_of_byte)
	{
		if (first_page)
		{
			if (num_of_byte > first_page_remain)
			{
				num_of_byte_write = first_page_remain;
			}
			else
			{
				num_of_byte_write = num_of_byte;
			}
			first_page = 0;
		}
		else
		{
			if (num_of_byte > 32)
			{
				num_of_byte_write = 32;
			}
			else
			{
				num_of_byte_write = num_of_byte;
			}
		}
		
		num_of_byte -= num_of_byte_write;
		
		write_result = eeprom_at24cxx_write_page(memaddress,romaddress,num_of_byte_write);
		memaddress += num_of_byte_write;
		romaddress += num_of_byte_write;
		delay_01ms(500);;
		if (write_result == 0)
		{
			return 0;
		}
	}
	
	delay_01ms(500);
	return 1;
}
	bool clearString(char *srcString)
{
    uint32_t length = lengthString(srcString);
    for (uint32_t i = 0; i < length; i++)
        srcString[i] = 0;
    return true;
}
uint32_t lengthString(char *string)
{
    uint32_t length = 0;
    while ((string[length] != 0) && (length < 2048))
        length++;
    return length;
}

