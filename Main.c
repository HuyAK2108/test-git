#include "stm32f4xx.h"
#include "stdio.h"
#include "math.h"

#define BUFFER_SIZE_RX 101
#define BUFFER_SIZE_TX 17
#define BAUDRATE 9600
#define RES2 820.0
#define ECREF 200.0


int8_t rxbuff[BUFFER_SIZE_RX];
char txbuff[BUFFER_SIZE_TX];
unsigned int pulse=0,count=0;
int mintime,runtime;
int TH;
float lit=0;
double ph[5],ph0[5],EC;
GPIO_InitTypeDef GPIO_InitStructure;
volatile int counterstm0,counterstm;
int t,a=0,T,i,b;
int x=20;
int mintimeh[5],mintimem[5],cycle[5],program[5],v1[5],v2[5],waterNo[5],Amount[5];
float sth0,stm0,sth,stm;
int before[5],after[5],water[5];
int mintimeh0[5],mintimem0[5],cycle0[5],program0[5],v10[5],v20[5],waterNo0[5],Amount0[5];
float sth0,stm0;
int before0[5],after0[5],water0[5];
int stt=1,totaltime[5];
int realamount=1;
int avergearray(int* arr, int number);
int pHArray[40];   
int time=1;
int pHArrayIndex=0;    
uint16_t adc_data,adc_data1 ; // we need min. 12 bit variable to keep  8 bitadc data. 
uint16_t adc_convert,adc_convert1;
double acidVoltage    = 2032.44;    //buffer solution 4.0 at 25C
double neutralVoltage = 1500.0;     //buffer solution 7.0 at 25C
double slope,intercept, phValue,voltage;
//static float compECsolution;
double ecvalue                = 0.0;
double kvalue                 = 1.0;
double kvalueLow              = 1.0;
double kvalueHigh             = 1.0;
double cmdReceivedBufferIndex = 0;
double voltage1                = 0.0;
double temperature            = 25.0;
double rawEC;
double valueTemp = 0;
double KValueTemp;
float e;


/* TypeDef */
GPIO_InitTypeDef GPIO_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructre;
DMA_InitTypeDef  DMA_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
TIM_ICInitTypeDef TimerIC_Confi;

uint16_t ReadADC(void) //created a void to read ADC value
{
ADC_RegularChannelConfig(ADC2,ADC_Channel_0, 1,ADC_SampleTime_480Cycles);
	//set reading cycle, decreasing this value will increase the reading speed.

	ADC_SoftwareStartConv(ADC2); // starts softwares of  ADC2 module
	
	/* wait for the state */
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC2);
}
uint16_t ReadADC_EC(void) //created a void to read ADC value
{
ADC_RegularChannelConfig(ADC2,ADC_Channel_1, 1,ADC_SampleTime_480Cycles);
	//set reading cycle, decreasing this value will increase the reading speed.

	ADC_SoftwareStartConv(ADC2); // starts softwares of  ADC2 module
	
	/* wait for the state */
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC2);
}

/* Void */
void ADC_config(void);
void USART_Config(void);
void TIMCounterConfig(void);
void TimerCaptureConf(void);
void PCTransmission(void);
void GPIO_Configuration(void);
void Delay1Ms(void);
void Delay_Ms(uint32_t u32DelayInMs);
void control(void);
void measure(void);
void acid(void);
void flowrate(void);
void counter(void);
/* Union */
union rx{
	
	float speed_float;
	char bufferRX [4];
	
}rx_uni;


union tx{
	
	float txspeed_float;
	char bufferTX [4];
	
}tx_uni;

/* Main */
int main(){
	SystemInit();
	USART_Config();
	TimerCaptureConf();
	TIMCounterConfig();
	GPIO_Configuration();
	ADC_config();
	
	while(1){
		e++;
		measure();
		control();
		//counter();
		PCTransmission();
	}
}
void ADC_config(void){
		
	//LEDS connected on the STM32F4-DISCO board
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// ADC 1 module connected to the APB2 bus. You can check stm32f4xx_rcc.c and line: 1599. 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	
	
	//ADC Pin configurations.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // We specified as GPIO_Mode_AN to read analog data.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure) ;	
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // We set the ADC Mode as Independent to work.
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // We have set the prescaler value. Received data will be divided by 4.
	ADC_CommonInit(&ADC_CommonInitStructure); //Started the above configurations.
	
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b; // this means we are going to divide the value by 20mV.
	ADC_Init(ADC2, &ADC_InitStructure); //Set configuration of ADC2
	
	//Enable ADC2
	ADC_Cmd(ADC2, ENABLE);
	
}
void USART_Config(void){
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);
	// GPIO CONFIG
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP ;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed ;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP ;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed ;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_15;

  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = BAUDRATE;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;//Specifies wether the hardware flow control mode is enabled
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
  USART_InitStructure.USART_Parity = USART_Parity_No;/*When parity is enabled, the computed parity is inserted
                                            at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                               word length is set to 8 data bits). */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//Specifies the number of stop bits transmitted.
  USART_InitStructure.USART_WordLength = USART_WordLength_8b ;//Specifies the number of data bits transmitted or received in a frame
  USART_Init(USART1, &USART_InitStructure);
	
	// DMA
	USART_Cmd(USART1, ENABLE);

  // Enable USART1 DMA
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

  // DMA2 Stream2 Channel4 for USART1 Rx configuration
  DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_RX;			//RX_Buffer_size
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff	;		//dia chi muon truyen
	DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte	;			//quy dinh kich thuoc cua 1 phan tu
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable			;					//truyen xong 1 phan tu -> truyen sang phan tu tiep thep + tang dia chi len 1
	//MemoryInc: specifies whether memory address be incremented or not
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal 	;		//normal: truyen enough du liêu ->stop, circular: truyen het -> truyen lai tu dau
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR	; //dia chi cua ngoai vi, the address of Peripheral that wanna go
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//ko tang gia tri ngoai vi
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//do uu tien
	DMA_InitStructure.DMA_PeripheralBurst= DMA_PeripheralBurst_Single;
	
	DMA_Init (DMA2_Stream2,&DMA_InitStructure );
	DMA_Cmd (DMA2_Stream2, ENABLE);
	
	//Cau hinh DMA cho TX
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_TX;			//TX_Buffer_size
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff	;		//dia chi muon truyen
	DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte	;			//quy dinh kich thuoc cua 1 phan tu
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable			;					//truyen xong 1 phan tu -> truyen sang phan tu tiep thep + tang dia chi len 1
	//MemoryInc: specifies whether memory address be incremented or not
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal 	;		//normal: truyen enough du liêu ->stop, circular: truyen het -> truyen lai tu dau
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR	; //dia chi cua ngoai vi, the address of Peripheral that wanna go
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//ko tang gia tri ngoai vi
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//do uu tien
	DMA_InitStructure.DMA_PeripheralBurst= DMA_PeripheralBurst_Single;
	
	DMA_Init (DMA2_Stream7,&DMA_InitStructure );
	DMA_Cmd (DMA2_Stream7, ENABLE);
	
	// Enable DMA Interrupt to the highest priority
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Transfer complete interrupt mask
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	

}
void DMA2_Stream2_IRQHandler(void) {
	int i=0;
	b++;
  // Clear the DMA2_Stream2 TCIF2 pending bit
  DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
  //No new input after finish these package
  DMA_Cmd(DMA2_Stream2, DISABLE);
	/// Set pH ////
	for(i=0;i<=4;i++){
	for(t=0;t<4;t++){
				rx_uni.bufferRX[3-t]=rxbuff[t+1+x*i];
			}				
			ph0[i]=rx_uni.speed_float;
		}
	/// Amount ///
	for(i=0;i<=4;i++){
	Amount0[i]=rxbuff[6+x*i];
	}
//before
		for(i=0;i<=4;i++){
	before0[i]=rxbuff[7+x*i];
	}
	//water
		for(i=0;i<=4;i++){
	water0[i]=rxbuff[8+x*i];
	}
	//after
		for(i=0;i<=4;i++){
	after0[i]=rxbuff[9+x*i];
	}
		
	counterstm0=rxbuff[10];
	/// Program ///
	for(i=0;i<=4;i++){
	program0[i]=rxbuff[11+x*i];
	}
	// cycle//
	for(i=0;i<=4;i++){
	cycle0[i]=rxbuff[12+x*i];
	}
	// waterNo//
		for(i=0;i<=4;i++){
	waterNo0[i]=rxbuff[13+x*i];
		}
	//mintimem//
		for(i=0;i<=4;i++){
	mintimem0[i]=rxbuff[14+x*i];
		}
	//mintimeh//
			for(i=0;i<=4;i++){
	mintimeh0[i]=rxbuff[15+x*i];
			}
	// n//
	//n=rxbuff[16];
	sth0=rxbuff[17];
	stm0=rxbuff[18];
			for(i=0;i<=4;i++){
	v10[i]=rxbuff[19+x*i];
	v20[i]=rxbuff[20+x*i];
			}
			
		if(rxbuff[0]=='F'){
				for(i=0;i<=4;i++){
					ph[i]=ph0[i];
					Amount[i]=Amount0[i];
					before[i]=before0[i];
					water[i]=water0[i];
					after[i]=after0[i];
					program[i]=	program0[i];
					cycle[i]=cycle0[i];
					waterNo[i]=waterNo0[i];
					mintimem[i]=mintimem0[i];
					mintimeh[i]=mintimeh0[i];
					totaltime[i]=mintimeh[i]*3600 +mintimem[i]*60;
					v1[i]=v10[i];
					v2[i]=v20[i];
					sth=sth0;
					stm=stm0;
				}
		}
			
		if(stt==cycle[a]+1){
			stt=1;
			a++;
			GPIO_ResetBits(GPIOD, GPIO_Pin_14|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOB, GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);
		}
		
		if(b == totaltime[a]){
			stt++;
			b=0;
			lit=0;
			pulse=0;
			GPIO_ResetBits(GPIOD, GPIO_Pin_14|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOB, GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);
			//a++;	
		}
	DMA_Cmd(DMA2_Stream2, ENABLE);
}

void TIMCounterConfig(void){
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitStructre.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStructre.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStructre.TIM_Prescaler = 8400 - 1;


  TIM_TimeBaseInit (TIM6, & TIM_TimeBaseInitStructre);
  TIM_Cmd(TIM6, ENABLE);

  TIM6->CNT = 0;
}

void PCTransmission(void){
	if(TIM6-> CNT >=100){
		if(rxbuff[0]=='P'){
			tx_uni.txspeed_float =  phValue;
			txbuff[0]='P';
			for(t=0;t<4;t++){
				txbuff[1+t]= tx_uni.bufferTX[3-t];
			}
				tx_uni.txspeed_float = EC;
			for(t=0;t<4;t++){
				txbuff[7+t]= tx_uni.bufferTX[3-t];
			}
			
			tx_uni.txspeed_float = lit;
			for(t=0;t<4;t++){
				txbuff[11+t]= tx_uni.bufferTX[3-t];
			}
		}
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		DMA2_Stream7->NDTR=13;
		DMA_Cmd(DMA2_Stream7,ENABLE);
		TIM6->CNT=0;
	}
}
void GPIO_Configuration(void){
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); /*Bat clock khoi D*/
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
/* Configure PD12 PD13 in output pushpull mode */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15; /*Khai bao dung chan PD12->PD15*/
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; /*Xac dinh day la chan xuat*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*/Chon che do push pull*/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/*Chon toc do dau ra*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;/*Chon che do khong dung dien tro keo len*/
GPIO_Init(GPIOD, &GPIO_InitStructure) ;/*truyen cac doi so xuong thiet lap thanh ghi cho phan cung*/
	
	TIM_TimeBaseInitStructre.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInitStructre.TIM_Period= 0xFFFF;
	TIM_TimeBaseInitStructre.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructre);
	TIM_Cmd(TIM2, ENABLE);
}

void Delay1Ms(void){
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 1000){
	}
}

void Delay_Ms(uint32_t u32DelayInMs){
	while(u32DelayInMs){
		Delay1Ms();
		--u32DelayInMs;
	}
}

void acid(void){
		if(  phValue <ph0[0]-0.1){
			GPIO_ResetBits(GPIOD,GPIO_Pin_13);	// test bang den
			GPIO_ResetBits(GPIOB,GPIO_Pin_11);	// van acid
		}
		if(  phValue >=ph0[0]+0.1){
			GPIO_SetBits(GPIOD,GPIO_Pin_13);		// test bang den
			GPIO_SetBits(GPIOB,GPIO_Pin_11);		// van acid
		}
}

void measure(void){
		adc_data = ReadADC(); // we will have a value between 0 and 1024
		//----------------do pH----------------------------
		adc_convert = adc_data/1.64407;
		voltage = ((double)adc_convert/1024)*5000;
		slope = (7.0-4.0)/((neutralVoltage-1500.0)/3.0 - (acidVoltage-1500.0)/3.0); 
    intercept =  7.0 - slope*(neutralVoltage-1500.0)/3.0;
    phValue = slope*(voltage-1500.0)/3.0+intercept; 
		//-----------------do EC----------------------------
		adc_data1=ReadADC_EC();
		adc_convert1 = adc_data1/1.64407;
		voltage1 = ((double)adc_convert1/1024.0)*5000;
    rawEC = 1000*voltage1/RES2/ECREF;
		//-------------------calib for EC-----------------------------------
		/*if((rawEC>0.9)&&(rawEC<1.9)){                         //recognize 1.413us/cm buffer solution
                compECsolution = 1.413*(1.0+0.0185*(temperature-25.0));  //temperature compensation
            }
		else if((rawEC>9)&&(rawEC<16.8)){                    //recognize 12.88ms/cm buffer solution
                compECsolution = 12.88*(1.0+0.0185*(temperature-25.0));  //temperature compensation
							}
							 KValueTemp = RES2*ECREF*compECsolution/1000.0/voltage;  
							if((rawEC>0.9)&&(rawEC<1.9)){
                    kvalueLow =  KValueTemp;
                }else if((rawEC>9)&&(rawEC<16.8)){
                   kvalueHigh =  KValueTemp;
                }*/
								
		if((rawEC>0.9)&&(rawEC<1.9)){                         //recognize 1.413us/cm buffer solution
                kvalueLow= 164*1.413*(1.0+0.0185*(temperature-25.0))/voltage1;  //temperature compensation
            }
		else{                    //recognize 12.88ms/cm buffer solution
                 kvalueHigh = 164*12.88*(1.0+0.0185*(temperature-25.0))/voltage1;  //temperature compensation
							}
		//------------------------------------------------------
    valueTemp =rawEC * kvalue;
    if(valueTemp > 2.5){
       kvalue = kvalueHigh;
    }else if(valueTemp < 2.0){
       kvalue = kvalueLow;
    }
    EC= rawEC * kvalue;             //calculate the EC value after automatic shift
    EC = EC / (1.0+0.0185*(temperature-25.0));  //temperature compensation
		//---------------------------------------------------------
	}

void control(void){
		mintime=mintimeh[a]*3600+mintimem[a]*60-(before[a]+water[a]+after[a])*60;
		runtime=(before[a]+water[a]+after[a])*60;
		
		//BEFORE
		if(b>1 && b<=(before[a]*60)){
			GPIO_SetBits(GPIOD, GPIO_Pin_14);  // Test bang den D14 - Bat may bom
			GPIO_SetBits(GPIOB, GPIO_Pin_15);  // Bat may bom
		}
		
		//WATER
		if(b>1 && b>(before[a]*60) && b<=((before[a]+water[a])*60)){
			GPIO_SetBits(GPIOD, GPIO_Pin_14);  // Test bang den D14 - Bat may bom
			GPIO_SetBits(GPIOB, GPIO_Pin_15);  // Bat may bom
			
			/* Dieu khien van acid */ 
			acid();
			
			/* Do luu luong phan */
			flowrate();
			
			/* Dieu khien van phan */
			if(v1[a]==0 && v2[a]==1) TH=0;
			if(v1[a]==1 && v2[a]==0) TH=1;
			if(v1[a]==1 && v2[a]==1) TH=2;
			switch(TH){
				case 0:  // Van 2
					if(lit<Amount[a])
					{
					GPIO_SetBits(GPIOD, GPIO_Pin_12);	 // Test van 2 bang den
					GPIO_SetBits(GPIOB, GPIO_Pin_13);  // Bat van 2
					}
					else
					{
						GPIO_ResetBits(GPIOD, GPIO_Pin_12);
						GPIO_ResetBits(GPIOB, GPIO_Pin_13); // Tat van 2
					}
					break;
					
				case 1:  // Van 1
					if(lit<Amount[a])
					{
					GPIO_SetBits(GPIOD, GPIO_Pin_15);
					GPIO_SetBits(GPIOA, GPIO_Pin_15);  // Bat van 1
					}
					else
					{
						GPIO_ResetBits(GPIOD, GPIO_Pin_15);
						GPIO_ResetBits(GPIOA, GPIO_Pin_15); // Tat van 1
					}
					break;
					
				case 2: // Van 1-2
					if(lit<Amount[a])
					{
					GPIO_SetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_12);
					GPIO_SetBits(GPIOA, GPIO_Pin_15);  // Bat van 1
					GPIO_SetBits(GPIOB, GPIO_Pin_13);  // Bat van 2
					}
					else
					{
						GPIO_ResetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_12);
						GPIO_ResetBits(GPIOA, GPIO_Pin_15); // Tat van 1
						GPIO_ResetBits(GPIOB, GPIO_Pin_13); // Tat van 2
					}
					break;	
			}
		}
		
		//AFTER
		if(b>((before[a]+water[a])*60) && b<=((before[a]+water[a]+after[a])*60)){
			/* Safeguard*/
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15); // Test bang den D12,13,15 - Tat van phan va acid
			GPIO_ResetBits(GPIOA, GPIO_Pin_15); 							// Tat van 1
			GPIO_ResetBits(GPIOB, GPIO_Pin_11 | GPIO_Pin_13); // Tat van acid va van 2
		}
		
		//MINTIME
		if(b>(runtime) && b <=(runtime+mintime)){
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14| GPIO_Pin_15); // Tat het
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);  // Tat may bom
		}
		
		if(b==0){
			GPIO_ResetBits(GPIOD, GPIO_Pin_14|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOB, GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15);
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);
		}
}

void TimerCaptureConf(){
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
  GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_TIM3);
	
	// Timer Capture Configuration
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseInitStructre.TIM_Period = 10000 - 1 ;
  TIM_TimeBaseInitStructre.TIM_Prescaler = (8400 - 1);
  TIM_TimeBaseInitStructre.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStructre.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit (TIM3, &TIM_TimeBaseInitStructre);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
  // Timer Input Capture Configuration
  TimerIC_Confi.TIM_Channel = TIM_Channel_1 ;
  TimerIC_Confi.TIM_ICFilter = 15;
  TimerIC_Confi.TIM_ICPolarity = TIM_ICPolarity_Rising ;
  TimerIC_Confi.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(TIM3, &TimerIC_Confi);
  TIM_Cmd(TIM3, ENABLE);
}

void flowrate(){
	if(TIM3->SR & (1<<1)){
			pulse++;
			TIM3->SR &= ~(1<<1);
		}
	lit= pulse*0.00225;
}

void counter(){
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInitStructre.TIM_Period = 1000000-1;
	TIM_TimeBaseInitStructre.TIM_Prescaler = (uint16_t)(SystemCoreClock/2/1000000)-1;
	TIM_TimeBaseInitStructre.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructre.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructre);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM5, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM5_IRQHandler(void) {
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		//GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
		count++;
	}
}


