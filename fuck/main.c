
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_boot/stm32f4xx_conf.h"


#include "cmsis_lib/include/stm32f4xx_rcc.h"
#include "cmsis_lib/include/stm32f4xx_rtc.h"
#include "cmsis_lib/include/stm32f4xx_adc.h"
#include "cmsis_lib/include/stm32f4xx_dma.h"
#include "cmsis_lib/include/stm32f4xx_dac.h"
#include "mkterm.h"

#include "gpio.h"
#include "bitband.h"

#include "tm_stm32f4_hd44780.h"
#include "dacinit.h"
#include "defines.h"


// *******Dane do ADC
volatile  uint16_t ADC3ConvertedValue[2][3] ;
 typedef struct {
	uint32_t All;
	uint16_t v;
	uint16_t mv;
	}T_Voltage;
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)
#define NBR_ADC_CHANNEL		4
__IO uint16_t ADCValue[4];
__IO  T_Voltage ADCVoltage[4] ;

void ADC3_4CH_DMA_Config(void);
// *******************************

#define HSE_VALUE    ((uint32_t)8000000)
#define LED_GPIO							GPIOD		///< GPIO port to which the LED is connected
#define LED_pin								12			///< pin number of the LED

#define LED									(1 << LED_pin)
#define LED_ODR								LED_GPIO->ODR	///< output register for the LED
#define LED_bb								bitband_t m_BITBAND_PERIPH(&LED_ODR, LED_pin)	///< bit-band "variable" to directly handle the pin
GPIO_InitTypeDef  GPIO_InitStructure;
#define MAX_STRLEN 16


char received_string[16];
volatile uint_fast8_t cnt;

uint_fast8_t flaga_rs;
volatile uint32_t nTime;
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime );
void rtc_setup(void);
void getadam(uint8_t il);
volatile uint16_t Timer1 ;
volatile uint16_t Timer2;
volatile uint16_t Timer3 ;
volatile uint8_t sec ;
volatile uint8_t secpop ;
volatile  uint8_t stan;  // 0 woda spuszczona 1 woda w kolektorze nie grzeje 2 woda w kolektorze grzeje
RTC_TimeTypeDef RTC_TimeStruct;
RTC_TimeTypeDef time_rtc;
uint32_t TimingDelay ;
volatile uint8_t czasstart ;
volatile uint16_t czasstop ;
volatile uint8_t flagastart;
volatile uint16_t czaswodout ;
volatile uint16_t czaswodoutstart ;
uint8_t flagabieg;
volatile uint8_t flagazimno ;
static uint8_t tcwumax ;


const uint8_t  azymut7[12] = { 3, 11 , 25 , 43 , 67 , 95 , 122 , 143 , 160 , 173 , 185 , 196 };
//                             7   8   9    10   11   12    13    14   15     16    17    18
const uint8_t  elewacja7[12] = { 3, 11 , 25 , 43 , 67 , 90 , 67 , 43 , 25 , 11 , 3 , 3 };
//                               7   8   9    10   11   12   13   14   15   16   17  18

uint8_t flagaback ;
uint8_t azymutg ;
uint8_t elewacjag ;
uint8_t flagag ;
uint8_t hourpop ;
uint8_t flagasec ;
uint8_t flagaobieg ;
uint8_t hour ;
double liczba1;
double tpt100[3];
static double Adam[8];
char Adamas[8][16];
#define T1 Adam[7]     // T1 temperatura na kolektorze
#define T2 Adam[6]     // T2 temperatura na wymienniku
#define T3 22 //Adam[5]     // T3 temperatura w Buforze
#define T4  Adam[5]     // T4 temperatura w CWU
#define T5 Adam[1]  // T5 temperatura zewnêtrzna otoczenia
#define T6 Adam[6]
#define T7 Adam[7]
#define T8 Adam[8]

volatile char tabl_wynikow[9][16];

uint8_t kanal ;
uint8_t chanels_id;
//****** Usrednianie adc
#define  srdt 4
 uint32_t sradc[4];

//****** Usrednianie adc

typedef struct {
	uint16_t adcmid;
	uint16_t refadc;
	uint16_t adc ;
	uint16_t refv;
	char v1[3];
	char v2[4];
}TVOL;

float tempzPT100(float temperature , float vzero , float tkrok)
{
		float tpt100 = 0;
		tpt100 = (temperature - vzero)/ tkrok ;

		return tpt100;
}

void obiegkrotkion(uint8_t *c);
void obiegdlugion(uint8_t *c);
void pompabiegup(uint8_t *c) ;
void pompabiegdown(uint8_t *c) ;
void NVIC_Conf(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
  		// Jezeli tablica wektorow w RAM, to ustaw jej adres na 0x20000000
  		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
  		// W przeciwnym wypadku ustaw na 0x08000000
  		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

  	// Wybranie grupy priorytetów
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  	// Wlacz przerwanie od USART1
  	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

void SYSTICK_Conf(void);


void getchannels(uint8_t kanal);
void odbior_rs( char str[] , char str1[]);

void usart_init(void)
{
  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);

  /* Enable USART */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USART3, USART_IT_TC, ENABLE);
  USART_Cmd(USART3, ENABLE);

}

void Usart_Printf(void);
//****************Funkcja main*********************************************************************************
int main(void)
{
//	gpio_pin_cfg(LED_GPIO, LED_pin, GPIO_OUT_PP_25MHz);
//	gpio_pin_cfg(GPIOD, 11, GPIO_OUT_PP_25MHz);
	uint8_t i = 0;
	RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configure Pins on PortD in output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  |GPIO_Pin_1  |GPIO_Pin_2 |GPIO_Pin_3 |GPIO_Pin_4  |   GPIO_Pin_5  | GPIO_Pin_6 |GPIO_Pin_7| GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
//	    //  Inicjalizacja portu GPIOD jako wyjcie
//	    GPIOD->MODER |= GPIO_Mode_OUT;
//	    GPIOD->OSPEEDR |= GPIO_Speed_25MHz;
//	    GPIOD->OTYPER |= GPIO_OType_PP;
//	    GPIOD->PUPDR |= GPIO_PuPd_NOPULL;

    //automatically added by CoIDE
	NVIC_Conf();
	usart_init();
//	Usart_Printf();
	SYSTICK_Conf();
	rtc_setup();
	tr_cls(1);
	TIM6_Config();
	tr_attr(1,RED,BLACK);
	tr_locate(1,0);
	usart_puts(USART3, "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789");
				tr_locate(2,30);
				usart_puts(USART3, "startup..STER.OBROTNICY 1.1");
				usart_puts(USART3, "\r\n");
				tr_attr(0,WHITE,BLACK);

	ADC3_4CH_DMA_Config();  // Uruchomienie zczytywania ADC z 4 kana³ów

	Timer1 = 1000;
	Timer2 = 500;
	Timer3 = 1000;
//	czasstart = 20;
//	obiegdlugion();
	obiegkrotkion(&flagaobieg);
	stan = 0;
	tcwumax = 27 ;
// *********Pêtla g³ówna*****************************************
	while(1)
    {
	RTC_GetTime(0 , &time_rtc);
		if (USART_GetITStatus(USART3, USART_IT_TC)== RESET) GPIOD->BSRRH = GPIO_Pin_10;



		hour = time_rtc.RTC_Hours;
		sec = time_rtc.RTC_Seconds;

		if (sec != secpop){
			flagasec = 1;
			secpop = sec ;
		}

		if (flagasec == 1){
			if (czasstart != 0 ){
					czasstart--;
					czasstop--;
			}
//			if (czasstart != 0 ){
//					czasstart--;
//			}

			flagasec = 0 ;
		}

		if(czasstart != 0){
			PD15ON ;
		}else{
			PD15OFF;
			}
//
		if (hour != hourpop){
					flagag = 1 ;
					}

							uint8_t i;
							i = hour ;


					if (flagag != 0){
							switch (i)
							{

							case 7 :
									azymutg = azymut7[0];
								    elewacjag = elewacja7[0];
								    break ;
							case 8 :
									azymutg = azymut7[1];
									elewacjag = elewacja7[1];
									break ;
							case 9 :
									azymutg = azymut7[2];
									elewacjag = elewacja7[2];
									break ;
							case 10 :
									azymutg = azymut7[3];
									elewacjag = elewacja7[3];
									break ;
							case 11 :
									azymutg = azymut7[4];
									elewacjag = elewacja7[4];
									break ;
							case 12 :
									azymutg = azymut7[5];
									elewacjag = elewacja7[5];
									break ;
							case 13 :
									azymutg = azymut7[6];
									elewacjag = elewacja7[6];
									break ;
							case 14 :
									azymutg = azymut7[7];
									elewacjag = elewacja7[7];
									break ;
							case 15 :
									azymutg = azymut7[8];
									elewacjag = elewacja7[8];
									break ;
							case 16 :
									azymutg = azymut7[9];
									elewacjag = elewacja7[9];
									break ;
							case 17 :
									azymutg = azymut7[10];
									elewacjag = elewacja7[10];
									break ;
							case 18 :
									azymutg = azymut7[11];
									elewacjag = elewacja7[11];
									break ;

							case 21 :
									azymutg = azymut7[11];
									elewacjag = elewacja7[0];
									flagaback = 1 ;
										    break ;
							default :

									break ;
							}
						}
					if (hour != hourpop){
						hourpop = hour;
					}

		tr_attr(0,WHITE,BLACK);
		tr_locate(2,1);
//       Odczyt z Adam oraz przeliczenia z ADC procesora
		if (Timer1 == 0 ){

						getadam(8);
//						T1 = strtod(Adamas[0] ,NULL);
			            Adam[1] = strtod(Adamas[0] ,NULL);
			            Adam[2] = strtod(Adamas[1] ,NULL);
			            Adam[3] = strtod(Adamas[2] ,NULL);
			            Adam[4] = strtod(Adamas[3] ,NULL);
			            Adam[5]= strtod(Adamas[4] ,NULL);
			            Adam[6] = strtod(Adamas[5] ,NULL);
			            Adam[7]= strtod(Adamas[6] ,NULL);
			            Adam[8] = strtod(Adamas[7] ,NULL);

						Timer1 = 500;
					}

			sradc[0] = sradc[0] * srdt ;
			sradc[0] = sradc[0] + ADCValue[0] ;
			sradc[0] = sradc[0] / (srdt+1) ;
			sradc[1] = sradc[1] * srdt ;
			sradc[1] = sradc[1] + ADCValue[1] ;
			sradc[1] = sradc[1] / (srdt+1) ;
			sradc[2] = sradc[2] * srdt ;
			sradc[2] = sradc[2] + ADCValue[2] ;
			sradc[2] = sradc[2] / (srdt+1) ;
			sradc[3] = sradc[3] * srdt ;
			sradc[3] = sradc[3] + ADCValue[3] ;
			sradc[3] = sradc[3] / (srdt+1) ;



//

			tpt100[0] = (T2-2.500f) /0.000722f;
			tpt100[1] = tpt100[0] * 0.000722f;

//      Koniec  odczyt z Adam oraz przeliczenia z ADC procesora


// ***********Nape³nianie i odpowietrzanie kolektora**********************************
			if ((stan == 0) && ((T1 > T3) || (T1 > T4) )){
				flagastart = 1;
				stan =1 ;
				PD0ON ;
				pompabiegup(&flagabieg);
				czasstart = 20 ;
			}

			if ((flagastart == 1) && (czasstart == 0)){
				pompabiegdown(&flagabieg);
				PD0OFF ;
				flagastart = 0 ;
			}

// ***********Nape³nianie i odpowietrzanie kolektora**********************************

// Sterowanie pompami i wypuszczaniem wody z kolektora ******************************
			if (T5 <= 2){
				flagazimno = 1;
			}else{
				flagazimno = 0;
			}

	if ((stan == 1) && ((T1 > T3+6) || (T1 > T4+6) )){

		stan = 2 ;
		PD0ON ;     // W³¹czenie pompy kolektora po podniesieniu siê temperatury
	}

	if (flagastart == 0 && stan == 2 && flagaobieg == 0 && (T1 < T4)){

			stan = 1 ;
			PD0OFF ;     // Wy³¹czenie pompy kolektora po spadku temp
		}

	if (flagastart == 0 && stan == 2 && flagaobieg == 1 && (T1 < T3)){

			stan = 1 ;
			PD0OFF ;     // Wy³¹czenie pompy kolektora po spadku temp
		}

	if(T4 > tcwumax){
				obiegdlugion(&flagaobieg);
			}else{
				obiegkrotkion(&flagaobieg);   // Prz³¹czanie obiegów
	}

	if(!stan == 0  ){
		if(T4 > tcwumax){
			obiegdlugion(&flagaobieg);
		}else{
			obiegkrotkion(&flagaobieg);   // Prz³¹czanie obiegów
		}

	}


			// Sterowanie pompami i wypuszczaniem wody z kolektora ******************************
	void wyswietlanie(void){

					tr_attr(1,WHITE,BLACK);
		//
					tr_locate(6,10);
					usart_puts(USART3, "Zmienna T1 :"); usart_putfloat(USART3, T1);
					tr_locate(7,10);
					usart_puts(USART3, "Zmienna T2 :"); usart_putfloat(USART3, T2);
					tr_locate(6,40);
					usart_puts(USART3, "Zmienna T3 :"); usart_putfloat(USART3, T3);
					tr_locate(7,40);
					usart_puts(USART3, "Zmienna T4 :"); usart_putfloat(USART3, T4);
					tr_locate(8,10);
					usart_puts(USART3, "Elewacjag :"); usart_putint(USART3, elewacjag , 10);
					tr_locate(8,40);
					usart_puts(USART3, "Zmienna T5:"); usart_putfloat(USART3, T5 );
					tr_locate(9,10);
					usart_puts(USART3, "Azymut :"); usart_putint(USART3, azymutg , 10);
					tr_locate(10,10);
					usart_puts(USART3, "Zmienna czasstart :"); usart_putint(USART3, czasstart ,10);
					tr_locate(10,40);
					usart_puts(USART3, "Zmienna stan :"); usart_putint(USART3, stan ,10);
					tr_locate(11,10);
					usart_puts(USART3, "Zmienna flagabieg :"); usart_putint(USART3, flagabieg ,10);
					tr_locate(11,40);
					usart_puts(USART3, "Zmienna zimno :"); usart_putint(USART3, flagazimno ,10);
					tr_locate(12,10);
					usart_puts(USART3, "Zmienna start :"); usart_putint(USART3, flagastart ,10);
					tr_locate(12,40);
					usart_puts(USART3, "Zmienna flagaobieg :"); usart_putint(USART3, flagaobieg ,10);
					tr_locate(13,10);
					usart_puts(USART3, "Zmienna czasstop :"); usart_putint(USART3, czasstop ,10);






				tr_locate(14,10);
				usart_puts(USART3, "Temperatura z Adam1:");
				usart_putfloat(USART3, tpt100[0] );

				tr_locate(21,10);
				usart_puts(USART3, "Czas z zegara systemowego:");
//				tr_locate(21,36);usart_puts(USART3, "                                    ");
				tr_locate(21,36);
				usart_putint(USART3, time_rtc.RTC_Hours, 10);
				usart_putc(USART3, ':');
				usart_putint(USART3, time_rtc.RTC_Minutes ,10);
				usart_putc(USART3, ':');
				usart_putint(USART3, time_rtc.RTC_Seconds ,10);

				tr_locate(22,10);usart_puts(USART3, "           ");
				tr_locate(22,10);
				usart_putint(USART3, sradc[0],10);
				tr_locate(23,10);usart_puts(USART3, "           ");
				tr_locate(23,10);
				usart_putint(USART3, sradc[1],10);
				tr_locate(24,10);usart_puts(USART3, "           ");
				tr_locate(24,10);
				usart_putint(USART3, sradc[2],10);
				tr_locate(25,10);usart_puts(USART3, "           ");
				tr_locate(25,10);
				usart_putint(USART3, sradc[3],10);
				tr_locate(26,5);usart_puts(USART3, "Temperatura z ADC:"); usart_putfloat(USART3, tpt100[0] );
				tr_locate(26,40);usart_puts(USART3, "Napiêcie z ADC[0]:"); usart_putfloat(USART3, tpt100[1] );
		}
			wyswietlanie();
//		if (Timer1 == 0 ){
//
//
//						Timer1 =1000;
//					}



		 if(ADCValue[0] > 2000) {
			 PD12ON;
		    }
		    	else { PD12OFF;}

		    if(ADCValue[3] > 1200) {

		    	PD14ON;


		        }
		        	else {
		        		PD14OFF;
		        	}
//		    DAC_Ch1_EscalatorConfig();
		    uint16_t data = ADCValue[2];
//		    data = data * 12 ;
//		   DAC_SetChannel1Data(DAC_Align_12b_R, data);
//		   Wartoœæ_PSP = __MRS_PSP();

//		    DAC_Ch1_EscalatorConfig();
		//Delay(2900);

//		GPIOD->BSRRH = GPIO_Pin_12;  // GPIOX->BSRRL = GPIO_Pin_X : ustawienie 1 na pinie
//		Delay(500);					// GPIOX->BSRRH = GPIO_Pin_X : ustawienie 0 na pinie
//		GPIOD->BSRRL = GPIO_Pin_14;
//		Delay(1000);
//		GPIOD->BSRRH = GPIO_Pin_14;
//		Delay(1000);

    }
}
//****************End Funkcja main*******************************
void  SysTick_Handler(void)
{
	TimingDelay_Decrement();
//
	if ( Timer1 != 0x00 ){
		Timer1--;
	}
	if ( Timer2 != 0x00 ){
			Timer2--;
	}
	if ( Timer3 != 0x00 ){
			Timer3--;
	}
}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}



void USART3_IRQHandler(void){

	// check if the USART3 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART3->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != '\r') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;

			cnt++;

		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;



		}
//		 if (USART_GetITStatus(USART3, USART_IT_TC)== RESET){
//			 USART_ClearITPendingBit(USART3, USART_IT_TC);
//
////			 GPIOD->BSRRH = GPIO_Pin_10;
//		 }
	}
		uint8_t RS485_DELAY = 10 ;
		uint8_t delay = 0 ;
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		for (delay = 0; delay < RS485_DELAY; delay++) {

		}
		GPIOD->BSRRH = GPIO_Pin_10;
	//jesli  TXE jest puste
//	   if (USART_GetITStatus(USART1, USART_IT_TXE)!= RESET)
//	   {
//		   GPIOD->BSRRL = GPIO_Pin_10;
//	      USART_SendData(USART1, TxBuffer[licz++]);
//
//	      //jesli nie ma nic w buforze
//	      if(TxBuffer[licz-1] == 0x00)
//	           {
//	            licz=0;
//	            //Ustawiam przerwanie na disable
//	            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//	            GPIOD->BSRRH = GPIO_Pin_10;
//	           }
//	   }
}

void SYSTICK_Conf(void)
{
    #define FCPU                8000000
    #define    SYSTICK_Prescaler    8
    if (SysTick_Config((FCPU/SYSTICK_Prescaler)/1000))
        while(1);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

void odbior_rs( char str[] , char str1[])
{
		uint8_t i = 0;

	do
	{

		if((str1[i]!= 0) && (str1[i]!= '>') )
			{
				str[i] = str1[i];
			}
		else str[i]=' ';

	}while(str1[i++]);





}



void getchannels(uint8_t kanal)
	{
	switch (kanal){

	case 0:
		usart_puts(USART3, "#090");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
		chanels_id = kanal;
		Delay(60);
		break;

	case 1:
		usart_puts(USART3, "#091");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
			chanels_id = kanal;
			Delay(60);
			break;
	case 2:
		usart_puts(USART3, "#092");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
			chanels_id = kanal;
			Delay(60);
			break;
	case 3:
		usart_puts(USART3, "#093");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
			chanels_id = kanal;
			Delay(60);
			break;
	case 4:
		usart_puts(USART3, "#094");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
				chanels_id = kanal;
				Delay(60);
				break;
	case 5:
		usart_puts(USART3, "#095");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
				chanels_id = kanal;
				Delay(60);
				break;
	case 6:
		usart_puts(USART3, "#096");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
				chanels_id = kanal;
				Delay(60);
				break;
	case 7:
		usart_puts(USART3, "#097");    //odczyt z Adam-4019+
		usart_putc(USART3,'\r'); //odczodczyt z Adam-4018+
		usart_putc(USART3,'\n');
			chanels_id = kanal;
			Delay(60);
				break;
	default:
		break;
	 }
}

void getadam(uint8_t il){


		uint8_t i = 0;
		for(i=0;i<il+1 ;i++)
		{
			getchannels(i);
			odbior_rs(Adamas[chanels_id] ,  received_string);
		}
}



void rtc_setup(void){
	RTC_InitTypeDef   RTC_InitStructure;

	 // Clock auf LSE einstellen
	  RCC_LSEConfig(RCC_LSE_ON);

	  // warten bis Clock eingestellt
	  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	  // Clock enable fuer LSE
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	  // enable RTC
	  RCC_RTCCLKCmd(ENABLE);

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
      PWR_BackupAccessCmd(ENABLE);
	  RCC_HSEConfig( RCC_HSE_ON );
//	  RCC_LSEConfig(RCC_LSE_ON);
	  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)  {}
//	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
//	  RCC_RTCCLKCmd(ENABLE);
	  RTC_WaitForSynchro();
	  RTC_InitStructure.RTC_AsynchPrediv = 127;//0x7C;
	  RTC_InitStructure.RTC_SynchPrediv  = 256;//0x100;
	  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	  RTC_Init(&RTC_InitStructure);
	  RTC_TimeStruct.RTC_H12   = RTC_H12_PM;
	  RTC_TimeStruct.RTC_Hours   = 17;
	  RTC_TimeStruct.RTC_Minutes = 54;
	  RTC_TimeStruct.RTC_Seconds = 01;
//      RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
}

void ADC3_4CH_DMA_Config(void){
	//  ADC_InitTypeDef       ADC_InitStructure;
	//  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	//    DMA_InitTypeDef       DMA_InitStructure;

		ADC_InitTypeDef ADC_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		DMA_InitTypeDef       DMA_InitStructure;
		GPIO_InitTypeDef      GPIO_InitStructure;



	  ADC_DeInit();

	  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		/* DMA2 Stream0 channel0 configuration **************************************/
			DMA_InitStructure.DMA_Channel = DMA_Channel_0;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_ADDRESS;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) ADCValue;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_BufferSize = NBR_ADC_CHANNEL;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream0, &DMA_InitStructure);
			DMA_Cmd(DMA2_Stream0, ENABLE);


	  /* Configure ADC3 Channel7 pin as analog input ******************************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);



	  /* ADC Common Init **********************************************************/
			ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
			ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
			ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
			ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
			ADC_CommonInit(&ADC_CommonInitStructure);

	  /* ADC3 Init ****************************************************************/
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; /* 12b = 40    */
	  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfConversion = NBR_ADC_CHANNEL;
	  ADC_Init(ADC1, &ADC_InitStructure);

	  /* ADC3 regular channel7 configuration *************************************/
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles);
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles);
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles);
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles);

	  ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	  /* Enable DMA request after last transfer (Single-ADC mode) */
	  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	 /* Enable ADC1 DMA */
	 ADC_DMACmd(ADC1, ENABLE);

	  /* Enable ADC1 */
	  ADC_Cmd(ADC1, ENABLE);

	  ADC_SoftwareStartConv(ADC1);
}

void obiegkrotkion(uint8_t *c){
	if(*c != 0){
		PD6ON;
		Delay(200);
		PD6OFF;
		*c = 0 ;
	}
}

	void obiegdlugion(uint8_t *c){
		if(*c !=1){
			PD7ON;
			Delay(200);
			PD7OFF;
			*c = 1;
		}
}

void  pompabiegup(uint8_t *c){

	if (*c == 0){
		PD1ON;
		Delay(200);
		PD1OFF;
		*c = 1;

	}

}

void pompabiegdown(uint8_t *c){
	if (*c == 1){
		    PD1ON;
		    Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			Delay(200);
			PD1ON;
			Delay(200);
			PD1OFF;
			*c = 0;

	}
}
/*****Definicje pomocnicze niewykorzystywane w programie - sci¹ga do nauki*****/
//#define GPIO_Mode_IN  = 0 //GPIO Input Mode (0)
//#define GPIO_Mode_OUT = 1 //GPIO Output Mode (1)
//#define GPIO_Mode_AF  = 2 //GPIO Alternate function Mode (2)
//#define GPIO_Mode_AN  = 3 //GPIO Analog Mode (3)
//
//#define GPIO_OType_PP = 0 //Push-Pull (0)
//#define GPIO_OType_OD = 1 //Open-drain (1)
//
//#define GPIO_Speed_2MHz   = 0, //Low speed
//#define GPIO_Speed_25MHz  = 1 //Medium speed
//#define GPIO_Speed_50MHz  = 2 //Fast speed
//#define GPIO_Speed_100MHz = 3  //High speed on 30 pF (80 MHz Output max speed on 15 pF) */
//#define GPIO_PuPd_NOPULL = 0 //No Pull-up
//#define GPIO_PuPd_UP     = 1 //Pull-up (to VCC)
//#define GPIO_PuPd_DOWN   = 2 //Pull-down (to GND)
/***********************************KONIEC**************************************/

//************************PINY *************************************
//PC10 RSRX
//PC11 RSTX
//PD10 RSEN
//
//PA0 ADC1
//PA1 ADC2
//PA2 ADC3
//PA3 ADC4
//
//PA4 DAC1
//PA5 DAC2
//
//PD0 Pompa kolektora ON
//PD1 Pompa wymiennika ON
//PD2 Imp Reg pompy kolektora
//PD3
//PD4
//PD5
//PD6 Obieg krótki
//PD7 Obieg d³ugi
//PD11 Pompa biegi
//PD12 Elewacja start
//PD13 Elewacja kierunek
//PD14 Azymut start
//PD15 Azymut Kierunek
