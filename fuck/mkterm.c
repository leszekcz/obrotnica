/*
 * mkterm.c
 *
 *  Created on: 21-06-2014

 */



#include "cmsis_lib/include/stm32f4xx_usart.h"
#include "mkterm.h"


 char  UCLS[]  = { "\x1b""[2J" };
 char UHOME[]  = { "\x1b""[;H" };
 char UCURHIDE[]  = { "\x1b""[?25l" };
 char UCURSHOW[]  = { "\x1b""[?25h" };
 char UATROFF[]  = { "\x1b""[m" };



void tr_cursor_hide(uint8_t hide){
	if(hide) usart_puts(USART3, UCURHIDE);
	else usart_puts(USART3, UCURSHOW);
}

void tr_cls(uint8_t _cursoronoff){
	usart_puts(USART3,UATROFF);
	tr_cursor_hide(_cursoronoff);
	usart_puts(USART3, UCLS);
	usart_puts(USART3, UHOME);
}

void tr_locate(uint8_t y , uint8_t x){
	usart_putc(USART3, '\x1b' );
	usart_putc(USART3,'[');
	usart_putint(USART3,y , 10);
	usart_putc(USART3,';');
	usart_putint(USART3,x , 10);
	usart_putc(USART3,'H');
}

void tr_pencolor(uint8_t cl){
	usart_putc(USART3,'\x1b');
	usart_putc(USART3,'[');
	usart_putc(USART3,'3');
	usart_putc(USART3,cl+'0');
	usart_putc(USART3,'m');
}

void tr_brushcolor(uint8_t cl){
	usart_putc(USART3,'\x1b');
	usart_putc(USART3,'[');
	usart_putc(USART3,'4');
	usart_putc(USART3,cl+'0');
	usart_putc(USART3,'m');
}

void tr_attr( uint8_t atr, uint8_t fg,uint8_t bg){
	usart_putc(USART3,'\x1b');
	usart_putc(USART3,'[');
	usart_putc(USART3,atr+'0');
	usart_putc(USART3,';');
	usart_putc(USART3,'3');
	usart_putc(USART3,fg+'0');
	usart_putc(USART3,';');
	usart_putc(USART3,'4');
	usart_putc(USART3,bg+'0');
	usart_putc(USART3,'m');
}
