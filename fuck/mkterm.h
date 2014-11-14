/*
 * mkterm.h
 *
 *  Created on: 21-06-2014
 *      Author: Admin
 */

#ifndef MKTERM_H_
#define MKTERM_H_

// atrybuty znaku
#define RESET 			0
#define BOLD			1
#define DIM				2
#define UNDERLINE		4
#define BLINK  			5
#define REVERSE			7
#define HIDDEN 			8


// atrybuty kolorow

// kolory znakow lub tla
#define BLACK 			0
#define RED				1
#define GREEN			2
#define YELOW			3
#define BLUE			4
#define MAGENTA			5
#define CYAN			6
#define WHITE			7

#endif /* MKTERM_H_ */







void tr_cursor_hide(uint8_t hide);
void tr_cls(uint8_t _cursoronoff);
void tr_locate(uint8_t y , uint8_t x);
void tr_brushcolor(uint8_t cl);
void tr_pencolor(uint8_t cl);
void tr_attr( uint8_t atr, uint8_t fg,uint8_t bg);
