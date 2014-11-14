/**
 *  Defines for your entire project at one place
 *
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

//Put your global defines for all libraries here used in your project

//T1 – temp. Na kolektorze
//T2 – wlot na wymiennik
//T3 – temperatura w buforze
//T4 – temperatura w CWU
//T5 – temperatura otoczenia na polu
//P1 – pompa kolektorów
//P2 – pompa dodatkowa kolektorów
//P3 – pompa CWU
//
//T1+6 >T3   	 P1 ON
//T2+3 <T3 	P1 OFF  Obieg d³ugi
//T2+3 < T4 P1 OFF Obieg krótki
//T2>T4+3	P3 ON
//T4>=52		P3 OFF
//T4<=42  to T1>T4+6
//T1>T3+25	P1+P2 ON
//T1>98 		P1+P2 ON
//Po spuszczeniu wody  P2 ON na oko³o 2 min

#define PD0ON (GPIOD->BSRRL = GPIO_Pin_0)
#define PD0OFF (GPIOD->BSRRH = GPIO_Pin_0)
#define PD1ON (GPIOD->BSRRL = GPIO_Pin_1)
#define PD1OFF (GPIOD->BSRRH = GPIO_Pin_1)
#define PD2ON (GPIOD->BSRRL = GPIO_Pin_2)
#define PD2OFF (GPIOD->BSRRH = GPIO_Pin_2)
#define PD3ON (GPIOD->BSRRL = GPIO_Pin_3)
#define PD3OFF (GPIOD->BSRRH = GPIO_Pin_3)
#define PD4ON (GPIOD->BSRRL = GPIO_Pin_4)
#define PD4OFF (GPIOD->BSRRH = GPIO_Pin_4)
#define PD5ON (GPIOD->BSRRL = GPIO_Pin_5)
#define PD5OFF (GPIOD->BSRRH = GPIO_Pin_5)
#define PD6ON (GPIOD->BSRRL = GPIO_Pin_6)
#define PD6OFF (GPIOD->BSRRH = GPIO_Pin_6)
#define PD7ON (GPIOD->BSRRL = GPIO_Pin_7)
#define PD7OFF (GPIOD->BSRRH = GPIO_Pin_7)
#define PD8ON (GPIOD->BSRRL = GPIO_Pin_8)
#define PD8OFF (GPIOD->BSRRH = GPIO_Pin_8)
#define PD9ON (GPIOD->BSRRL = GPIO_Pin_9)
#define PD9OFF (GPIOD->BSRRH = GPIO_Pin_9)
#define PD10ON (GPIOD->BSRRL = GPIO_Pin_10)
#define PD10OFF (GPIOD->BSRRH = GPIO_Pin_10)
#define PD11ON (GPIOD->BSRRL = GPIO_Pin_11)
#define PD11OFF (GPIOD->BSRRH = GPIO_Pin_11)
#define PD12ON (GPIOD->BSRRL = GPIO_Pin_12)
#define PD12OFF (GPIOD->BSRRH = GPIO_Pin_12)
#define PD13ON (GPIOD->BSRRL = GPIO_Pin_13)
#define PD13OFF (GPIOD->BSRRH = GPIO_Pin_13)
#define PD14ON (GPIOD->BSRRL = GPIO_Pin_14)
#define PD14OFF (GPIOD->BSRRH = GPIO_Pin_14)
#define PD15ON (GPIOD->BSRRL = GPIO_Pin_15)
#define PD15OFF (GPIOD->BSRRH = GPIO_Pin_15)



#endif
