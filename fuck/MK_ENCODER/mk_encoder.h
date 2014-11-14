/*
 * mk_encoder.h
 *
 *  Created on: 15-04-2013
 *      Author: Miros³aw Kardaœ
 */
#ifndef MK_ENCODER_H_
#define MK_ENCODER_H_

// definicja pinów enkodera
#define ENC_PH_A 	(1<<PC7)
#define ENC_PH_B 	(1<<PC6)
#define ENC_PORT 	C

// definicja pinu przycisku enkodera
#define ENC_SW		(1<<PC5)
#define ENC_SWPORT	C

//-----------------------------------

// wybór timera 0 lub 2
#define TIMER		0
//-----------------------------------


#define PHASE_A         (PIN(ENC_PORT) & ENC_PH_A)
#define PHASE_B         (PIN(ENC_PORT) & ENC_PH_B)

#if TIMER == 0
	#define ENCODER_vect TIMER0_COMP_vect
#endif

#if TIMER == 2
	#define ENCODER_vect TIMER2_COMP_vect
#endif




// Makra upraszczaj¹ce dostêp do portów
// *** PORT
#define PORT(x) SPORT(x)
#define SPORT(x) (PORT##x)
// *** PIN
#define PIN(x) SPIN(x)
#define SPIN(x) (PIN##x)
// *** DDR
#define DDR(x) SDDR(x)
#define SDDR(x) (DDR##x)


void register_enc_callback( void (*callback)( int8_t dir, int8_t cnt ) );
void register_sw_callback( void (*callback)( void ) );

void ENCODER_EVENT( int8_t res );

void encoder_init( uint8_t on_off );


#endif /* MK_ENCODER_H_ */
