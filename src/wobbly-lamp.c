#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

const uint8_t PROGMEM gamma[] = {
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
	1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
	2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
	5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
	10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
	25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
	51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
	69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
	115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
	144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
	177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
	215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255};

#define	RED		OCR1A
#define GREEN	OCR1B
#define BLUE	OCR2

void init_lights() {
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
	PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
	// timers on /8 prescalers, fastPWM
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11);
	TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21);
	RED = GREEN = BLUE = pgm_read_byte(&gamma[0x80]);
}

/***** General 3D config */
#define DOF 3
int16_t accel[DOF] = {0,0,0};	// X, Y, Z
uint8_t axis = 0;	// currently measured axis
uint8_t zerog = 0;	// 0g indicator
int16_t rest[DOF] = {0,0,0};

/***** Accelerometer's IO config */
// use Vcc as reference, left-adjust result (8b precision)
#define GENERAL_MUX (1 << REFS0) | (1 << ADLAR)
uint8_t mux_axes[DOF] = {
	(1 << MUX1),	// x @ ADC2
	(1 << MUX0),	// y @ ADC1
	0,				// z @ ADC0
};
#define LOWER_SENS_THR 0xa0
#define RAISE_SENS_THR 0x20
uint8_t inlowsensmode = 0;	// 6g instead of 1.5g enabled

inline void go_lowsens() {
	PORTB |= (1 << PB6);
	inlowsensmode = 1;
}

inline void go_hisens() {
	PORTB &= ~(1 << PB6);
	inlowsensmode = 0;
}

void init_accel() {
	DDRC = 0xff & ~((1 << PC2) | (1 << PC1) | (1 << PC0));	// X, Y, Z analog
	DDRD &= ~(1 << PD2);									// 0g digital
	DDRB |= (1 << PB6);										// g-select digital output
	go_lowsens();
	ADMUX = GENERAL_MUX | mux_axes[axis];
	// enable ADC, turn on interrupt, /128 prescaler giving 62,5kHz
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// start next conversion
	ADCSRA |= (1 << ADSC);
}


ISR(ADC_vect) {
	static uint8_t clk = 0;	// interrupt counter
	static uint8_t inc_sens = 1, dec_sens = 0;
	int16_t val;

	// read current axis
	val = ADCH - 0x80;	// normalize to 0
	if (inlowsensmode) val = val << 2;	// 6g to 1.5g

	// store the value
	accel[axis] = val;
	// next axis
	axis = (axis + 1) % DOF;

	// If ANY axis had high value, we should probably go to lower sensitivity mode.
	// If ALL axes have low values, we might enable hi sensitivity mode.
	if (abs(val) > LOWER_SENS_THR) dec_sens = 1;
	if (abs(val) >= RAISE_SENS_THR) inc_sens = 0;

	if (axis == 0) {	// once all axes have been probed
		// set sensitivity
		if (!inlowsensmode && dec_sens) {
			// TODO: never happens. why?
			go_lowsens();
		}
		if (inlowsensmode && inc_sens && !dec_sens) {
			go_hisens();
		}

		// reset all local flags
		inc_sens = 1;
		dec_sens = 0;
	}

	// reset all MUX bits then set to next axis
	ADMUX = GENERAL_MUX | mux_axes[axis];

	// start conversion again
	ADCSRA |= (1 << ADSC);

	clk++;
}


int main()
{
	uint8_t ax;
	init_lights();
	sei();
	init_accel();
	// make sure we have some stable measurement
	_delay_ms(100);
	// store current position as rest
	for (int ax=0; ax<DOF; ax++) {
		rest[ax] = accel[ax];
	}
	for (uint8_t i=0;;i++) {
		int16_t lvl[DOF], diff[DOF];
		_delay_ms(100);
		// deflections to levels
		for (ax=0; ax<DOF; ax++) {
			diff[ax] = accel[ax] - rest[ax];
			lvl[ax] = diff[ax] + 0x80;	// set zero at 50% power
		}
		// equalize total light amount
		// what is added to one channel gets subtracted from others
		for (ax=0; ax<DOF; ax++) {
			for (uint8_t bx=0; bx<DOF; bx++) {
				if (ax!=bx) lvl[ax] -= diff[bx] / (DOF - 1);
			}
		}
		// make sure levels are in bounds
		for (ax=0; ax<DOF; ax++) {
			if (lvl[ax] < 0) lvl[ax] = 0;
			if (lvl[ax] > 0xff) lvl[ax] = 0xff;
		}
		RED = pgm_read_byte(&gamma[(uint8_t)lvl[0]]);
		GREEN = pgm_read_byte(&gamma[(uint8_t)lvl[1]]);
		BLUE = pgm_read_byte(&gamma[(uint8_t)lvl[2]]);
	}
	return 0;
}
