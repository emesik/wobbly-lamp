#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// SENSORS is the number of analog inputs we read, while first DOF of them
// are accelerometer's data
#define SENSORS 4
#define DOF 3
#define DECAY_IDX 3
int16_t analog[SENSORS] = {0,0,0,0};	// X, Y, Z, decay
int16_t trail_deflection[DOF] = {0,0,0};	// trailing deflection, subject to decaying
int16_t rest[DOF] = {0,0,-0x28};	// Z adjustment for Earth's g
#define KICK_THRESHOLD 12
#define REST_THRESHOLD 4

// use Vcc as reference, left-adjust result (8b precision)
#define GENERAL_MUX (1 << REFS0) | (1 << ADLAR)
#define ALL_MUX_MASK ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0))
const uint8_t PROGMEM mux_sensors[SENSORS] = {
	(1 << MUX1),	// x @ ADC2
	(1 << MUX0),	// y @ ADC1
	0,				// z @ ADC0
	(1 << MUX1) | (1 << MUX0),	// decay @ ADC3
};
#define LOWER_SENS_THR 0xa0
#define RAISE_SENS_THR 0x20
uint8_t inlowsensmode = 0;	// 6g instead of 1.5g enabled

int16_t level[DOF] = {0x80, 0x80, 0x80};
uint16_t decay_period = 0x10;

const uint8_t PROGMEM gamma[0x100] = {
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

// the default white balance
const int16_t PROGMEM bias[DOF] = {10,6,-6};

#define	RED		OCR1A
#define	GREEN	OCR1B
#define	BLUE	OCR2

void init_lights() {
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
	PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
	// timers on /8 prescalers, phase-correct PWM
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << CS11);
	TCCR2 = (1 << WGM20) | (1 << COM21) | (1 << CS21);
	RED = GREEN = BLUE = 0;
}

inline void go_lowsens() {
	PORTB |= (1 << PB6);
	inlowsensmode = 1;
}

inline void go_hisens() {
	PORTB &= ~(1 << PB6);
	inlowsensmode = 0;
}

void init_sensors() {
	// X, Y, Z, decay inputs
	DDRC = 0xff & ~((1 << PC3) | (1 << PC2) | (1 << PC1) | (1 << PC0));
	DDRD &= ~(1 << PD2);									// 0g digital
	DDRB |= (1 << PB6);										// g-select digital output
	go_lowsens();
	ADMUX = GENERAL_MUX | pgm_read_byte(&mux_sensors[0]);
	// enable ADC, turn on interrupt, /128 prescaler giving 62,5kHz
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// start first conversion
	ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
	static uint8_t inc_sens = 1, dec_sens = 0;
	int16_t val;
	uint8_t sensor = 0;	// currently measured axis

	// check which axis has been measured
	for (sensor = 0; sensor < DOF; sensor++) {
		if ((ADMUX & ALL_MUX_MASK) == pgm_read_byte(&mux_sensors[sensor]))
			break;
	}

	// read the analog value
	val = ADCH;
	if (sensor < DOF) {
		// X, Y, Z get normalized to 0
		val -= 0x80;
		// rescale 6g to 1.5g if we are in low sensitivity mode
		if (inlowsensmode) val = val * 4;
	}

	// store the value
	analog[sensor] = val;
	// next input
	sensor = (sensor + 1) % SENSORS;

	// If ANY axis had high value, we should probably go to lower sensitivity mode.
	// If ALL axes have low values, we might enable hi sensitivity mode.
	if (sensor < DOF) {
		if (abs(val) > LOWER_SENS_THR) dec_sens = 1;
		if (abs(val) >= RAISE_SENS_THR) inc_sens = 0;
	}

	if (sensor == 0) {	// once all sensors have been probed
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

		for (uint8_t i=0; i < DOF; i++) {
			int16_t defl;
			// if current deflection from rest is bigger than the decaying one,
			// use it as the new value
			defl = analog[i] - rest[i];
			if (abs(defl) < REST_THRESHOLD) defl = 0;
			if (abs(defl) > abs(trail_deflection[i]) + KICK_THRESHOLD)
				trail_deflection[i] = defl;
		}
	}

	// reset all MUX bits then set to next axis
	ADMUX = GENERAL_MUX | pgm_read_byte(&mux_sensors[sensor]);

	// start conversion again
	ADCSRA |= (1 << ADSC);
}

void init_decay() {
	TCCR0 |= (1 << CS01) | (1 << CS00);	// prescaler /64
	TIMSK |= (1 << TOIE0);
}

uint8_t get_pwm_value(uint8_t color, int16_t lev) {
	lev += pgm_read_word(&bias[color]);
	// make sure that level is within bounds
	if (lev < 0) lev = 0;
	if (lev > 0xff) lev = 0xff;
	return pgm_read_byte(&gamma[(uint8_t)lev]);
}

ISR(TIMER0_OVF_vect) {
	static uint16_t decay_overflows = 0;
	uint8_t ax;
	int16_t d;

	// disable interrupts
	cli();

	// do the decay
	if (++decay_overflows >= analog[DECAY_IDX]) {
		for (ax=0; ax<DOF; ax++) {
			if (trail_deflection[ax] > 0) trail_deflection[ax]--;
			else if (trail_deflection[ax] < 0) trail_deflection[ax]++;
		}
		decay_overflows = 0;
	}

	// deflections to levels
	d = 0;
	for (ax=0; ax<DOF; ax++) {
		level[ax] = trail_deflection[ax] + 0x80;	// set zero at 50% power
		d += trail_deflection[ax];
	}
	d = d / DOF;
	// equalize total light amount
	// what is added to one channel gets subtracted from others
	for (ax=0; ax<DOF; ax++) {
		level[ax] -= d;
	}
	RED   = get_pwm_value(0, level[0]);
	GREEN = get_pwm_value(1, level[1]);
	BLUE  = get_pwm_value(2, level[2]);

	sei();
}

int main()
{
	init_lights();
	sei();
	init_sensors();
	// make sure we have some stable measurement
	_delay_ms(100);
	// store current position as rest
	// TODO: update rest on inactivity
	for (int ax=0; ax<DOF; ax++) {
		rest[ax] = analog[ax];
	}
	// now we can initialize the decay timer
	init_decay();
	while (1) {};
}
