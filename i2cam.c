#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "usiTwiSlave.h"

#define TRIG_DDR DDRA
#define TRIG_PORT PORTA
#define TRIG_BIT PA0

#define HALF_DDR DDRA
#define HALF_PORT PORTA
#define HALF_BIT PA1

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_BIT PD5

#define BTN_MS 100

#define SNAPSHOT_INTERVAL 5000
#define SNAPSHOT_FOCUS_MS 100

#define TWI_ADDRESS 0x4c

#define SERVO_MIN 560L
#define SERVO_MAX 2500L

enum btn_state {
	RELEASED,
	HALF,
	PRESSED
};

static volatile struct {
	uint8_t enabled;
	uint16_t interval;
	uint8_t servo_pos[2];
} cam;

static void waylay(uint16_t *time) {
	uint16_t cnt = 0;
	while (cnt++ < *time) _delay_ms(1);
}

void set_btn(enum btn_state s) {
	TRIG_PORT &= ~(1<<TRIG_BIT);
	HALF_PORT &= ~(1<<HALF_BIT);
	switch (s) {
		case PRESSED:
			TRIG_PORT |= 1<<TRIG_BIT;
		case HALF:
			HALF_PORT |= 1<<HALF_BIT;
			break;
		default:
			break;
	}
}

int main(void) {
	TRIG_DDR |= 1<<TRIG_BIT;
	HALF_DDR |= 1<<HALF_BIT;

	LED_DDR |= 1<<LED_BIT;

	DDRB |= (1<<PB4);

	/* configure timer for PWM */
	ICR1 = 20000;
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);

	cam.enabled = 0;
	cam.interval = SNAPSHOT_INTERVAL;

	usiTwiSlaveInit(TWI_ADDRESS);
	usiTwiSetTransmitWindow( &cam, sizeof(cam) );

	sei();

	while (1) {
		if (cam.enabled) {
			LED_PORT |= (1<<LED_BIT);
			waylay(&cam.interval);
			LED_PORT &= ~(1<<LED_BIT);
			set_btn(HALF);
			_delay_ms(SNAPSHOT_FOCUS_MS);
			set_btn(PRESSED);
			_delay_ms(BTN_MS);
			set_btn(RELEASED);
			LED_PORT |= (1<<LED_BIT);
		} else {
			LED_PORT &= ~(1<<LED_BIT);
		}

		OCR1B = SERVO_MIN+((uint16_t)cam.servo_pos*(SERVO_MAX-SERVO_MIN)/255);
	}
}

