#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define SERVO_PIN PB1
#define SERVO_MIN 1000
#define SERVO_MAX 2000

uint16_t pos = 1400;

void Initialize() {
    cli();

    // ADC Setup
    DDRC &= ~(1<<DDC0); // Set AC0 pin

    // Clear power reduction for ADC
    PRR &= ~(1<<PRADC);

    // Select Vref = AVcc
    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1);

    // Set ADC clock to have prescaler of 128 (125kHz where ADC needs 50 - 200 kHz)
    ADCSRA |= (1<<ADPS0);
    ADCSRA |= (1<<ADPS1);
    ADCSRA |= (1<<ADPS2);

    // Select channel 0
    ADMUX &= ~(1<<MUX0);
    ADMUX &= ~(1<<MUX1);
    ADMUX &= ~(1<<MUX2);
    ADMUX &= ~(1<<MUX3);

    // Set to auto trigger
    ADCSRA |= (1<<ADATE);

    // Set to free running
    ADCSRB &= ~(1<<ADTS0);
    ADCSRB &= ~(1<<ADTS1);
    ADCSRB &= ~(1<<ADTS2);

    // Disable digital input buffer on ADC pin
    DIDR0 |= (1<<ADC0D);

    // Enable ADC
    ADCSRA |= (1<<ADEN);

    // Enable ADC interrupt
    ADCSRA |= (1<<ADIE);

    // Start conversion
    ADCSRA |= (1<<ADSC);

    sei();
}

void servo_init() {
    // Set the servo pin as an output
    DDRB |= (1 << PINB1);

    // Set Timer/Counter1 to Fast PWM mode, 8-bit resolution, and non-inverted mode
    TCCR1A |= (1 << COM1A1) | (1 << WGM10);
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);

    // Set the ICR1 register to the top value for 50Hz PWM (20000us period)
    ICR1 = 1250;

}

void servo_write_us(uint16_t us) {
    // Calculate the compare match value from the us input
    uint16_t cmp_match = (us * 2) / 5;

    // Write the compare match value to OCR1A
    OCR1A = cmp_match;
}

void getPosition() {
    if (ADC >= 600 && pos != 1000) {
        servo_write_us(950);
        _delay_us(200);
        pos = 1000;
    } else if (ADC < 600 && pos != 200) {
        servo_write_us(200);
        _delay_us(200);
        pos = 200;
    }
}

int main() {
    Initialize();

    // Initialize the servo
    servo_init();

    // Move the servo back and forth
    while (1) {
        getPosition();
    }
}
