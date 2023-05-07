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

//char String[25];
uint16_t pos = 1400;

///*-------------------UART Functions----------------------*/
//void UART_init(void) {
//    /* Set Baud Rate*/
//    UBRR0H = (unsigned char) (BAUD_PRESCALER>>8);
//    UBRR0L = (unsigned char) BAUD_PRESCALER;
//    // Enable receiver and transmitter
//    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
//
//   /* Set Frame Format*/
//    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8 data bits
//    UCSR0C |= (1<<USBS0); // 2 stop bits
//}
//
//void UART_send(unsigned char data) {
//    // Wait for empty transmit buffer
//    while (!(UCSR0A & (1<<UDRE0)));
//    // Put data into buffer and send data
//    UDR0 = data;
//}
//
//void UART_putstring(char* StringPtr) {
//    while(*StringPtr != 0x00) {
//        UART_send(*StringPtr);
//        StringPtr++;
//    }
//}

/*-----------------------------------------*/

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
//    OCR1B = OCR1A * 20 / 100;
}

void getPosition() {
//    uint16_t pos;
    if (ADC >= 600 && pos != 1000) {
//        _delay_us(200);
        servo_write_us(950);
        _delay_us(200);
        pos = 1000;
//    } else if (ADC >= 333 && ADC <= 666) {
//        pos = 1500;
    } else if (ADC < 600 && pos != 200) {
//        _delay_us(200);
        servo_write_us(200);
        _delay_us(200);
        pos = 200;
    }
//    _delay_ms(200);
//    sprintf(String, "ADC: %u\n", ADC);
//    UART_putstring(String);
//    sprintf(String, "Position: %d\n", pos);
//    UART_putstring(String);


}

int main() {
    Initialize();
//    UART_init();

    // Initialize the servo
    servo_init();
//    servo_write_us(1000);

//    sprintf(String, "hello");
//    UART_putstring(String);

    // Move the servo back and forth
    while (1) {
//        _delay_ms(1000);
        getPosition();
//        getPosition();
//        sprintf(String, "hello \n");
//        UART_putstring(String);
//        _delay_ms(1000);

    }
}
