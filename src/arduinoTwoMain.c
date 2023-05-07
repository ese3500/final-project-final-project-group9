#include "../lib/uart.h"
#include <stdio.h>
#include <stdlib.h>
#define F_CPU 16000000UL

int prevTemp;
int currTemp;
char String[25];
void adcSetup() {
    // setup for ADC
    // clear power for reduction
    PRR &= ~(1<<PRADC);
    // select Vref = AVcc
    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1);
    // set the ADC clock by 128
    // 16M/128=125kHz
    ADCSRA |= (1<<ADPS0);
    ADCSRA |= (1<<ADPS1);
    ADCSRA |= (1<<ADPS2);
    // select channel 0
    ADMUX &= ~(1<<MUX0);
    ADMUX &= ~(1<<MUX1);
    ADMUX &= ~(1<<MUX2);
    ADMUX &= ~(1<<MUX3);
    // set to auto trigger
    ADCSRA |= (1<<ADATE);
    // set to free running
    ADCSRB &= ~(1<<ADTS0);
    ADCSRB &= ~(1<<ADTS1);
    ADCSRB &= ~(1<<ADTS2);
    // disable digital input buffer on ADC pin
    DIDR0 |= (1<<ADC0D);
    // enable ADC
    ADCSRA |= (1<<ADEN);
    // Enable ADC interrupt
    ADCSRA |= (1<<ADIE);
    // Start conversion
    ADCSRA |= (1<<ADSC);
}

void Init() {
    cli();
    // Initialize beam-breaker pins as inputs
    // Pins: 2, 4, 7, 8
    DDRD &= ~(1<<PD7);
    DDRD &= ~(1<<PD2);
    DDRB &= ~(1<<PB0);
    DDRD &= ~(1<<PD4);

    // Initialize temperature sensor pin @ A0
    DDRC &= ~(1<<PC0);
    adcSetup();

    // Initialize buzzer pin 12 as output
    DDRB |= (1<<PB4);

    // Initialize pin 13 as output to trigger node MCU
    DDRB |= (1<< PB5);

    sei();


}

volatile int adc;

ISR(ADC_vect) {
    adc = ADC;
}

/**
 * Checks if the beam break sensors detect an obstacle
 * @return 1 if beam broken, else 0
 */
int isBeamBroken() {
    // if the pin is low, then it is broken
    sprintf(String, "PIND7: %d ", PIND & (1 << PIND7));
    UART_putstring(String);
    sprintf(String, "PIND2: %d ", PIND & (1 << PIND2));
    UART_putstring(String);
    sprintf(String, "PINB0: %d ", PINB & (1 << PINB0));
    UART_putstring(String);
    sprintf(String, "PIND4: %d ", PINB & (1 << PIND4));
    UART_putstring(String);
    int expression = !((PIND & (1 << PIND7)) && (PIND & (1 << PIND2)) && (PINB & (1 << PINB0)) && (PIND & (1 << PIND4)));
    sprintf(String, "isBroken: %d ", expression);
    UART_putstring(String);
    return expression ;
}


void printTemperature() {
    sprintf(String, "ADC: %d ", adc);
    UART_putstring(String);
    int vOut = (int) (((float) adc * 5000) / 1024);
    int temperatureF = (int) ((float) vOut / 10);
    sprintf(String, "Temp F: %d ", temperatureF);
    UART_putstring(String);
    int temperatureC = (int) ((temperatureF - 32.0)*(5.0/9.0));
    sprintf(String, "Temp C: %d\n\n", temperatureC);
    UART_putstring(String);
    _delay_ms(500);
}

/**
 * Return temperature as Fahreinheits?
 */
int tempF() {
    int vOut = (int) (((float) adc * 5000) / 1024);
    int temperatureF = (int) ((float) vOut / 10);
    return temperatureF;
}

int soundBuzzer(int delay) {
    PORTB |= (1<<PORTB4);
    _delay_ms(delay);
    PORTB &= ~(1<<PORTB4);
}

int triggerNotification(int delay) {
    PORTB |= (1<<PORTB5);
    _delay_ms(delay);
    PORTB &= ~(1<<PORTB5);
}

int main(void) {
    Init();
    UART_init();
    while (1) {
        printTemperature();
        if(isBeamBroken()) {
            soundBuzzer(250);
            triggerNotification(1000);
        }
        currTemp = tempF();
        if (abs(prevTemp - currTemp) > 5) {
            soundBuzzer(250);
            soundBuzzer(250);
            soundBuzzer(250);
        }
        prevTemp = currTemp;
        }
    }
