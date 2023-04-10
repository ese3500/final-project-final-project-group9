//#include <Arduino.h>
//#include <avr/io.h>
//#include <util/delay.h>
//#define F_CPU 16000000UL
//#define BAUD_RATE 9600
//#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
//
//char String[25];
//void dht11_start(void);
//uint8_t dht11_check_response(void);
//uint8_t dht11_read_byte(void);
//
/*-------------------UART Functions----------------------*/
//void UART_init(void) {
//    /* Set Baud Rate*/
//    UBRR0H = (unsigned char) (BAUD_PRESCALER>>8);
//    UBRR0L = (unsigned char) BAUD_PRESCALER;
//    // Enable receiver and transmitter
//    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
//
//    /* Set Frame Format*/
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
//
/*-----------------------------------------*/
//
//void Initialize() {
//    // Set PD2 as output
//    DDRD |= (1 << PORTD2);
//}
//
//void dht11_start(void)
//{
//    // Set the pin high for at least 18ms
//    PORTD |= (1 << PORTD2);
//    _delay_ms(20);
//
//    // Set the pin low for at least 20us
//    PORTD &= ~(1 << PORTD2);
//    _delay_us(20);
//}
//
//uint8_t dht11_check_response(void)
//{
//    // Wait for the sensor to respond
//    _delay_us(40);
//
//    // Check if the sensor is sending a low signal for at least 80us
//    if (!(PIND & (1 << PIND2)))
//    {
//        _delay_us(80);
//        sprintf(String, "hit low \n");
//        UART_putstring(String);
//
//        // Check if the sensor has released the line (high signal for at least 80us)
//        if (PIND & (1 << PIND2))
//        {
//            sprintf(String, "hit high \n");
//            UART_putstring(String);
//            return 1;
//        }
//    }
//
//    return 0;
//}
//
//uint8_t dht11_read_byte(void)
//{
//    uint8_t byte = 0;
//
//    for (int i = 7; i >= 0; i--)
//    {
//        // Wait for the sensor to send a low signal (bit start)
//        while (!(PIND & (1 << PORTD2)));
//
//        // Wait for the sensor to release the line (bit duration)
//        _delay_us(30);
//
//        // If the line is still low, the bit is a 0; otherwise, it's a 1
//        if (PIND & (1 << PORTD2))
//        {
//            byte |= (1 << i);
//        }
//
//        // Wait for the bit to end
//        while (PIND & (1 << PORTD2));
//    }
//
//    return byte;
//}
//
//int main(void) {
//    Initialize();
//    UART_init();
//    while (1)
//    {
//        dht11_start();
//        if (dht11_check_response())
//        {
//            sprintf(String, "hit check res");
//            UART_putstring(String);
//            uint8_t hum_int = dht11_read_byte();
//            uint8_t hum_dec = dht11_read_byte();
//            uint8_t temp_int = dht11_read_byte();
//            uint8_t temp_dec = dht11_read_byte();
//            uint8_t checksum = dht11_read_byte();
//
//            // Checksum verification
//            if ((hum_int + hum_dec + temp_int + temp_dec) == checksum)
//            {
//                // Convert raw data to human-readable format
//                float humidity = (float) hum_int + ((float)hum_dec / 10.0);
//                float temperature = (float) temp_int + ((float)temp_dec / 10.0);
//
//                // Send data to serial
//                sprintf(String, "hit send data");
//                UART_putstring(String);
//
//                sprintf(String, "Humidity: %u\n", humidity);
//                UART_putstring(String);
//                sprintf(String, "Temperature: %u\n", temperature);
//                UART_putstring(String);
//            }
//        }
//
//        // Wait for 2 seconds before taking another measurement
//        _delay_ms(2000);
//    }
//}
//
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

char String[25];
//uint16_t pos = 0;

/*-------------------UART Functions----------------------*/
void UART_init(void) {
    /* Set Baud Rate*/
    UBRR0H = (unsigned char) (BAUD_PRESCALER>>8);
    UBRR0L = (unsigned char) BAUD_PRESCALER;
    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);

   /* Set Frame Format*/
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8 data bits
    UCSR0C |= (1<<USBS0); // 2 stop bits
}

void UART_send(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1<<UDRE0)));
    // Put data into buffer and send data
    UDR0 = data;
}

void UART_putstring(char* StringPtr) {
    while(*StringPtr != 0x00) {
        UART_send(*StringPtr);
        StringPtr++;
    }
}

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

uint16_t getPosition() {
    uint16_t pos;
    if (ADC >= 1000) {
        pos = 2000;
//    } else if (ADC >= 333 && ADC <= 666) {
//        pos = 1500;
    } else {
        pos = ADC + 1000;
    }

    return  pos;
//    _delay_ms(200);
//    sprintf(String, "ADC: %u\n", ADC);
//    UART_putstring(String);
//    sprintf(String, "Position: %d\n", pos);
//    UART_putstring(String);


}

int main() {
    Initialize();
    UART_init();

    // Initialize the servo
    servo_init();

//    sprintf(String, "hello");
//    UART_putstring(String);

    // Move the servo back and forth
    while (1) {
//        _delay_ms(1000);
//        getPosition();
        servo_write_us(getPosition());
//        sprintf(String, "hello \n");
//        UART_putstring(String);
        _delay_ms(1000);

    }
}
