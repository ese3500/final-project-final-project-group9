#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

char String[25];
void dht11_start(void);
uint8_t dht11_check_response(void);
uint8_t dht11_read_byte(void);

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
    // Set PD2 as output
    DDRD |= (1 << PORTD2);
}

void dht11_start(void)
{
    // Set the pin high for at least 18ms
    PORTD |= (1 << PORTD2);
    _delay_ms(20);

    // Set the pin low for at least 20us
    PORTD &= ~(1 << PORTD2);
    _delay_us(20);
}

uint8_t dht11_check_response(void)
{
    // Wait for the sensor to respond
    _delay_us(40);

    // Check if the sensor is sending a low signal for at least 80us
    if (!(PIND & (1 << PIND2)))
    {
        _delay_us(80);
        sprintf(String, "hit low");
        UART_putstring(String);

        // Check if the sensor has released the line (high signal for at least 80us)
        if (PIND & (1 << PIND2))
        {
            return 1;
        }
    }

    return 0;
}

uint8_t dht11_read_byte(void)
{
    uint8_t byte = 0;

    for (int i = 7; i >= 0; i--)
    {
        // Wait for the sensor to send a low signal (bit start)
        while (!(PIND & (1 << PORTD2)));

        // Wait for the sensor to release the line (bit duration)
        _delay_us(30);

        // If the line is still low, the bit is a 0; otherwise, it's a 1
        if (PIND & (1 << PORTD2))
        {
            byte |= (1 << i);
        }

        // Wait for the bit to end
        while (PIND & (1 << PORTD2));
    }

    return byte;
}

int main(void) {
    while (1)
    {
        dht11_start();
        if (dht11_check_response())
        {
            sprintf(String, "hit check res");
            UART_putstring(String);
            uint8_t hum_int = dht11_read_byte();
            uint8_t hum_dec = dht11_read_byte();
            uint8_t temp_int = dht11_read_byte();
            uint8_t temp_dec = dht11_read_byte();
            uint8_t checksum = dht11_read_byte();

            // Checksum verification
            if ((hum_int + hum_dec + temp_int + temp_dec) == checksum)
            {
                // Convert raw data to human-readable format
                float humidity = (float) hum_int + ((float)hum_dec / 10.0);
                float temperature = (float) temp_int + ((float)temp_dec / 10.0);

                // Send data to serial
                sprintf(String, "hit send data");
                UART_putstring(String);

                sprintf(String, "Humidity: %u\n", humidity);
                UART_putstring(String);
                sprintf(String, "Temperature: %u\n", temperature);
                UART_putstring(String);
            }
        }

        // Wait for 2 seconds before taking another measurement
        _delay_ms(2000);
    }
}