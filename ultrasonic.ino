#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define TRIG1 PD2
#define ECHO1 PD3

#define TRIG2 PD4
#define ECHO2 PD5

#define TRIG3 PB2
#define ECHO3 PB3

#define TRIG4 PB0
#define ECHO4 PB1

uint16_t d1, d2, d3, d4;

void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << TXEN0); // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void send_trigger(volatile uint8_t *port, uint8_t pin) {
    *port |= (1 << pin);
    _delay_us(10);
    *port &= ~(1 << pin);
}

uint16_t measure_echo(volatile uint8_t *pin_reg, uint8_t pin) {
    uint16_t timer = 0;

    // Wait for echo high
    while (!(*pin_reg & (1 << pin)));
    // Count while echo is high
    while (*pin_reg & (1 << pin)) {
        _delay_us(1);
        timer++;
        if (timer > 30000) break; // Timeout safety
    }

    return timer;
}

void initialize_ultrasonic()
{
   // Set trigger pins as output
    DDRD |= (1 << TRIG1) | (1 << TRIG2);
    DDRB |= (1 << TRIG4) | (1 << TRIG3);

    // Set echo pins as input
    DDRD &= ~((1 << ECHO1) | (1 << ECHO2));
    DDRB &= ~((1 << ECHO4) | (1 << ECHO3));

    uart_init(103); // Baud 9600 for 16MHz
}

// Modular function for reading a specific ultrasonic sensor
uint16_t read_ultrasonic(volatile uint8_t *port, uint8_t trig_pin, volatile uint8_t *pin_reg, uint8_t echo_pin) {
    send_trigger(port, trig_pin);
    uint16_t echo_time = measure_echo(pin_reg, echo_pin);
    return (echo_time * 0.0343) / 2;  // Convert time to distance in cm
}

int main(void) {
    char buffer[64];

    // Initialize ultrasonic sensors
    initialize_ultrasonic();

    while (1) {
        uart_print("Measuring distances...\r\n");

        // Read from Sensor 1
        d1 = read_ultrasonic(&PORTD, TRIG1, &PIND, ECHO1);
        uart_print("Sensor 1: ");
        sprintf(buffer, "%u cm\r\n", d1);
        uart_print(buffer);

        // Read from Sensor 2
        d2 = read_ultrasonic(&PORTD, TRIG2, &PIND, ECHO2);
        uart_print("Sensor 2: ");
        sprintf(buffer, "%u cm\r\n", d2);
        uart_print(buffer);

        // Read from Sensor 3
        d3 = read_ultrasonic(&PORTB, TRIG3, &PINB, ECHO3);
        uart_print("Sensor 3: ");
        sprintf(buffer, "%u cm\r\n", d3);
        uart_print(buffer);

        // Read from Sensor 4
        d4 = read_ultrasonic(&PORTB, TRIG4, &PINB, ECHO4);
        uart_print("Sensor 4: ");
        sprintf(buffer, "%u cm\r\n", d4);
        uart_print(buffer);

        // Print all distances
        sprintf(buffer, "S1: %u cm | S2: %u cm | S3: %u cm | S4: %u cm\r\n", d1, d2, d3, d4);
        uart_print(buffer);

        _delay_ms(500);  // Delay for a short while before repeating the measurements
    }

    return 0;
}
