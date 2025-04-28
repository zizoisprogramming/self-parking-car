
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>


// Connect a resistor (e.g., 10kΩ) between PD0 and GND physically on the circuit.

#define RX_PIN  PD0
#define TX_PIN  PD1

void initialize_bluetooth() {
    DDRD &= ~(1 << RX_PIN);
    DDRD |= (1 << TX_PIN);
}

void wait_for_bluetooth() {
    while (!(PIND & (1 << RX_PIN)));
}

int main() {
    initialize_bluetooth();
    wait_for_bluetooth();
    while (1) {
    }
}
