
#include <avr/io.h>
#include <util/delay.h>

void set_motor_pins() {
    DDRD |= (1 << PD6) | (1 << PD5); // pwm for ports PD6: 6, PD5: 5
}

void set_duty_cycles(uin8_t duty_cycle_A, uin8_t duty_cycle_B) {
    OCR0A = (255 * duty_cycle_A) / 100; // duty cycle A
    OCR0B = (255 * duty_cycle_B) / 100; // duty cycle B
}

void timer_motor_init() {
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); // fast pwm with toggle on compare match and non-inverting mode
    TCCR0B &= ~(1 << WGM02); 
    TCCR0B |= (1 << CS01); // prescaler = 8
}


void move_forward(uint8_t duty_cycle) {
    set_duty_cycle(duty_cycle, duty_cycle)
}


void rotate_right(uint8_t duty_cycle) {
    set_duty_cycles(duty_cycle, 0);
}

void rotate_left(uint8_t duty_cycle) {
    set_duty_cycles(0, duty_cycle);
}

int main() {
    set_motor_pins();
    timer_motor_init();
    while (1)
    {
        move_forward(100);
        _delay_ms(1000);
        rotate_right(100);
        _delay_ms(1000);
        rotate_left(100);
        _delay_ms(1000);
    }
}