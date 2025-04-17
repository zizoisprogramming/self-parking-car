
#include <avr/io.h>
#include <util/delay.h>


#define MOTOR_A PD5 // pin 5
#define MOTOR_B PD6 // pin 6

#define LEFT_DIR_PIN_1 PD4 // pin 4
#define LEFT_DIR_PIN_2 PD7 // pin 4
#define RIGHT_DIR_PIN_1 PB0 // pin 8
#define RIGHT_DIR_PIN_2 PB4 // pin 12


void set_motor_pins() {
    DDRD |= (1 << MOTOR_A) | (1 << MOTOR_B); // pwm for ports PD6: 6, PD5: 5
    DDRD |= (1 << LEFT_DIR_PIN_1) | (1 << LEFT_DIR_PIN_2); // direction for ports PD4: 4, PD7: 7 (just for H-bridge)
    DDRB |= (1 << RIGHT_DIR_PIN_1) | (1 << RIGHT_DIR_PIN_2); // direction for ports PB0: 8, PB4: 12
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

void set_motor_direction(uint8_t left_dir, uint8_t right_dir) {
    if (left_dir == 1) {  // Forward
        PORTD |= (1 << LEFT_DIR_PIN1);   
        PORTD &= ~(1 << LEFT_DIR_PIN2);  
    } 
    else if (left_dir == 2) {  // Backward
        PORTD &= ~(1 << LEFT_DIR_PIN1);  
        PORTD |= (1 << LEFT_DIR_PIN2);   
    } 
    else {  // stop
        PORTD &= ~((1 << LEFT_DIR_PIN1) | (1 << LEFT_DIR_PIN2));  
    }

    if (right_dir == 1) {  // Forward
        PORTB |= (1 << RIGHT_DIR_PIN1);   
        PORTB &= ~(1 << RIGHT_DIR_PIN2);  
    } 
    else if (right_dir == 2) {  // Backward
        PORTB &= ~(1 << RIGHT_DIR_PIN1);  
        PORTB |= (1 << RIGHT_DIR_PIN2);   
    } 
    else {  // stop
        PORTB &= ~((1 << RIGHT_DIR_PIN1) | (1 << RIGHT_DIR_PIN2));  
    }
}


void move_forward(uint8_t duty_cycle) {
    set_motor_direction(1, 1);
    set_duty_cycles(duty_cycle, duty_cycle)
}


void rotate_right(uint8_t duty_cycle) {
    set_motor_direction(1, 2);
    set_duty_cycles(duty_cycle, duty_cycle);
}

void rotate_left(uint8_t duty_cycle) {
    set_motor_direction(2, 1);
    set_duty_cycles(duty_cycle, duty_cycle);
}

void stop() {
    set_motor_direction(0, 0);
    set_duty_cycles(0, 0);
}

void main() {
    set_motor_pins();
    timer_motor_init();

    for (int pwm = 100; pwm >= 0; pwm--) {
        int iterations = 5;
        while (iterations--)
        {
            move_forward(pwm);
            _delay_ms(1000);
            rotate_right(pwm);
            _delay_ms(1000);
            rotate_left(pwm);
            _delay_ms(1000);
        }
        stop();
        _delay_ms(1000);
    }

}