#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Connect a resistor (e.g., 10kΩ) between PD0 and GND physically on the circuit.

#define MOTOR_A PD5
#define MOTOR_B PD6 

#define LEFT_DIR_PIN_1 PD4 
#define LEFT_DIR_PIN_2 PD7 
#define RIGHT_DIR_PIN_1 PB5 
#define RIGHT_DIR_PIN_2 PB4 

#define RX_PIN  PD0
#define TX_PIN  PD1

#define TRIG1 PB0
#define ECHO1 PB1

#define TRIG2 PB2  
#define ECHO2 PB3  

#define TRIG3 PD2  
#define ECHO3 PD3  

#define TRIG4 PC4  
#define ECHO4 PC5  

#define TRIG5 PC2
#define ECHO5 PC3

#define TRIG6 PC0
#define ECHO6 PC1
uint16_t d1, d2, d3, d4, d5, d6;

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

void initialize_bluetooth() {
    DDRD &= ~(1 << RX_PIN);
    DDRD |= (1 << TX_PIN);
}

void wait_for_bluetooth() {
    while (!(PIND & (1 << RX_PIN)));
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
    DDRD |= (1 << TRIG3);
    DDRC |= (1 << TRIG4) | (1 << TRIG5) | (1 << TRIG6);  // PC0-PC4 as outputs
    DDRB |= (1 << TRIG1) | (1 << TRIG2);

    // Set echo pins as input
    DDRD &= ~(1 << ECHO3);
    DDRC &= ~((1 << ECHO4) | (1 << ECHO5) | (1 << ECHO6));  // PC1-PC5 as inputs
    DDRB &= ~((1 << ECHO1) | (1 << ECHO2));

    uart_init(103); 
}

uint16_t read_ultrasonic(volatile uint8_t *port, uint8_t trig_pin, volatile uint8_t *pin_reg, uint8_t echo_pin) {
    send_trigger(port, trig_pin);
    uint16_t echo_time = measure_echo(pin_reg, echo_pin);
    return (echo_time * 0.0343) / 2;  // Convert time to distance in cm
}


void set_motor_pins() {
    DDRD |= (1 << MOTOR_A) | (1 << MOTOR_B); // pwm for ports PD6: 6, PD5: 5
    DDRD |= (1 << LEFT_DIR_PIN_1) | (1 << LEFT_DIR_PIN_2); // direction for ports PD4: 4, PD7: 7 (just for H-bridge)
    DDRB |= (1 << RIGHT_DIR_PIN_1) | (1 << RIGHT_DIR_PIN_2); // direction for ports PB0: 8, PB4: 12
}

void set_duty_cycles(uint8_t duty_cycle_A, uint8_t duty_cycle_B) {
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
        PORTD |= (1 << LEFT_DIR_PIN_1);   
        PORTD &= ~(1 << LEFT_DIR_PIN_2);  
    } 
    else if (left_dir == 2) {  // Backward
        PORTD &= ~(1 << LEFT_DIR_PIN_1);  
        PORTD |= (1 << LEFT_DIR_PIN_2);   
    } 
    else {  // stop
        PORTD &= ~((1 << LEFT_DIR_PIN_1) | (1 << LEFT_DIR_PIN_2));  
    }

    if (right_dir == 1) {  // Forward
        PORTB |= (1 << RIGHT_DIR_PIN_1);   
        PORTB &= ~(1 << RIGHT_DIR_PIN_2);  
    } 
    else if (right_dir == 2) {  // Backward
        PORTB &= ~(1 << RIGHT_DIR_PIN_1);  
        PORTB |= (1 << RIGHT_DIR_PIN_2);   
    } 
    else {  // stop
        PORTB &= ~((1 << RIGHT_DIR_PIN_1) | (1 << RIGHT_DIR_PIN_2));  
    }
}


void move_forward(uint8_t duty_cycle) {
    set_motor_direction(2, 2);
    set_duty_cycles(duty_cycle, duty_cycle);
}

void move_backward(uint8_t duty_cycle) {
    set_motor_direction(1, 1);
    set_duty_cycles(duty_cycle, duty_cycle);
}
void rotate_left(uint8_t duty_cycle) {
    set_motor_direction(1, 2);
    set_duty_cycles(duty_cycle, duty_cycle);
}

void rotate_right(uint8_t duty_cycle) {
    set_motor_direction(2, 1);
    set_duty_cycles(duty_cycle, duty_cycle);
}

void stop() {
    set_motor_direction(0, 0);
    set_duty_cycles(0, 0);
}
const int space = 18;
const int slow = 80;
const int front = 6;
const int front_other = 6;

const int radius = 5;
const int slow_time = 25;
const int thresh_rotate = 25;
const int thresh_forward = 10;
const int shift_dir = 2; // for going forward feedback
const int count_rotations = 40;
void park2(int direction)
{
    char buffer[64];
    sprintf(buffer, "parking2 %u \n", direction);
    uart_print(buffer);
  
  if(direction == 1) //right shit
  {
    int d3,d1,d2, d4; // d3 right d1,d2 front
    d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      sprintf(buffer, "before d3 %u", d3);
            uart_print(buffer);
      sprintf(buffer, "before d4 %u", d4);
            uart_print(buffer);

    int count = 0;
    while(count++ <=count_rotations)
    {
      sprintf(buffer, "d3 %u", d3);
      uart_print(buffer);
      rotate_right(slow);
      _delay_ms(slow_time);
      stop();
      d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    }
          sprintf(buffer, "after d3 %u", d3);
            uart_print(buffer);
      sprintf(buffer, "after d4 %u", d4);
            uart_print(buffer);
    d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
    d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
    d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    while(d1 >= front || d2 >= front_other)
    {
        sprintf(buffer, "after d1 %u \n", d1);
        uart_print(buffer);
        sprintf(buffer, "after d2 %u \n", d2);
        uart_print(buffer);
      if(d1 <= shift_dir && d1 <= d2)
      {
        move_backward(slow);
        _delay_ms(slow_time);
        rotate_right(slow);
        _delay_ms(slow_time);
        stop();
      }
      else if (d2 <= shift_dir) {
        move_backward(slow);
        _delay_ms(slow_time);
        rotate_left(slow);
        _delay_ms(slow_time);
        stop();
      }
      else {
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
      }
      d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
      d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
      d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    }
    return;
   
  }
  else { 
    int d5,d1,d2, d6; // d3 right d1,d2 front
    d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
      sprintf(buffer, "before d5 %u", d5);
            uart_print(buffer);
      sprintf(buffer, "before d6 %u", d6);
            uart_print(buffer);
    int count = 0;
    while(count++ <=count_rotations)
    {
      sprintf(buffer, "d3 %u", d3);
      uart_print(buffer);
      rotate_left(slow);
      _delay_ms(slow_time);
      stop();
      d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    }
    // while(d5 >= thresh_rotate && d6 >= thresh_rotate)
    // {
    //   sprintf(buffer, "d5 %u /n", d5);
    //   uart_print(buffer);
    //         sprintf(buffer, "d6 %u /n", d6);
    //   uart_print(buffer);
    //   rotate_left(slow);
    //   _delay_ms(slow_time);
    //   stop();
    // d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    // d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
    // }
    // while(d5 <= thresh_rotate || d6 <= thresh_rotate)
    // {
    //   sprintf(buffer, "d5 %u /n", d5);
    //   uart_print(buffer);
    //         sprintf(buffer, "d6 %u /n", d6);
    //   uart_print(buffer);
    //   rotate_left(slow);
    //   _delay_ms(slow_time);
    //   stop();
    //   d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    //   d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
    // }
    
          sprintf(buffer, "after d5 %u", d6);
            uart_print(buffer);
      sprintf(buffer, "after d6 %u", d6);
            uart_print(buffer);

    d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
    d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
    while(d1 >= front_other || d2 >= front)
    {
        sprintf(buffer, "after d1 %u \n", d1);
        uart_print(buffer);
        sprintf(buffer, "after d2 %u \n", d2);
        uart_print(buffer);
      if(d1 <= shift_dir && d1 <= d2)
      {
        move_backward(slow);
        _delay_ms(slow_time);
        rotate_right(slow);
        _delay_ms(slow_time);
        stop();
      }
      else if (d2 <= shift_dir) {
        move_backward(slow);
        _delay_ms(slow_time);
        rotate_left(slow);
        _delay_ms(slow_time);
        stop();
      }
      else {
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
      }
      d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
      d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
    }
    return;
  }
}
void park(int direction)
{
      char buffer[64];

    sprintf(buffer, "parking %u \n", direction);
      uart_print(buffer);

  if(direction == 1) //right shit
  {
    int d3,d1,d2, d4; // d3 right d1,d2 front
    d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);

    while(d4 >= space)
    {
      sprintf(buffer, "d3 %u", d3);
      uart_print(buffer);
      move_backward(slow - 10);
      _delay_ms(slow_time);
      stop();
      d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    }
    d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
    d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
    d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
    while(d1 >= front || d2 >= front)
    {
      while(d3 >= radius)
      {
        rotate_right(slow);
        _delay_ms(slow_time);
        stop();
        sprintf(buffer, "d3_1 %u \n", d3);
        uart_print(buffer);
        d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
      }
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
        d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
        d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
        d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
        d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
        sprintf(buffer, "d3_2 %u \n", d3);
        uart_print(buffer);
    }
    return;
  }
  else {  // left shit
    int d6,d1,d2, d5; // d6 left d1,d2 front
    d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
    while(d5 >= space)  // elly wara
    {
      sprintf(buffer, "d6 %u", d6);
      uart_print(buffer);
      move_backward(slow);
      _delay_ms(slow_time);
      stop();
      d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
      d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
    }
    d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
    d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    while(d1 >= front || d2 >= front)
    {
      while(d6 >= radius)
      {
        rotate_left(slow);
        _delay_ms(slow_time);
        stop();
        d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
      }
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
        d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
        d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
        d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
        sprintf(buffer, "\n d1 %u", d1);
        sprintf(buffer, "\n d2 %u", d2);

      uart_print(buffer);
    }
    return;
  }
}
int main(void) {
    char buffer[64];
    // Initialize ultrasonic sensors
    initialize_ultrasonic();
    // Initialize motors
    set_motor_pins();
    timer_motor_init();
    initialize_bluetooth();
    wait_for_bluetooth();
    while (1) {
        uart_print("Measuring distances...\r\n");

        //  Read from Sensor 1
        d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
        uart_print("Sensor 1: ");
        sprintf(buffer, "%u cm\r\n", d1);
        uart_print(buffer);

        // Read from Sensor 2
        d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
        uart_print("Sensor 2: ");
        sprintf(buffer, "%u cm\r\n", d2);
        uart_print(buffer);

        // Read from Sensor 3
        d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
        uart_print("Sensor 3: ");
        sprintf(buffer, "%u cm\r\n", d3);
        uart_print(buffer);

        // Read from Sensor 4
        d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
        uart_print("Sensor 4: ");
        sprintf(buffer, "%u cm\r\n", d4);
        uart_print(buffer);

        // Read from Sensor 5
        d5 = read_ultrasonic(&PORTC, TRIG5, &PINC, ECHO5);
        uart_print("Sensor 5: ");
        sprintf(buffer, "%u cm\r\n", d5);
        uart_print(buffer);

        // Read from Sensor 6
        d6 = read_ultrasonic(&PORTC, TRIG6, &PINC, ECHO6);
        uart_print("Sensor 6: ");
        sprintf(buffer, "%u cm\r\n", d6);
        uart_print(buffer);

        // Print all distances
        sprintf(buffer, "S1: %u | S2: %u | S3: %u | S4: %u | S5: %u | S6: %u cm\r\n", 
               d1, d2, d3, d4, d5, d6);
        uart_print(buffer);
        // _delay_ms(1000);

        if(d3 >= space && d4 >= space)  //right
        {
          park2(1);
          break;
        }
        if(d5 >= space && d6 >= space)
        {
          park2(0);
          break;
        }
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
        }

    return 0;
}
