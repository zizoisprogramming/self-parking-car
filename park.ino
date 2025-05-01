d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
    d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
    while(d1 >= front || d2 >= front)
    {
      d4 = read_ultrasonic(&PORTC, TRIG4, &PINC, ECHO4);
      d3 = read_ultrasonic(&PORTD, TRIG3, &PIND, ECHO3);
      d1 = read_ultrasonic(&PORTB, TRIG1, &PINB, ECHO1);
      d2 = read_ultrasonic(&PORTB, TRIG2, &PINB, ECHO2);
      if(d3 - d4 > 3)
      {
        rotate_left(slow);
        _delay_ms(slow_time);
        stop();
      }
      else if(d4 - d3 > 3)
      {
        rotate_right(slow);
        _delay_ms(slow_time);
        stop();
      }
      else {
        move_forward(slow);
        _delay_ms(slow_time);
        stop();
      }
    }
    return;
