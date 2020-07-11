

//timer setup for timer0, timer1, and timer2.
//For arduino uno or any board with ATMEL 328/168.. diecimila, duemilanove, lilypad, nano, mini...

//this code will enable all three arduino timer interrupts.
//timer0 will interrupt at 2kHz
//timer1 will interrupt at 1Hz
//timer2 will interrupt at 8kHz

//storage variables
boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;

void setup(){
  
  //set pins as outputs
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

cli();//stop interrupts

//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

//set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


sei();//allow interrupts

}//end setup

ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle0){
    digitalWrite(8,HIGH);
    toggle0 = 0;
  }
  else{
    digitalWrite(8,LOW);
    toggle0 = 1;
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    digitalWrite(13,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(13,LOW);
    toggle1 = 1;
  }
}
  
ISR(TIMER2_COMPA_vect){//timer1 interrupt 8kHz toggles pin 9
//generates pulse wave of frequency 8kHz/2 = 4kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle2){
    digitalWrite(9,HIGH);
    toggle2 = 0;
  }
  else{
    digitalWrite(9,LOW);
    toggle2 = 1;
  }
}


void loop(){
  //do other things here
}

//
////DIGITAL PINS
//
//const int digitalPin0 = 0 //RX
//const int digitalPin1 = 1 //TX
//const int digitalPin2 = 2 //INT0
//const int digitalPin3 = 3 //INT1, PWM
//const int digitalPin4 = 4 //
//const int digitalPin5 = 5 //PWM
//const int digitalPin6 = 6 //PWM
//const int digitalPin7 = 7 //
//const int digitalPin8 = 8 //CLK0
//const int digitalPin9 = 9 //PWM
//const int digitalPin10 = 10 //slave select(SS), PWM
//const int digitalPin11 = 11 //MOSI, PWM
//const int digitalPin12 = 12 //MISO
//const int digitalPin13 = 13 //SCK(Serial Clock), LED
////ANALOG PINS
//
//const int analogPinA0 = A0 //
//const int analogPinA1 = A1 //
//const int analogPinA2 = A2 //
//const int analogPinA3 = A3 //
//const int analogPinA4 = A4 //SDA, interior
//const int analogPinA5 = A5 //SCL, interior
//const int analogPinA6 = A6 //Analog exclusive, end of board
//const int analogPinA7 = A7 //Analog exclusive, end of board
