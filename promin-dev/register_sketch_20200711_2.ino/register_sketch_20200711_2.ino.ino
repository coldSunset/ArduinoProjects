// 11/07/2020
// DESCRIPTION 
// Sets inbuilt led (13) HIGH and pin 10 HIGH

int extraTime = 0; 

void setup() {

TCCR0A = 0b00000010;    // C1:: timer 0 mode 2 - CTC
TCCR0B = 0b00000100;    // C2:: set prescaler to 256
OCR0A = 250;            // C3:: number of ticks in Output Compare Register
TIMSK0 = 0b00000010;    // C4:: trigger interrupt when ctr (TCNT0) >= OCR0A

DDRB = DDRB | (1<<PORTB5)| (1<<PORTB2); //set 13 and 10 as OUPUTS
sei();

 
}

ISR(TIMER0_COMPA_vect)
{
  extraTime++; 
  if(extraTime >100)
  {
    PORTB ^= (1<<PORTB5) | (1<<PORTB2); //toggle outputs 
    extraTime = 0; 
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
