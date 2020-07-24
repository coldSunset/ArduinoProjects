// 23/07/20
// Timer interrupt on timer 2, 100 Hz 

int pin = 13;
boolean toggle2 = 0;

void setup() {

cli();//stop interrupts

pinMode(11,OUTPUT); 
//set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 100 Hz increments
  OCR2A = 155;// = (16*10^6) / (8000*1024) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= (1 << CS21)|(1 << CS20)|(1 << CS22);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


sei();//allow interrupts
}

ISR(TIMER2_COMPA_vect){//timer1 interrupt 8kHz toggles pin 9
//generates pulse wave of frequency 8kHz/2 = 4kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle2){
    digitalWrite(pin,HIGH);
    toggle2 = 0;
  }
  else{
    digitalWrite(pin,LOW);
    toggle2 = 1;
  }
}
void loop() {
  // put your main code here, to run repeatedly:
//digitalWrite(11,HIGH);
}
