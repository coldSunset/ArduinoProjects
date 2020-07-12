
void setup() 
{
pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  TCCR1A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR1B = _BV(WGM22) | _BV(CS22);
  OCR2A = 180;
  OCR2B = 50;

}

void loop() 
{
   OCR2A = 180;
  delay(1000);
  OCR2A = 90;

}
