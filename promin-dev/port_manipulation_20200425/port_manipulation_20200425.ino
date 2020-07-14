
void setup() 
{
pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
//  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
//   TCCR1B = _BV(CS10);
//
//  OCR1A = 180;
//  OCR1B = 50;
  analogWrite(9,255);
//  analogWrite(10,180);
//Serial.begin(115200);
}

void loop() 
{
//Serial.println((OCR1A));

}
