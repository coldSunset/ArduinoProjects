void setup() 
{
pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
Serial.begin(115200);

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
   TCCR1B = _BV(CS10);

  OCR1A = 180;
  OCR1B = 50;
}

void loop() 
{
int  val = analogRead(A0);

  Serial.println((val));
}
