
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT); 
  analogWrite(6, 55); 
  analogWrite(11, 200); 
  Serial.begin(115200); 
  Serial.println(LED_BUILTIN); 
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(400);                       // wait for a second
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
