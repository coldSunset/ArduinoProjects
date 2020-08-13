#include "motor.h" 






//MOTOR FUNCTIONS
void setup_Motor(struct Motor thisMotor) // sets up all motors in brake position
{
  pinMode(thisMotor.enablePin, OUTPUT); // set PWM signal to output
  pinMode(thisMotor.inputPin1, OUTPUT);
  pinMode(thisMotor.inputPin2, OUTPUT);
  digitalWrite(thisMotor.inputPin1, LOW);
  digitalWrite(thisMotor.inputPin2, LOW);
  analogWrite(thisMotor.enablePin, 0);

}

void goForward(struct Motor thisMotor)
{
  analogWrite(thisMotor.enablePin, thisMotor.motorSpeed);
  digitalWrite(thisMotor.inputPin1, HIGH);
  digitalWrite(thisMotor.inputPin2, LOW);
}

void goBackward(struct Motor thisMotor)
{
  analogWrite(thisMotor.enablePin, thisMotor.motorSpeed);
  digitalWrite(thisMotor.inputPin1, LOW);
  digitalWrite(thisMotor.inputPin2, HIGH);
}

void brakeMotor(struct Motor thisMotor)
{
  analogWrite(thisMotor.enablePin, 0);
  digitalWrite(thisMotor.inputPin1, LOW);
  digitalWrite(thisMotor.inputPin2, LOW);
}