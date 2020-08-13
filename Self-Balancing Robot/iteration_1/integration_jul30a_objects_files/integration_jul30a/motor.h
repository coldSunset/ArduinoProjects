#ifndef MOTOR_h
#define MOTOR_h

// MOTOR PIN ASSIGNMENTS
const int ENA = 9; // PWM signal on promini (remove jumper)
const int ENB = 10; // second PWM signal on promini

const int MOT_IN1 = 3; //digital pins for direction control
const int MOT_IN2 = 4;

const int MOT_IN3 = 5;
const int MOT_IN4 = 6;

int MOT_SPEED1 = 0; // speed of motors
int MOT_SPEED2 = 0;

//Function prototypes
void setup_Motor(struct Motor thisMotor);
void brakeMotor(struct Motor thisMotor);
void goForward(struct Motor thisMotor);
void goBackward(struct Motor thisMotor);

//structure defintions for motor
struct Motor
{
  const int enablePin;
  const int inputPin1;
  const int inputPin2;
  int motorSpeed;

};

//strcuture initialisations
struct Motor motorA = {ENA, MOT_IN1, MOT_IN2, MOT_SPEED1};
struct Motor motorB = {ENB, MOT_IN3, MOT_IN4, MOT_SPEED2};

#endif