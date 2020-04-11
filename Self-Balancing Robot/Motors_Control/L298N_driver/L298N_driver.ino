
const int ENA = 5; // PWM signal on promini (remove jumper) 
const int ENB = 6; // second PWM signal on promini

const int MOT_IN1 = 4; //digital pins for direction control 
const int MOT_IN2 = 7;

const int MOT_IN3 = 2;
const int MOT_IN4 = 3;

int MOT_SPEED1 = 255; // speed of motors 
int MOT_SPEED2 = 255; 

//structure defintions for motor 

struct Motor
{
  const int enablePin; 
  const int inputPin1; 
  const int inputPin2; 
  int motorSpeed;      

}; 

//Function prototypes
void setup_Motor(struct Motor thisMotor);
void brakeMotor(struct Motor thisMotor); 
void goForward(struct Motor thisMotor); 
void goBackward(struct Motor thisMotor); 

//strcuture initialisations 
struct Motor motorA = {ENA, MOT_IN1, MOT_IN2, MOT_SPEED1}; 
struct Motor motorB = {ENB, MOT_IN3, MOT_IN4, MOT_SPEED2}; 
  
void setup()
{

  setup_Motor(motorA); 
  setup_Motor(motorB); 


}

void loop() 
{



}

//

void setup_Motor(struct Motor thisMotor) // sets up all motors in brake position
{
  pinMode(thisMotor.enablePin, OUTPUT); // set PWM signal to output 
  pinMode(thisMotor.inputPin1, OUTPUT); 
  pinMode(thisMotor.inputPin2, OUTPUT); 
  digitalWrite(thisMotor.inputPin1, LOW); 
  digitalWrite(thisMotor.inputPin2, LOW); 
  analogWrite(thisMotor.motorSpeed, 0); 

}

void goForward(struct Motor thisMotor)
{
  analogWrite(thisMotor.motorSpeed, 255); 
  digitalWrite(thisMotor.inputPin1, HIGH); 
  digitalWrite(thisMotor.inputPin2, LOW); 
}

void goBackward(struct Motor thisMotor)
{
  analogWrite(thisMotor.motorSpeed, 255); 
  digitalWrite(thisMotor.inputPin1, LOW); 
  digitalWrite(thisMotor.inputPin2, HIGH);  
}

void brakeMotor(struct Motor thisMotor)
{
  analogWrite(thisMotor.motorSpeed, 0); 
  digitalWrite(thisMotor.inputPin1, LOW); 
  digitalWrite(thisMotor.inputPin2, LOW);  
}
