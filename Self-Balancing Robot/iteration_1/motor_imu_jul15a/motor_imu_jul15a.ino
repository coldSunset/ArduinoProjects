
// Most Recent Update = 17072020 

//.... HEADER FILES ...//
#include "I2Cdev.h" 
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define BAUD_RATE 9600
//........ IMU Definitions........//
MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z] 
VectorFloat gravity;    // [x, y, z]            gravity vector

//........ INTERUPT DETECTION ROUTINE.....//
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//..............ANGLE CALCULATION VARS........//
const int arrayLength = 5;
int xAccelArray[arrayLength] = {0,0,0,0,0};
char xAccelIndex = 0;
long Ax = 0;
int yAccelArray[arrayLength] = {0,0,0,0,0};
char yAccelIndex = 0;
long Ay = 0;
int zAccelArray[arrayLength] = {0,0,0,0,0};
char zAccelIndex = 0;
long Az = 0;

double roll = 0.0; 

int xGyroArray[arrayLength] = {0,0,0,0,0};
char xGyroIndex = 0;
int Gx = 0;
float GxConversionFactor = 245 / (65535/2); // Gyro is +-245 dps for signed 16 bit number, converts 16 bit to dps value

float alpha = 0.9;
float dt = 0.01;    // for 100 Hz sensing

int angleEst = 0;

long sumArray(int a[], int);

//........ MOTOR CONTROL DEFINITIONS......//
const int ENA = 9; // PWM signal on promini (remove jumper) 
const int ENB = 10; // second PWM signal on promini
const int MOT_IN1 = 3; //digital pins for direction control 
const int MOT_IN2 = 4;
const int MOT_IN3 = 5;
const int MOT_IN4 = 6;
int MOT_SPEED1 = 0; // speed of motors 
int MOT_SPEED2 = 0; 

//structure defintions for motor 
struct Motor
{
  const int enablePin; 
  const int inputPin1; 
  const int inputPin2; 
  int motorSpeed;      
}; 

//Motor Function prototypes
void setup_Motor(struct Motor thisMotor);
void brakeMotor(struct Motor thisMotor); 
void goForward(struct Motor thisMotor); 
void goBackward(struct Motor thisMotor); 

//Motor structure initialisations 
struct Motor motorA = {ENA, MOT_IN1, MOT_IN2, MOT_SPEED1}; 
struct Motor motorB = {ENB, MOT_IN3, MOT_IN4, MOT_SPEED2}; 
  
void setup()
{
  //Motors
  setup_Motor(motorA); 
  setup_Motor(motorB); 

  // IMU Sensor
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(BAUD_RATE);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
      // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            //Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
           // Serial.print("\t");
           // Serial.println(aaReal.z);
          xAccelArray[xAccelIndex] = aaReal.x;
          xAccelIndex = (xAccelIndex+1)%arrayLength;

          yAccelArray[yAccelIndex] = aaReal.y;
          yAccelIndex = (yAccelIndex+1)%arrayLength;

          zAccelArray[zAccelIndex] = aaReal.z;
          zAccelIndex = (zAccelIndex+1)%arrayLength;

          xGyroArray[xGyroIndex] = yGyro;
          xGyroIndex = (xGyroIndex+1)%arrayLength;

          // Estimate Angle
          Ax = sumArray(xAccelArray, arrayLength)/arrayLength;
          Ay = sumArray(yAccelArray, arrayLength)/arrayLength;
          Az = sumArray(zAccelArray, arrayLength)/arrayLength;

          Gx = sumArray(xGyroArray,arrayLength)/arrayLength;
          Gx = Gx * GxConversionFactor;

          roll = atan2(((double) Ax), (double) sqrt(Ay*Ay + Az*Az)) * 57.3;     // 180/pi = 57.3 (for conversion to degrees)

          angleEst = round(alpha*(angleEst + dt*Gx) + (1-alpha)*roll);     // angle estimation equation in degrees

          Serial.print("angle\t");
          Serial.print("\t");
          Serial.print(angleEst);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


long sumArray(int a[], int num_elements)
{
   int i;
   long sum=0;
   for (i=0; i<num_elements; i++)
   {
     sum = sum + a[i];
   }
   return(sum);
}
//........... MOTOR FUNCTIONS ............//

void setup_Motor(struct Motor thisMotor) // sets up all motors in brake position
{
 // pinMode(thisMotor.enablePin, OUTPUT); // set PWM signal to output 
  pinMode(thisMotor.inputPin1, OUTPUT); 
  pinMode(thisMotor.inputPin2, OUTPUT); 
  digitalWrite(thisMotor.inputPin1, LOW); 
  digitalWrite(thisMotor.inputPin2, LOW); 
  analogWrite(thisMotor.enablePin, 127); 

}

void goForward(struct Motor thisMotor)
{
  analogWrite(thisMotor.motorSpeed, 127); 
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
