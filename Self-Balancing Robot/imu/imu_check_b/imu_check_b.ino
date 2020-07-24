#include<Wire.h>

const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double roll = 0.0;
float alpha = 0.9; 
float dt= 0.01; 
float GxConversionFactor = 245 / (65535/2); 
int angleest = 0;

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

int xGyroArray[arrayLength] = {0,0,0,0,0};
char xGyroIndex = 0;
int Gx = 0;

long sumArray(int a[], int);
void angleEst(void);

void setup(){

Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B); // PWR_MGMT_1 register
Wire.write(0); // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);
Serial.begin(9600);

}
void loop(){
angleEst();
Serial.print("angleest\t"); Serial.println(angleest);

delay(100);
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

void angleEst(void)
{
  Wire.beginTransmission(MPU_addr);
Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
//Serial.print("AcX = "); Serial.print(AcX);
//Serial.print(" | AcY = "); Serial.print(AcY);
//Serial.print(" | AcZ = "); Serial.print(AcZ);
//Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
//Serial.print(" | GyX = "); Serial.print(GyX);
//Serial.print(" | GyY = "); Serial.print(GyY);
//Serial.print(" | GyZ = "); Serial.println(GyZ);

xAccelArray[xAccelIndex] = AcX;
xAccelIndex = (xAccelIndex+1)%arrayLength;

yAccelArray[yAccelIndex] = AcY;
yAccelIndex = (yAccelIndex+1)%arrayLength;

zAccelArray[zAccelIndex] = AcZ;
zAccelIndex = (zAccelIndex+1)%arrayLength;

xGyroArray[xGyroIndex] = GyX;
xGyroIndex = (xGyroIndex+1)%arrayLength;

// Estimate Angle
Ax = sumArray(xAccelArray, arrayLength)/arrayLength;
Ay = sumArray(yAccelArray, arrayLength)/arrayLength;
Az = sumArray(zAccelArray, arrayLength)/arrayLength;

Gx = sumArray(xGyroArray,arrayLength)/arrayLength;

roll = atan2(((double) Ax), (double) sqrt(Ay*Ay + Az*Az))*57.3;
GyX = Gx*GxConversionFactor;
angleest = round(alpha*(angleest + dt*GyX) + (1-alpha)*roll); 
}
