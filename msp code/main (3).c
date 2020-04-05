// Basic MSP430 and driverLib #includes
#include "msp430.h"
#include "driverlib/MSP430F5xx_6xx/wdt_a.h"
#include "driverlib/MSP430F5xx_6xx/ucs.h"
#include "driverlib/MSP430F5xx_6xx/pmm.h"
#include "driverlib/MSP430F5xx_6xx/sfr.h"

// USB API #includes
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"

#include "USB_app/usbConstructs.h"

// Application #includes
#include "BCUart.h"           // Include the backchannel UART "library"
#include "hal.h"              // Modify hal.h to select your hardware
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

//*********************USB BACKCHANNEL VARS**************************
char buf_usbToBcuart[128]; // This can be any size
//const long clockSpeed = 25000000;   //highest clock speed 25MHz
//*******************************************************************

//*********************I2C VARS**************************************
unsigned char RX_Data[12];
unsigned char TX_Data[2];
unsigned char RX_ByteCtr;
unsigned char TX_ByteCtr;

int xGyro;
int yGyro;
int zGyro;

int xAccel;
int yAccel;
int zAccel;

const unsigned char minIMUAddress = 0x6B;  // Set slave address for minIMU

const unsigned char CTRL1_XL = 0x10;    // minIMU register address
const unsigned char CTRL2_G = 0x11;    // minIMU register address
const unsigned char CTRL3_C = 0x12;    // minIMU register address

const unsigned char ODR = 0x80;         // output data rate, 80 = 1.66kHz, +-2g range for XL, 245dps for Gyro
const unsigned char IF_INC = 0x04;      // auto increment register address on multiple read

const unsigned char OUTX_L_G = 0x22;    // minIMU register address
const unsigned char OUTX_H_G = 0x23;    // minIMU register address
const unsigned char OUTY_L_G = 0x24;    // minIMU register address
const unsigned char OUTY_H_G = 0x25;    // minIMU register address
const unsigned char OUTZ_L_G = 0x26;    // minIMU register address
const unsigned char OUTZ_H_G = 0x27;    // minIMU register address

const unsigned char OUTX_L_XL = 0x28;    // minIMU register address
const unsigned char OUTX_H_XL = 0x29;    // minIMU register address
const unsigned char OUTY_L_XL = 0x2A;    // minIMU register address
const unsigned char OUTY_H_XL = 0x2B;    // minIMU register address
const unsigned char OUTZ_L_XL = 0x2C;    // minIMU register address
const unsigned char OUTZ_H_XL = 0x2D;    // minIMU register address

void i2cInit(void);
void i2cWrite(unsigned char);
void i2cRead(unsigned char);
void imuSetup(void);
void imuRead();
//*******************************************************************

//*************************ANGLE CALCULATION VARS********************
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
//*******************************************************************

//*************************TIMER INTERRUPT VARS**********************
const long timerB0MaxCount = 31250;  // for 100Hz timer -> 25MHz clock, 8* divider, 31250 count = 100Hz
char timerB0Interrupted = 0;
//*******************************************************************

//*************************MOTOR CONTROL VARS************************
const int PWMMax = 1250;
int P24Int = 0;
int P25Int = 0;
int P20Int = 0;
int P22Int = 0;
long P245Counter = 0;
long P202Counter = 0;
int P245State = 0;
int P202State = 0;
int timerInterrupted = 0;
int factor = 1.1;
const int arraySize = 20;
//******************************************************************

//----------------------TIMING VARS---------------------------------
const long timerClockCount = 31250;
const long clockSpeed = 25000000;
//float interruptTime = timerClockCount / (clockSpeed/8);
const float interruptFreq = 100;//(clockSpeed/8)/timerClockCount;
const float interruptFreqPerMin = 6000;//interruptFreq * 60;
const int encoderRatio = 150 * 12;
const float RPMRatio = 3.3333;//interruptFreqPerMin / encoderRatio;
int state = 0;
//------------------------------------------------------------------

//----------------------SPEED CONTROLLER VARS-----------------------
float Kp = 0.5;
float Kd = 0;
float Ki = 0;
float Kv = 0;
float arrayAngles[arraySize] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int indexAngles = 0;
int RPMWant = 0;
//------------------------------------------------------------------


//------------------------RIGHT WHEEL--------------------------------
int PWMCurrentR = 600;
int RPMR = 0;
int ErrorR = 0;
int RPMWantR = 20;
float KpSetR = 0.05;
float KiSetR = 0.07;
float KpR;
float KiR;
float factorR = 1.1;
float integratorR = 0;
float arrayR[arraySize] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int indexR = 0;
int RWantDir = 1;

//------------------------LEFT WHEEL---------------------------------
int PWMCurrentL = 600;
int RPML = 0;
int ErrorL = 0;
int RPMWantL = 20;
float KpSetL = 0.05;
float KiSetL = 0.07;
float KpL;
float KiL;
float factorL = 1.1;
float integratorL = 0;
float arrayL[arraySize] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int indexL = 0;
int LWantDir = 1;
//*******************************************************************

//***************************FUNCTIONS*******************************
void encoderFunctionP245(void);
void encoderFunctionP202(void);

int sumArrayR(void)
{
    int i = 0;
    int total = 0;
    for (i = arraySize-1; i >= 0; i--)
    {
        arrayR[i] *= KiR;
        total+=arrayR[i];
    }
    return round(total);
}
int sumArrayL(void)
{
    int i = 0;
    int total = 0;
    for (i = arraySize-1; i >= 0; i--)
    {
        arrayR[i] *= KiL;
        total+=arrayR[i];
    }
    return round(total);
}

void motorRForward(void)
{
    TA0CCR1 = PWMMax;                     // CCR1 PWM duty cycle   1.2
    TA0CCR2 = PWMCurrentR;                            // CCR2 PWM duty cycle   1.3
}
void motorRBackward(void)
{
    TA0CCR1 = PWMCurrentR;                     // CCR1 PWM duty cycle   1.2
    TA0CCR2 = PWMMax ;                           // CCR2 PWM duty cycle   1.3
}
void motorLForward(void)
{
    TA0CCR3 = PWMMax;                // CCR2 PWM duty cycle   1.4
    TA0CCR4 = PWMCurrentL;                     // CCR1 PWM duty cycle   1.5
}
void motorLBackward(void)
{
    TA0CCR3 = PWMCurrentL;                     // CCR1 PWM duty cycle   1.4
    TA0CCR4 =  PWMMax;                         // CCR2 PWM duty cycle   1.5
}
//*******************************************************************

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop WDT

    PMM_setVCore(PMM_CORE_LEVEL_3);
    initClocks(clockSpeed);        // Config clocks. MCLK=SMCLK=FLL=4MHz;
    bcUartInit();               // Init the back-channel UART
    USB_setup(TRUE,TRUE);       // Init USB; if a USB host (PC) is present, connect

    // set up I2C pins
    P3SEL |= BIT0 + BIT1;                   // Assign I2C pins to USCI_B0

    // Initialize the I2C state machine
    i2cInit();

    imuSetup();

    // MOTOR AND ENCODER SETUP
    P1DIR |= BIT2 + BIT3 + BIT4 + BIT5;                       // P1.2 and P1.3 output
    P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;                       // P1.2 and P1.3 options select

    TA0CCR0 = PWMMax;                          // PWM Period SMCLK 25MHz - 1250 cycles: 25MHz/1250 = 20KHz signal

    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set   1.2
    TA0CCR1 = PWMCurrentR;                     // CCR1 PWM duty cycle   1.2

    TA0CCTL2 = OUTMOD_7;                      // CCR2 reset/set   1.3
    TA0CCR2 = PWMCurrentR;                            // CCR2 PWM duty cycle   1.3

    TA0CCTL3 = OUTMOD_7;                      // CCR3 reset/set   1.4
    TA0CCR3 = PWMCurrentL;                            // CCR3 PWM duty cycle   1.4

    TA0CCTL4 = OUTMOD_7;                      // CCR4 reset/set   1.5
    TA0CCR4 = PWMCurrentL;                            // CCR4 PWM duty cycle   1.5

    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR

    TA0CCR4 = PWMMax;                             // CCR4 PWM duty cycle    1.5
    TA0CCR2 = PWMMax;                            // CCR2 PWM duty cycle    1.3

    P4DIR |= BIT7;              // P4.7 to output
    P4OUT |= BIT7;             // ensure P4.7 out = 0

    P2DIR &= ~(BIT0 + BIT2 + BIT4 + BIT5);    // P2.0, P2.2, P2.4 and P2.5 to input
    P2IE |= BIT0 + BIT2 + BIT4 + BIT5;        // P2.0, P2.2, P2.4 and P2.5 int mask enable
    P2IES |= BIT0 + BIT2 + BIT4 + BIT5;       // P2.0, P2.2, P2.4 and P2.5 HI->LO egde int trigger
    P2REN |= BIT0 + BIT2 + BIT4 + BIT5;       // Enable resistor on P2.0, P2.2, P2.4 and P2.5
    P2OUT &= ~(BIT0 + BIT2 + BIT4 +BIT5);     // Set P2.0, P2.2, P2.4 and P2.5 to have pulldown resistor
    P2IFG &= ~(BIT0 + BIT2 + BIT4 + BIT5);    // P2.0, P2.2, P2.4 and P2.5 int flag clear

    //Setup Timer interrupt
    TB0CCTL0 = CCIE;                    //CCR0 interrupt enable
    TB0CTL = TBSSEL_2 | MC_1 | ID_3;    //SMCLK/8, upmode - TASSEL_2 selects SMCLK (now 8MHz), MC_1 is count up, ID_3 is internal divider for clock (set to 8* division)
    TB0CCR0 = timerB0MaxCount;          //max value 2^16-1 = 65525//Period = (SMCLK period/divisor)/(TB0CCR0)

    while (1)
    {
        if(P24Int)
        {
            P2IES ^= BIT4;      //P2.4 toggle between HI->LO and LO->HI edge int trigger
            P24Int = 0;
            encoderFunctionP245();
        }

        if(P25Int)
        {
            P2IES ^= BIT5;      //P2.4 toggle between HI->LO and LO->HI edge int trigger
            P25Int = 0;
            encoderFunctionP245();
        }
        if(P20Int)
        {
            P2IES ^= BIT0;      //P2.0 toggle between HI->LO and LO->HI edge int trigger
            P20Int = 0;
            encoderFunctionP202();
        }

        if(P22Int)
        {
            P2IES ^= BIT2;      //P2.2 toggle between HI->LO and LO->HI edge int trigger
            P22Int = 0;
            encoderFunctionP202();
        }

        if (timerB0Interrupted)
        {
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~Angle Estimation~~~~~~~~~~~~~~~~~~~~~~~
          // Read IMU XL and Gyro data, then place in array and increment index
          imuRead();
          xAccelArray[xAccelIndex] = xAccel;
          xAccelIndex = (xAccelIndex+1)%arrayLength;

          yAccelArray[yAccelIndex] = yAccel;
          yAccelIndex = (yAccelIndex+1)%arrayLength;

          zAccelArray[zAccelIndex] = zAccel;
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


          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END ANGLE EST~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~SPEED CONTROLLER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          RPMWant -= Kp * angleEst;

          if (RPMWant < - 100)  RPMWant = -100;
          else if (RPMWant > 100)    RPMWant = 100;

          if (RPMWant < 0)
          {
              RWantDir = -1;
              LWantDir = -1;
              RPMWantR = -RPMWant;
              RPMWantL = -RPMWant;
          }
          else
          {
              RWantDir = 1;
              LWantDir = 1;
              RPMWantR = RPMWant;
              RPMWantL = RPMWant;
          }
          sprintf(buf_usbToBcuart, "%d\n\r", (int) RPMWant);  //send to buffer
          bcUartSend(buf_usbToBcuart, 11);  //send to backchannel
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~END SPEED CONTROLLER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

          ///////////////////////////////////RIGHT WHEEL//////////////////////////////////////
          RPMR = round(P245Counter * RPMRatio);            //calculate RPM
          P245Counter = 0;

          ErrorR = RPMWantR - RPMR;                          //calculate Error
          arrayR[indexR] = ErrorR;                           //add integral terms to array (removes furthest away)
          indexR++;
          if (indexR >= arraySize)    indexR = 0;                  //ensure 0 <= index <= 9

          integratorR = sumArrayR();

          if(abs(ErrorR) <= 2)
          {
              KpR = KpSetR * 5;
              KiR = KiSetR * factor;
          }
          else if (abs(ErrorR) <= 4)
          {
              KpR = KpSetR * 4;
              KiR = KiSetR * factor;
          }
          else if (abs(ErrorR) <= 6)
          {
              KpR = KpSetR * 3;
              KiR = KiSetR * factor;
          }
          else if (abs(ErrorR) <= 8)
          {
              KpR = KpSetR * 2;
              KiR = KiSetR * factor;
          }
          else if (abs(ErrorR) > 8)
          {
              KpR = KpSetR * 3;
              KiR = KiSetR * 2;
          }
          PWMCurrentR = PWMMax-PWMCurrentR;

          PWMCurrentR += round(ErrorR*KpR) + round(integratorR*KiR);   //perform PI update

          if(PWMCurrentR >= PWMMax)                            //ensure PWMCurrent is between 0 and PWMMax
              PWMCurrentR = PWMMax;
          if (PWMCurrentR <= 0)
              PWMCurrentR = 0;

          PWMCurrentR = PWMMax-PWMCurrentR;

          if (RWantDir == 1)           motorRForward();          //update PWM and direction
          else                    motorRBackward();
          //////////////////////////////////END RIGHT WHEEL///////////////////////////////////

          ///////////////////////////////////LEFT WHEEL///////////////////////////////////////
          RPML= round(P202Counter * RPMRatio);            //calculate RPM
          //sprintf(buf_usbToBcuart, "%d\n\r", (int) RPML);  //send to buffer
          //bcUartSend(buf_usbToBcuart, 11);                //send to backchannel
          P202Counter = 0;

          ErrorL = RPMWantL - RPML;                          //calculate Error
          arrayL[indexL] = ErrorL;                           //add integral terms to array (removes furthest away)
          indexL++;
          if (indexL >= arraySize)    indexL = 0;                  //ensure 0 <= index <= 9

          integratorL = sumArrayL();

          if(abs(ErrorL) <= 2)
          {
              KpL = KpSetL * 5;
              KiL = KiSetL * factor;
          }
          else if (abs(ErrorL) <= 4)
          {
              KpL = KpSetL * 4;
              KiL = KiSetL * factor;
          }
          else if (abs(ErrorL) <= 6)
          {
              KpL = KpSetL * 3;
              KiL = KiSetL * factor;
          }
          else if (abs(ErrorL) <= 8)
          {
              KpL = KpSetL * 2;
              KiL = KiSetL * factor;
          }
          else if (abs(ErrorL) > 8)
          {
              KpL = KpSetL * 3;
              KiL = KiSetL * 2;
          }
          PWMCurrentL = PWMMax-PWMCurrentL;

          PWMCurrentL += round(ErrorL*KpL) + round(integratorL*KiL);   //perform PI update

          if(PWMCurrentL >= PWMMax)                            //ensure PWMCurrent is between 0 and PWMMax
              PWMCurrentL = PWMMax;
          if (PWMCurrentL <= 0)
              PWMCurrentL = 0;

          PWMCurrentL = PWMMax-PWMCurrentL;

          if (LWantDir == 1)           motorLForward();          //update PWM and direction
          else                    motorLBackward();

          ///////////////////////////////////END LEFT WHEEL//////////////////////////////////////
          timerB0Interrupted = 0;
        }
    }
}
//*********************************************************************************************
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
//*********************************************************************************************
void imuRead(void)
{
    // set up minIMU with specific settings
    TX_Data[0] = OUTX_L_G;             // register address
    TX_ByteCtr = 1;
    i2cWrite(minIMUAddress);

    RX_ByteCtr = 12;
    i2cRead(minIMUAddress);
    xGyro  = RX_Data[10] << 8;              // MSB
    xGyro |= RX_Data[11];                   // LSB
    yGyro  = RX_Data[8] << 8;               // MSB
    yGyro |= RX_Data[9];                    // LSB
    zGyro  = RX_Data[6] << 8;               // MSB
    zGyro |= RX_Data[7];                    // LSB

    xAccel  = RX_Data[4] << 8;              // MSB
    xAccel |= RX_Data[5];                   // LSB
    yAccel  = RX_Data[2] << 8;              // MSB
    yAccel |= RX_Data[3];                   // LSB
    zAccel  = RX_Data[0] << 8;              // MSB
    zAccel |= RX_Data[1];                   // LSB
}

//*********************************************************************************************
void imuSetup(void)
{
    // set up minIMU with specific settings
    TX_Data[1] = CTRL1_XL;             // register address
    TX_Data[0] = ODR;                  // register data - 0x80 for high frequency update
    TX_ByteCtr = 2;
    i2cWrite(minIMUAddress);

    TX_Data[1] = CTRL2_G;             // register address
    TX_Data[0] = ODR;                  // register data - 0x80 for high frequency update
    TX_ByteCtr = 2;
    i2cWrite(minIMUAddress);

    TX_Data[1] = CTRL3_C;             // register address
    TX_Data[0] = IF_INC;               // register data - 0x40 for auto increment
    TX_ByteCtr = 2;
    i2cWrite(minIMUAddress);
}

//*********************************************************************************************
void i2cInit(void)
{
    // set up I2C module
    UCB0CTL1 |= UCSWRST;                // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;           // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;          // Use SMCLK, keep SW reset
    UCB0BR0 = 250;                   // fSCL = SMCLK/250 = ~100kHz
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;               // Clear SW reset, resume operation
}

//*********************************************************************************************
void i2cWrite(unsigned char address)
{
    __disable_interrupt();
    UCB0I2CSA = address;                // Load slave address
    UCB0IE |= UCTXIE;                // Enable TX interrupt
    while(UCB0CTL1 & UCTXSTP);          // Ensure stop condition sent
    UCB0CTL1 |= UCTR + UCTXSTT;         // TX mode and START condition
    __bis_SR_register(CPUOFF + GIE);        // sleep until UCB0TXIFG is set ...
}

//*********************************************************************************************
void i2cRead(unsigned char address)
{
    __disable_interrupt();
    UCB0I2CSA = address;                // Load slave address
    UCB0IE |= UCRXIE;                // Enable RX interrupt
    while(UCB0CTL1 & UCTXSTP);          // Ensure stop condition sent
    UCB0CTL1 &= ~UCTR;              // RX mode
    UCB0CTL1 |= UCTXSTT;                // Start Condition
    __bis_SR_register(CPUOFF + GIE);        // sleep until UCB0RXIFG is set ...
}

/**********************************************************************************************/
// USCIAB0TX_ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: break;                           // Vector  4: NACKIFG
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case 10:                                  // Vector 10: RXIFG
  {
      RX_ByteCtr--;                       // Decrement RX byte counter
      if (RX_ByteCtr)                     // RxByteCtr != 0
      {
          RX_Data[RX_ByteCtr] = UCB0RXBUF;    // Get received byte
          if (RX_ByteCtr == 1)            // Only one byte left?
          UCB0CTL1 |= UCTXSTP;            // Generate I2C stop condition
      }
      else                        // RxByteCtr == 0
      {
          RX_Data[RX_ByteCtr] = UCB0RXBUF;    // Get final received byte
          __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
      }
  }
      break;
  case 12:                                  // Vector 12: TXIFG
      if (TX_ByteCtr)                     // TRUE if more bytes remain
      {
          TX_ByteCtr--;               // Decrement TX byte counter
          UCB0TXBUF = TX_Data[TX_ByteCtr];    // Load TX buffer
      }
      else                        // no more bytes to send
      {
          UCB0CTL1 |= UCTXSTP;            // I2C stop condition
          UCB0IFG &= ~UCTXIFG;            // Clear USCI_B0 TX int flag
          __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
      }
  default: break;
  }
}

//Timer B0 interrupt service routine
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B0(void)
{
    timerB0Interrupted = 1;
}

void encoderFunctionP245(void)      //reads P2.4 and P2.5 and determines direction
{                                   //via state machine. Also adds to counter for speed determination
    P245Counter++;
}


void encoderFunctionP202(void)      //reads P2.0 and P2.2 and determines direction
{
    P202Counter++;
}

//Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if(P2IFG & BIT4)
    {
        P24Int = 1;
    }
    if(P2IFG & BIT5)
    {
        P25Int = 1;
    }
    if(P2IFG & BIT0)
    {
        P20Int = 1;
    }
    if(P2IFG & BIT2)
    {
        P22Int = 2;
    }
    P2IFG &= ~(BIT0 + BIT2 + BIT4 + BIT5);    // P2.4 and P2.5 int flag clear
}
