/*
  Name: RoboDog_STM32_IO
	Description: STM32 GPIO for RoboDog
  Created:	2021.05.21
	Revised:	  
  Author:     DesignCell

	Ball Screw Details:
	3.5mm Bearing
	4.5mm Pitch
	Gearmotor 560rpm
	Belt 11/20T

 	Actuator Speed (No Load):
 	560R/m / 60s/m x 11T_Drv / 20T_Drvn x 4.5mm/Rev = 23.1mm/s

  Limits NC

	Wiring:
	Retract Limit:  C = BlueWhite	     NC = Blue
	Extend Limit:   C = OrangeWhite	   NC = Orange
	Gearmotor:		(-) = Black	(L)	    (+) = Red (R) When viewed from back

  (6) 3DP_Actuators Connected to (1) STM32

  Device    PIN   ::  PIN     Device
            PB12- ::  GND     
  LIM1.2_E  PB13- ::  GND     
  LIM1.1_R  PB14- ::  3.3v    
  LIM1.1_E  PB15- ::  NRST    
  PWM2.1    PA8~  ::  PB11-   I2C_SDA
  PWM2.2    PA9~  ::  PB10-   I2C_SCL
  PWM2.3    PA10~ ::  PB1~    LIM2.2_R
  DIR1.1    PA11- ::  PB0~    LIM2.2_E
  DIR1.2    PA12- ::  PA7~    LIM2.1_R
  DIR1.3    PA15- ::  PA6~    LIM2.1_E
  DIR2.1    PB3-  ::  PA5-    LIM1.3_R
  DIR2.2    PB4-  ::  PA4-    LIM1.3_E
  DIR2.3    PB5-  ::  PA3~    LIM1.2_R
  I2C_SCL   PB6~  ::  PA2~    PWM1.1
  I2C_SDA   PB7~  ::  PA1~    PWM1.2
  LIM2.3_R  PB8~  ::  PA0~    PWM1.3
  LIM2.3_E  PB9~  ::  PC15    
            5.0v  ::  PC14    
            GND   ::  PC13    
            3.3v  ::  VBAT   


	TB6612FNG
  Device    PIN   ::  PIN     Device
  M+ (RED)  M1+	  ::  DIR1    DIR (Color See table below)
  M- (BLK)  M1-	  ::  PWM1    PWM	''
  M- (BLK)  M2-	  ::  PWM2	  PWM	''
  M+ (RED)	M2+	  ::  DIR2    DIR	''
  WHT       GND   ::  GND	  BRN_W		    
  RED       VM	  ::  VCC     BRN

Driver Cable (Typical for each leg driver set)
#.# VCC :: BRN
#.# GND :: BRN_W
#.1 PWM :: GRN
#.1 DIR :: GRN_W
#.2 PWM :: BLU
#.2 DIR :: BLU_W
#.3 PWM :: ORN
#.3 DIR :: ORN_W

Actuator
Lim_R 	::	BLU
Lim_E	  ::	ORN
GND		  :: 	WHT ALL BONDED

AS5600
VCC ::  ORN_W
GND ::  BLU_W
SCL ::  BLU
SDA ::  ORN

*/

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <math.h>

//************************************************************************  I2C Address
TwoWire Wire2(PB11,PB10);
// 0 :: Front
// 2 :: Rear
#define I2C_Address 2

int8_t Btn_Stop;
double Spd_Limit = 255;
int8_t In_Kp;

//PID Parameters	
double Kp = 20, Ki = Kp / 10.0, Kd = Kp / 30.0;
uint8_t pwm_deadzone = 25;

AMS_5600 AS5600;

// [0] = LEFT :: [1] = RIGHT
int16_t Leg_Angle_Setpoint_Shoulder[2];
int16_t Leg_Angle_Setpoint_Upper[2];
int16_t Leg_Angle_Setpoint_Lower[2];

int16_t Leg_Angle_Measured_Shoulder[2];
int16_t Leg_Angle_Measured_Upper[2];
int16_t Leg_Angle_Measured_Lower[2];

int16_t Leg_PWM_Shoulder[2];
int16_t Leg_PWM_Upper[2];
int16_t Leg_PWM_Lower[2];

// for PID...
double _Leg_Angle_Setpoint_Shoulder[2];
double _Leg_Angle_Setpoint_Upper[2];
double _Leg_Angle_Setpoint_Lower[2];

double _Leg_Angle_Measured_Shoulder[2];
double _Leg_Angle_Measured_Upper[2];
double _Leg_Angle_Measured_Lower[2];

double _Leg_PWM_Shoulder[2];
double _Leg_PWM_Upper[2];
double _Leg_PWM_Lower[2];

// Angle Soft Limits
int16_t aLimit_Shoulder = 180;
int16_t aLimit_Upper_min = 280;
int16_t aLimit_Upper_max = 640;
int16_t aLimit_Lower_min = 730;
int16_t aLimit_Lower_max = 1290;

PID Act_Shoulder_1(
  &_Leg_Angle_Measured_Shoulder[0],
  &_Leg_PWM_Shoulder[0],
  &_Leg_Angle_Setpoint_Shoulder[0],
  Kp, Ki, Kd, 0);
PID Act_Upper_1(
  &_Leg_Angle_Measured_Upper[0],
  &_Leg_PWM_Upper[0],
  &_Leg_Angle_Setpoint_Upper[0],
  Kp, Ki, Kd, 0);
PID Act_Lower_1(
  &_Leg_Angle_Measured_Lower[0],
  &_Leg_PWM_Lower[0],
  &_Leg_Angle_Setpoint_Lower[0],
  Kp, Ki, Kd, 0);

PID Act_Shoulder_2(
  &_Leg_Angle_Measured_Shoulder[1],
  &_Leg_PWM_Shoulder[1],
  &_Leg_Angle_Setpoint_Shoulder[1],
  Kp, Ki, Kd, 0);
PID Act_Upper_2(
  &_Leg_Angle_Measured_Upper[1],
  &_Leg_PWM_Upper[1],
  &_Leg_Angle_Setpoint_Upper[1],
  Kp, Ki, Kd, 0);
PID Act_Lower_2(
  &_Leg_Angle_Measured_Lower[1],
  &_Leg_PWM_Lower[1],
  &_Leg_Angle_Setpoint_Lower[1],
  Kp, Ki, Kd, 0);

//Limit Switches Pins
#define LIM_11_E PB15
#define LIM_11_R PB14
#define LIM_12_E PB13
#define LIM_12_R PA3
#define LIM_13_E PA4
#define LIM_13_R PA5
#define LIM_21_E PA6
#define LIM_21_R PA7
#define LIM_22_E PB0
#define LIM_22_R PB1
#define LIM_23_E PB9
#define LIM_23_R PB8

bool LIMIT[3][4][2]; // Ignoring [0][0]

//TB6612FNG DC Motor Driver
#define PWM_11 PA2
#define DIR_11 PA11
#define PWM_12 PA1
#define DIR_12 PA12
#define PWM_13 PA0
#define DIR_13 PA15
#define PWM_21 PA8
#define DIR_21 PB3
#define PWM_22 PA9
#define DIR_22 PB4
#define PWM_23 PA10
#define DIR_23 PB5

#define LED PC13

bool pulse = 0;

uint16_t Loop_time; // Loop time debug observation
uint32_t pLoop;
uint32_t cWatchdog; // I2C Timeout Watchdog Current
uint32_t pWatchdog; // I2C Timeout Watchdog Previous

void receiveEvent(int howMany)  // 7us
{
  byte a,b;
  // LEFT
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Shoulder[0] = a;
  Leg_Angle_Setpoint_Shoulder[0] = Leg_Angle_Setpoint_Shoulder[0] << 8 | b;
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Upper[0] = a;
  Leg_Angle_Setpoint_Upper[0] = Leg_Angle_Setpoint_Upper[0] << 8 | b;
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Lower[0] = a;
  Leg_Angle_Setpoint_Lower[0] = Leg_Angle_Setpoint_Lower[0] << 8 | b;

  // RIGHT
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Shoulder[1] = a;
  Leg_Angle_Setpoint_Shoulder[1] = Leg_Angle_Setpoint_Shoulder[1] << 8 | b;
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Upper[1] = a;
  Leg_Angle_Setpoint_Upper[1] = Leg_Angle_Setpoint_Upper[1] << 8 | b;
  a = Wire2.read();
  b = Wire2.read();
  Leg_Angle_Setpoint_Lower[1] = a;
  Leg_Angle_Setpoint_Lower[1] = Leg_Angle_Setpoint_Lower[1] << 8 | b;

  Btn_Stop = Wire2.read(); // 0-1
  In_Kp = Wire2.read(); // 0-255

  Kp = Wire2.read();
  Ki = Wire2.read();
  Kd = Wire2.read();

}

void requestEvent() // 6-7us
{
  byte bitArray[14];
 
  // LEFT
  bitArray[0] = (Leg_Angle_Measured_Shoulder[0] >> 8) & 0xFF;
  bitArray[1] = Leg_Angle_Measured_Shoulder[0] & 0xFF;
  bitArray[2] = (Leg_Angle_Measured_Upper[0] >> 8) & 0xFF;
  bitArray[3] = Leg_Angle_Measured_Upper[0] & 0xFF;
  bitArray[4] = (Leg_Angle_Measured_Lower[0] >> 8) & 0xFF;
  bitArray[5] = Leg_Angle_Measured_Lower[0] & 0xFF;
  // RIGHT
  bitArray[6] = (Leg_Angle_Measured_Shoulder[1] >> 8) & 0xFF;
  bitArray[7] = Leg_Angle_Measured_Shoulder[1] & 0xFF;
  bitArray[8] = (Leg_Angle_Measured_Upper[1] >> 8) & 0xFF;
  bitArray[9] = Leg_Angle_Measured_Upper[1] & 0xFF;
  bitArray[10] = (Leg_Angle_Measured_Lower[1] >> 8) & 0xFF;
  bitArray[11] = Leg_Angle_Measured_Lower[1] & 0xFF;
  // Loop Time
  bitArray[12] = (Loop_time >> 8) & 0xFF;
  bitArray[13] = Loop_time & 0xFF;

  Wire2.write(bitArray, 14);

  pulse = 1;
  pWatchdog = micros();
}

/****************************************************
  Sets I2C Multiplexer channel 1-8
*****************************************************/
void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70); // TCA9548A address is 0x70
  Wire.write(1 << bus);
  Wire.endTransmission();
}

/****************************************************
  Sets Min/Max angles for each AS5600
  Values determined through observation.
*****************************************************/
void AS5600_setCalibration_I2C0()
{
  /*
  ANGLE * 4096 / 360 = RAW
  RAW * 360 / 4096 = ANGLE
  RAW * 3600 / 4096 = ANGLEx10

  Process:
  Set acturator to min
  Confirm acturator min length to cad
  Observe actuator min length joint angle
  Read current position raw
  Offset raw by joint angle and save as start position
  */
  TCA9548A(0); //ch1 Front Left Shoulder
  AS5600.setStartPosition(3794); // Raw 960 - 1252, Angle 110.00 * (4096/360) = 1251.55, Min limit +20d

  TCA9548A(1); //ch1 Front Left Upper
  AS5600.setStartPosition(353); // Raw 668 - 315, min Angle 27.70 * (4096/360) = 315.16

  TCA9548A(2); //ch1 Front Left Lower
  AS5600.setStartPosition(2235); // Raw 3060 - 825, min Angle 72.50 * (4096/360) = 824.88

  TCA9548A(3); //ch1 Front Right Shoulder
  AS5600.setStartPosition(1289); // Raw 2085 - 796, Angle 70.00 * (4096/360) = 796.44, Min limit -20d

  TCA9548A(4); //ch1 Front Right Upper
  AS5600.setStartPosition(3183); // Raw 3498 - 315, min Angle 27.70 * (4096/360) = 315.16

  TCA9548A(5); //ch1 Front Right Lower
  AS5600.setStartPosition(160); // Raw 985 - 825, min Angle 72.50 * (4096/360) = 824.88

}

void AS5600_setCalibration_I2C2()
{
  TCA9548A(0); //ch1 Rear Left Shoulder
  AS5600.setStartPosition(2033); // Raw 3285 - 1252, Angle 110.00 * (4096/360) = 1251.55, Min limit +20d

  TCA9548A(1); //ch1 Rear Left Upper
  AS5600.setStartPosition(2209); // Raw 2524 - 315, min Angle 27.70 * (4096/360) = 315.16

  TCA9548A(2); //ch1 Rear Left Lower
  AS5600.setStartPosition(35); // Raw 860 - 825, min Angle 72.50 * (4096/360) = 824.88

  TCA9548A(3); //ch1 Rear Right Shoulder
  AS5600.setStartPosition(570); // Raw 1380 - 796, Angle 70.00 * (4096/360) = 796.44, Min limit -20d  was 584

  TCA9548A(4); //ch1 Rear Right Upper
  AS5600.setStartPosition(3305); // Raw 3620 - 315, min Angle 27.70 * (4096/360) = 315.16

  TCA9548A(5); //ch1 Rear Right Lower
  AS5600.setStartPosition(3230); // Raw 4055 - 825, min Angle 72.50 * (4096/360) = 824.88

}

void Init_PID()
{

  Act_Shoulder_1.SetMode(AUTOMATIC);
  Act_Shoulder_1.SetTunings(Kp, Ki, Kd);
  Act_Shoulder_1.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation
  
  Act_Shoulder_2.SetMode(AUTOMATIC);
  Act_Shoulder_2.SetTunings(Kp, Ki, Kd);
  Act_Shoulder_2.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation
  
  Act_Upper_1.SetMode(AUTOMATIC);
  Act_Upper_1.SetTunings(Kp, Ki, Kd);
  Act_Upper_1.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation
  
  Act_Upper_2.SetMode(AUTOMATIC);
  Act_Upper_2.SetTunings(Kp, Ki, Kd);
  Act_Upper_2.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation

  Act_Lower_1.SetMode(AUTOMATIC);
  Act_Lower_1.SetTunings(Kp, Ki, Kd);
  Act_Lower_1.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation
  
  Act_Lower_2.SetMode(AUTOMATIC);
  Act_Lower_2.SetTunings(Kp, Ki, Kd);
  Act_Lower_2.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -LIMIT to +LIMIT allows negative rotation
}

void Tune_PID()
{
  Kp = In_Kp / 10.0;
  Ki = Kp / 10.0;
  Kd = Kp / 30.0;
  Act_Shoulder_1.SetTunings(Kp, Ki, Kd);
  Act_Shoulder_2.SetTunings(Kp, Ki, Kd);
  Act_Upper_1.SetTunings(Kp, Ki, Kd);
  Act_Upper_2.SetTunings(Kp, Ki, Kd);
  Act_Lower_1.SetTunings(Kp, Ki, Kd);
  Act_Lower_2.SetTunings(Kp, Ki, Kd);
}

// Initialize IO
void Init_IO()
{
  // Initialize Limit Input Pins
  // Limit Switched wired NC, Input Pullup
  pinMode(LIM_11_E,INPUT_PULLUP);
  pinMode(LIM_11_R,INPUT_PULLUP);
  pinMode(LIM_12_E,INPUT_PULLUP);
  pinMode(LIM_12_R,INPUT_PULLUP);
  pinMode(LIM_13_E,INPUT_PULLUP);
  pinMode(LIM_13_R,INPUT_PULLUP);
  pinMode(LIM_21_E,INPUT_PULLUP);
  pinMode(LIM_21_R,INPUT_PULLUP);
  pinMode(LIM_22_E,INPUT_PULLUP);
  pinMode(LIM_22_R,INPUT_PULLUP);
  pinMode(LIM_23_E,INPUT_PULLUP);
  pinMode(LIM_23_R,INPUT_PULLUP);

  // Initialize DC Motor Driver Direction Output Pins
  // HIGH = Retract, LOW = Extend
  pinMode(DIR_11,OUTPUT);
  pinMode(DIR_12,OUTPUT);
  pinMode(DIR_13,OUTPUT);
  pinMode(DIR_21,OUTPUT);
  pinMode(DIR_22,OUTPUT);
  pinMode(DIR_23,OUTPUT);

  // Initialize DC Motor Driver PWM Output Pins
  pinMode(PWM_11,OUTPUT);
  pinMode(PWM_12,OUTPUT);
  pinMode(PWM_13,OUTPUT);
  pinMode(PWM_21,OUTPUT);
  pinMode(PWM_22,OUTPUT);
  pinMode(PWM_23,OUTPUT);

  // LED I2C Active
  pinMode(LED,OUTPUT);

}

void setup()
{
  Wire.begin();

  Wire2.begin(I2C_Address);

  Wire2.onReceive(receiveEvent); // register event
  Wire2.onRequest(requestEvent); // register event

  Init_IO();
  Init_PID();

  if (I2C_Address == 0)
  {
    AS5600_setCalibration_I2C0();
  } else if (I2C_Address == 2)
  {
    AS5600_setCalibration_I2C2();
  }
}

/****************************************************
  Cycles through AS5600 sensors and sets Leg_Angle
*****************************************************/
void multiplexAS5600()
{
  float mA, b, c;

  TCA9548A(0);  // #.1
  mA = AS5600.getScaledAngle() * 360.0 / 4096.0 - 90.0; //offset zero -90°
  b = degrees(asin((73.095 * sin(radians(mA + 90)))/142.628));
  c = 180 - (mA + 90) - b;
  _Leg_Angle_Measured_Shoulder[0] = (61.221 - c) * 10;
  Leg_Angle_Measured_Shoulder[0] = int(_Leg_Angle_Measured_Shoulder[0]);

  TCA9548A(1);  // #.2
  Leg_Angle_Measured_Upper[0] = AS5600.getScaledAngle() * 3600 / 4096; // Scale x10 degrees 
  _Leg_Angle_Measured_Upper[0] = double(Leg_Angle_Measured_Upper[0]);

  TCA9548A(2);  // #.3
  Leg_Angle_Measured_Lower[0] = AS5600.getScaledAngle() * 3600 / 4096; // Scale x10 degrees 
  _Leg_Angle_Measured_Lower[0] = double(Leg_Angle_Measured_Lower[0]);

  TCA9548A(3);  // #.1
  mA = AS5600.getScaledAngle() * 360.0 / 4096.0 - 90.0; //offset zero -90°
  b = degrees(asin((73.095 * sin(radians(90 - mA)))/142.628));
  c = 180 - (90 - mA) - b;
  _Leg_Angle_Measured_Shoulder[1] = -(61.2114 - c) * 10;
  Leg_Angle_Measured_Shoulder[1] = int(_Leg_Angle_Measured_Shoulder[1]);

  TCA9548A(4);  // #.2
  Leg_Angle_Measured_Upper[1] = AS5600.getScaledAngle() * 3600 / 4096; // Scale x10 degrees 
  _Leg_Angle_Measured_Upper[1] = double(Leg_Angle_Measured_Upper[1]);

  TCA9548A(5);  // #.3
  Leg_Angle_Measured_Lower[1] = AS5600.getScaledAngle() * 3600 / 4096; // Scale x10 degrees
  _Leg_Angle_Measured_Lower[1] = double(Leg_Angle_Measured_Lower[1]);

}

void Limit_Read()
{
  // LIMIT[0/LEFT/RIGHT][0/SHOULDER/UPPER/LOWER][E/R]
  // 0 = NC :: 1 = On Limit
  /*
  LIMIT[1][1][0] = digitalReadFast(digitalPinToPinName(LIM_11_E));
  LIMIT[1][2][0] = digitalReadFast(digitalPinToPinName(LIM_12_E));
  LIMIT[1][3][0] = digitalReadFast(digitalPinToPinName(LIM_13_E));
  LIMIT[1][1][1] = digitalReadFast(digitalPinToPinName(LIM_11_R));
  LIMIT[1][2][1] = digitalReadFast(digitalPinToPinName(LIM_12_R));
  LIMIT[1][3][1] = digitalReadFast(digitalPinToPinName(LIM_13_R));
  LIMIT[2][1][0] = digitalReadFast(digitalPinToPinName(LIM_21_E));
  LIMIT[2][2][0] = digitalReadFast(digitalPinToPinName(LIM_22_E)); // Front Limit Failure
  LIMIT[2][3][0] = digitalReadFast(digitalPinToPinName(LIM_23_E)); // Front Limit Failure
  LIMIT[2][1][1] = digitalReadFast(digitalPinToPinName(LIM_21_R));
  LIMIT[2][2][1] = digitalReadFast(digitalPinToPinName(LIM_22_R)); // Front Limit Failure
  LIMIT[2][3][1] = digitalReadFast(digitalPinToPinName(LIM_23_R)); // Front Limit Failure
  */

  // 1.1 Shoulder
  if (Leg_Angle_Measured_Shoulder[0] < -aLimit_Shoulder)
  {
    Act_Shoulder_1.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Shoulder[0] > aLimit_Shoulder)
  {
    Act_Shoulder_1.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Shoulder_1.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }
  // 2.1 Shoulder
  if (Leg_Angle_Measured_Shoulder[1] > aLimit_Shoulder)
  {
    Act_Shoulder_2.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Shoulder[0] < -aLimit_Shoulder)
  {
    Act_Shoulder_2.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Shoulder_2.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }

  // 1.2 Upper
  if (Leg_Angle_Measured_Upper[0] > aLimit_Upper_max)
  {
    Act_Upper_1.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Upper[0] < aLimit_Upper_min)
  {
    Act_Upper_1.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Upper_1.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }
  // 2.2 Upper
  if (Leg_Angle_Measured_Upper[1] > aLimit_Upper_max)
  {
    Act_Upper_2.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Upper[1] < aLimit_Upper_min)
  {
    Act_Upper_2.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Upper_2.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }

  // 1.3 Lower
  if (Leg_Angle_Measured_Lower[0] > aLimit_Lower_max)
  {
    Act_Lower_1.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Lower[0] < aLimit_Lower_min)
  {
    Act_Lower_1.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Lower_1.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }
  // 2.3 Lower
  if (Leg_Angle_Measured_Lower[1] > aLimit_Lower_max)
  {
    Act_Lower_2.SetOutputLimits(-Spd_Limit, 0); // Limit Extend
  } else if (Leg_Angle_Measured_Lower[1] < aLimit_Lower_min)
  {
    Act_Lower_2.SetOutputLimits(0, Spd_Limit); // Limit Retract
  } else
  {
    Act_Lower_2.SetOutputLimits(-Spd_Limit, Spd_Limit); // Open Travel
  }

}

void Float_Setpoints()
{
  _Leg_Angle_Setpoint_Shoulder[0] = float(Leg_Angle_Setpoint_Shoulder[0]);
  _Leg_Angle_Setpoint_Shoulder[1] = float(Leg_Angle_Setpoint_Shoulder[1]);
  _Leg_Angle_Setpoint_Upper[0] = float(Leg_Angle_Setpoint_Upper[0]);
  _Leg_Angle_Setpoint_Upper[1] = float(Leg_Angle_Setpoint_Upper[1]);
  _Leg_Angle_Setpoint_Lower[0] = float(Leg_Angle_Setpoint_Lower[0]);
  _Leg_Angle_Setpoint_Lower[1] = float(Leg_Angle_Setpoint_Lower[1]);
}

void Drive_PWM()
{
  // Direction LOW = Extend, LIMIT[#][#][0], Limit Active LOW
  // Direction HIGH = Retract, LIMIT[#][#][1], Limit Active LOW
  // _Leg_PWM_Shoulder/Upper/Lower[0/1]
  
  // 1.1
	if (_Leg_PWM_Shoulder[0] > pwm_deadzone) // Retract = +Angle
	{
		digitalWrite(DIR_11, HIGH); // Retract
		analogWrite(PWM_11, _Leg_PWM_Shoulder[0]); // (+) PWM
	}
	else if (_Leg_PWM_Shoulder[0] < -pwm_deadzone) // Extend = -Angle
	{
		digitalWrite(DIR_11, LOW); // Extend
		analogWrite(PWM_11, -_Leg_PWM_Shoulder[0]); // Flip (-) PWM
	}
	else 
	{
		digitalWrite(DIR_11, LOW);
		analogWrite(PWM_11, 0);
	}

  // 1.2
	if (_Leg_PWM_Upper[0] > pwm_deadzone) // Extend
	{
		digitalWrite(DIR_12, LOW); // Extend
		analogWrite(PWM_12, _Leg_PWM_Upper[0]); // (+) PWM
	}
	else if (_Leg_PWM_Upper[0] < -pwm_deadzone) // Retract
	{
		digitalWrite(DIR_12, HIGH); // Retract
		analogWrite(PWM_12, -_Leg_PWM_Upper[0]); // Flip (-) PWM
	}
	else 
	{ 
		digitalWrite(DIR_12, LOW);
		analogWrite(PWM_12, 0);
	}

  // 1.3
	if (_Leg_PWM_Lower[0] > pwm_deadzone) // Extend
	{
		digitalWrite(DIR_13, LOW); // Extend
		analogWrite(PWM_13, _Leg_PWM_Lower[0]); // (+) PWM
	}
	else if (_Leg_PWM_Lower[0] < -pwm_deadzone) // Retract
	{
		digitalWrite(DIR_13, HIGH); // Retract
		analogWrite(PWM_13, -_Leg_PWM_Lower[0]); // Flip (-) PWM
	}
	else 
	{ 
		digitalWrite(DIR_13, LOW);
		analogWrite(PWM_13, 0);
	}
  
  // 2.1
 	if (_Leg_PWM_Shoulder[1] > pwm_deadzone) // Extend
	{
		digitalWrite(DIR_21, LOW); // Extend
		analogWrite(PWM_21, _Leg_PWM_Shoulder[1]); // (+) PWM
	}
	else if (_Leg_PWM_Shoulder[1] < -pwm_deadzone) // Retract
	{
		digitalWrite(DIR_21, HIGH); // Retract
		analogWrite(PWM_21, -_Leg_PWM_Shoulder[1]); // Flip (-) PWM
	}
	else 
	{ 
		digitalWrite(DIR_21, LOW);
		analogWrite(PWM_21, 0);
	}

  // 2.2
	if (_Leg_PWM_Upper[1] > pwm_deadzone) // Extend
	{
		digitalWrite(DIR_22, LOW); // Extend
		analogWrite(PWM_22, _Leg_PWM_Upper[1]); // (+) PWM
	}
	else if (_Leg_PWM_Upper[1] < -pwm_deadzone) // Retract
	{
		digitalWrite(DIR_22, HIGH); // Retract
		analogWrite(PWM_22, -_Leg_PWM_Upper[1]); // Flip (-) PWM
	}
	else 
	{ 
		digitalWrite(DIR_22, LOW);
		analogWrite(PWM_22, 0);
	}

  // 2.3
	if (_Leg_PWM_Lower[1] > pwm_deadzone) // Extend
	{
		digitalWrite(DIR_23, LOW); // Extend
		analogWrite(PWM_23, _Leg_PWM_Lower[1]); // (+) PWM
	}
	else if (_Leg_PWM_Lower[1] < -pwm_deadzone) // Retract
	{
		digitalWrite(DIR_23, HIGH); // Retract
		analogWrite(PWM_23, -_Leg_PWM_Lower[1]); // Flip (-) PWM
	}
	else 
	{ 
		digitalWrite(DIR_23, LOW);
		analogWrite(PWM_23, 0);
	}
  
}

void Zero_PWM()
{
  analogWrite(PWM_11,0);
  analogWrite(PWM_12,0);
  analogWrite(PWM_13,0);
  analogWrite(PWM_21,0);
  analogWrite(PWM_22,0);
  analogWrite(PWM_23,0);
}


void loop()
{
  if (micros() - pWatchdog > 40000) // If lost I2C receive then hault PWMs
  {
    Zero_PWM();
  }

	if (pulse == 1)
  {
    pLoop = micros();
    digitalWrite(LED,LOW); // LED On
    pulse = 0; // reset pulse flag

    multiplexAS5600();
    Limit_Read();
    Float_Setpoints();

    Tune_PID();

    Act_Shoulder_1.Compute();
    Act_Shoulder_2.Compute();
    Act_Upper_1.Compute();
    Act_Upper_2.Compute();
    Act_Lower_1.Compute();
    Act_Lower_2.Compute();

    if (Btn_Stop == 1)
    {
      Drive_PWM();
    } else
    {
      Zero_PWM();
    }
    Loop_time = micros() - pLoop; // Core loop time excluding pulse interval      
  } 
  digitalWrite(LED,HIGH); // LED Off

}

