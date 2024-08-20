/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  29 December 2022 -  initial release
*/
// Seems to achieve ~ 200 steps/sec max with this code

#include <Wire.h>
#include <AccelStepper.h>

//Motor Setup
//User-defined values
int directionMultiplier = 1;
AccelStepper myStepper(1, 6, 5);


//int driverPUL = 6;
//int driverDIR = 5;
//int pd = 100;
//boolean setdir = LOW;
//---------------------------------------------------

const int CSn = 2; // Chip select
const int CLK = 3; // Clock signal
const int DO = 8; // Digital Output from the encoder which delivers me a 0 or 1, depending on the bar angle..

unsigned int sensorWaarde = 0;
float AnglePosition;
float AnglePosition_Prior;
float Speed;

unsigned long startMillis;
unsigned long endMillis;
unsigned long elapsedMillis;
unsigned long TotalTime;

unsigned long previousMicros = 0;
unsigned long currentMicros = 0;

//Accelerometer Setup Starts
// Declare all gyroscope and accelerometer values (seen in part 14)
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
int count1 = 1;
int count2 = 1;

// Define the roll and pitch angles combing from the Kalman filter
  // defining the predicted angles and uncertainties
  // Initial guess for the angle values is zero because quadcopter will generally take off from a level surface
    // But surface can never be exactly level so take the uncertainty on the intial guess
    // for the angles to be 2 degrees
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
//float KalmanAnglePitchPrev=0;
//int counter = 0;

// Define the output from the Kalman filter
  // these are two variables
    // the common prediction for the state (angle in our case) and the uncertainty on this prediction
      // both variables are updated during each iteration
float Kalman1DOutput[]={0,0};

// Create the function that calculates the predicted angle and uncertainty using the Kalman equations
  // 1. Predict the current state of the system with rotation rate integration
  // 2. Calculate uncertainty on the prediction
  // 3. Calculate the Kalman gain from the uncertainties on the prediction and measurements
  // 4. Update the predicted state with the accelerometer measurements
  // 5. Update the uncertainty of the predicted state
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

  // The outout of the Kalman filter function consists of a prediction for the state which is the angle
  // and the corresponding uncertainty
    // **Remember the Kalman input contain the rotation rate measurement the Kalman measurement and the accelerometer angle measurement
      // AND that Kalman state contains the angle calculated with the Kalman filter
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Remaining parts are the same as project 14
  // reads the rotation rates, acceleration and angles from the MPU-6050
// ***DO NOT FORGET TO PUT OWN CALIBRATION VALUES FOR THE NUMBERS HIGHLIGHTED (accelerometer) BEFORE CALCULATING ANGLES***
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;

  // My own calibrated values from part 14
  AccX=(float)AccXLSB/4096 - 0.06;      
  AccY=(float)AccYLSB/4096 + 0.02;
  AccZ=(float)AccZLSB/4096 - 0.11;

  // Part 14
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

// Setup parts where you communicate with they gyroscope and calibrate it comes straight from parts 4 and 5
void setup() {
  
  Serial.begin(115200);

  myStepper.setMaxSpeed(200); //SPEED = Steps / second
  myStepper.setAcceleration(50); //ACCELERATION = Steps / (second)^2

// For Encoder
  pinMode(CSn, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DO, INPUT);

  digitalWrite(CLK, HIGH);
  digitalWrite(CSn, HIGH);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  // Trying to change here to see if the angle updates faster
  delay(100);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }

// Calibrating and starting the rotation rates
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

// Once that is all started/calculated, can start the Kalman filter function
void loop() {
  
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  // Calculate roll angle and uncertainty on the Kalman angle together with the measured rotation rate and angle
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  // The output of the filter will give you the updated common angle and its associated uncertainty
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  // Repeat the same for the pitch angle and print the predicted angle values 
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1]; 

//  Serial.print(" Pitch Angle [°] ");
//  Serial.println(KalmanAnglePitch);

  //Serial.print("The difference between angles is ");
  //Serial.println(abs(KalmanAnglePitchPrev - KalmanAnglePitch));

  if ((KalmanAnglePitch <= -5.0) && (KalmanAnglePitch > -60.0)){
    //Serial.println("Reaching inside if statement");
    myStepper.setSpeed(-150);
    myStepper.runSpeed();

    if ((count1 == 1) && ((KalmanAnglePitch <= -15.0) && (KalmanAnglePitch > -20.0))){
      startMillis = millis();
      Serial.print("Start time: ");
      Serial.print(startMillis);
      sensorWaarde = readSensor();
      Serial.print(" Accelerometer Angle: ");
      Serial.println(KalmanAnglePitch);
      count1 = count1 + 1; 
    }  
    
  }
  else if ((KalmanAnglePitch >= 5.0) && (KalmanAnglePitch < 60.0)){
    //Serial.println("should not be in here");
    myStepper.setSpeed(150);
    myStepper.runSpeed();
  
  if (((KalmanAnglePitch >= 15.0) && (KalmanAnglePitch < 20.0)) && (count2 == 1)){
      startMillis = millis();
      Serial.print("Start time: ");
      Serial.print(startMillis);
      sensorWaarde = readSensor();
      Serial.print(" Accelerometer Angle: ");
      Serial.println(KalmanAnglePitch);
      count2 = count2 + 1;
  } 
    
}
  else {
   myStepper.stop();

   endMillis = millis();
   sensorWaarde = readSensor();
   Serial.print(" End Time: ");
   Serial.print(endMillis);
   Serial.print(" Final Accelerometer Angle: ");
   Serial.println(KalmanAnglePitch);

  }
}
//  Serial.print("Roll Angle [°] ");
//  Serial.print(KalmanAngleRoll);
//  Serial.print(" Pitch Angle [°] ");
//  Serial.println(KalmanAnglePitch); 

// Encoder Reading

//if (KalmanPitchAngle < 5) && (KalmanPitchAngle > -10){
// sensorWaarde = readSensor();
// Serial.println(dataOut);
// delayMicroseconds(1); //Tcs waiting for another read in
//
//
// 
//

unsigned int readSensor(){
  unsigned int dataOut = 0;

  digitalWrite(CSn, LOW);
  delayMicroseconds(1); //Waiting for Tclkfe

  //Passing 12 times, from 0 to 11
  for(int x=0; x<12; x++){
    digitalWrite(CLK, LOW); 
    delayMicroseconds(1); //Tclk/2
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1); //Tdo valid, like Tclk/2
    dataOut = (dataOut << 1) | digitalRead(DO); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
  }
//    AnglePosition = (dataOut*0.0879);
//    currentMicros = micros();
//    unsigned long elapsedMicros = currentMicros-previousMicros;
//    Speed = ((abs(AnglePosition-AnglePosition_Prior))/elapsedMicros) * 1000000.0;
//    
  digitalWrite(CSn, HIGH); //deselects the encoder from reading
//        Serial.print(" Encoder value:");
//        Serial.print(dataOut);
//        Serial.print(" Angular value:");
//        Serial.print(AnglePosition);
//        Serial.print(" Speed (deg/s)");
//        Serial.println(Speed);
  Serial.print( " Encoder: ");
  Serial.print(dataOut);
  return dataOut;
//
 //AnglePosition_Prior = AnglePosition;
//  // This value was 4000 prior
//
  //previousMicros = currentMicros;
 
//  counter = counter + 1;
//  // Need to know the previous from 40 prior
//  if (counter % 10 == 0){
//  KalmanAnglePitchPrev = KalmanAnglePitch;
//  //delayMicroseconds(pd);

//while (micros() - LoopTimer < 2000);
//  LoopTimer=micros();
//  }
}
