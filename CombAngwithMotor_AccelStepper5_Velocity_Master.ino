//Workspace Volume
// Main AccelStepper Code
// Use to move so many steps until the straps on the belt cannot go farther on both directions
// Record number of steps moved
//Implement encoder
#include <AccelStepper.h>;
#include <Wire.h>

long receivedSteps = 0; //Number of steps
long receivedSpeed = 0; //Steps / second
long receivedAcceleration = 0; //Steps / second^2
char receivedCommand;
int directionMultiplier = 1;
bool newData, runallowed = false;

// for timer
//unsigned long startMicros;
//unsigned long endMicros;
//unsigned long elapsedMicros;
//unsigned long TotalTime;

unsigned long startMillis;
unsigned long endMillis;
unsigned long elapsedMillis;
unsigned long TotalTime;


//for encoder
const int CSn = 2; // Chip select
const int CLK = 3; // Clock signal
const int DO = 8; // Digital Output from the encoder which delivers me a 0 or 1, depending on the bar angle..
//int EncoderValues;
float AnglePosition;
unsigned int sensorWaarde = 0;

AccelStepper stepper(1, 6, 5);

void setup() {

  Serial.begin(57600);
  Serial.println("Ready to receive command");
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(10000000000);

  stepper.disableOutputs(); //disable outputs

  // For Encoder
  pinMode(CSn, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DO, INPUT);

  digitalWrite(CLK, HIGH);
  digitalWrite(CSn, HIGH);

}

void loop() {
  checkSerial();//check serial port for new commands
  //Serial.println("Finished");
  RunTheMotor(); //function to handle the motor

}
void RunTheMotor() {
  if (runallowed == true) {
    //startMicros = micros();

//    Wire.beginTransmission(8); // Start communication with slave device at address 8
//    Wire.write(1);             // Send the value '1' to the slave
//    Wire.endTransmission();
//    
    stepper.enableOutputs();
    //Serial.println("Motor should take one step");
    stepper.run();


//    Wire.beginTransmission(8); // Start communication with slave device at address 8
//    Wire.write(0);             // Send the value '1' to the slave
//    Wire.endTransmission();

    // Encoder Reading
    //    sensorWaarde = readSensor();
    //    delayMicroseconds(1); //Tcs waiting for another read in
    //endMicros = micros();
    //elapsedMicros = endMicros - startMicros;
    //TotalTime = TotalTime + elapsedMicros;
    //Serial.print("Total time is: ");
    //Serial.print(TotalTime);
    //    Serial.print("  TotalTime (microsec):");
    //    Serial.println(TotalTime);
    //Serial.println("Finished");

    // UNCOMMENT HERE
//    if (stepper.distanceToGo() == 0) {
//      Serial.print("Finished; ");
//      endMillis = millis();
//      elapsedMillis = endMillis - startMillis;
//      Serial.print(" Final Time (millis): ");
//      Serial.print(elapsedMillis);
//      sensorWaarde = readSensor();
//    }
  }
  // END UNCOMMENT HERE
  else
  {
    //Serial.println("Finished");
    stepper.disableOutputs();
    return;
  }

  //}
}

void checkSerial() {
  //Serial.println("checking");
  if (Serial.available() > 0) {
    //Serial.println("Finished");
    receivedCommand = Serial.read();
    newData = true;
    if (newData == true) {
      switch (receivedCommand)
      {
        case 'P':

          // UNCOMMENT HERE
//          Serial.print("Started; ");
//          startMillis = millis();
//          sensorWaarde = readSensor();
          // STOP HERE

          receivedSteps = Serial.parseFloat(); //value for the steps
          receivedSpeed = Serial.parseFloat(); //value for the speed
          //Serial.print("Received Steps is: ");
          //Serial.println(receivedSteps);
          //Serial.print("Received Speed is: ");
          //Serial.println(receivedSpeed);
          directionMultiplier = 1;
          RotateRelative();

          break;

        case 'N':

          // UNCOMMENT HERE
//          Serial.print("Started; ");
//          startMillis = millis();
//          sensorWaarde = readSensor();
          // STOP HERE

          receivedSteps = Serial.parseFloat(); //value for the steps
          receivedSpeed = Serial.parseFloat(); //value for the speed
          //Serial.print("Received Steps is: ");
          //Serial.println(receivedSteps);
          //Serial.print("Received Speed is: ");
          //Serial.println(receivedSpeed);
          directionMultiplier = -1;
          RotateRelative();

          break;
      }
    }
    newData = false;
  }
}

void RotateRelative() {
  runallowed = true;
  stepper.setMaxSpeed(receivedSpeed);
  stepper.move(directionMultiplier * receivedSteps);
}
//
//
//unsigned int readSensor() {
//  unsigned int dataOut = 0;
//  digitalWrite(CSn, LOW);
//  delayMicroseconds(1); //Waiting for Tclkfe
//  //Passing 12 times, from 0 to 11
//  for (int x = 0; x < 12; x++) {
//    digitalWrite(CLK, LOW);
//    delayMicroseconds(1); //Tclk/2
//    digitalWrite(CLK, HIGH);
//    delayMicroseconds(1); //Tdo valid, like Tclk/2
//    dataOut = (dataOut << 1) | digitalRead(DO); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
//    //Serial.print(dataOut);
//    AnglePosition = (dataOut * 0.0879);
//    //      Serial.print("Angle (deg):");
//    //      Serial.print(AnglePosition);
//  }
//  digitalWrite(CSn, HIGH); //deselects the encoder from reading
//  Serial.print(" The encoder value is: ");
//  Serial.println(dataOut);
//  return dataOut;
//}
