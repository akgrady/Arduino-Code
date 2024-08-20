#include <Wire.h>

float StartMicros, EndMicros, TimeDiff_sec, Encoder1, Encoder2, Angle, Speed, AngleChange;
unsigned int sensorWaarde = 0;

const int CSn = 2; // Chip select
const int CLK = 3; // Clock signal
const int DO = 8; // Digital Output from the encoder which delivers me a 0 or 1, depending on the bar angle..
int Value;

void setup() {
  Serial.begin(57600);

  pinMode(CSn, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DO, INPUT);

  digitalWrite(CLK, HIGH);
  digitalWrite(CSn, HIGH);
}

void loop() {
  Value = 1;
  StartMicros = millis();
  sensorWaarde = readSensor();
  delay(10);
  Value = 0;
  sensorWaarde = readSensor();
  EndMicros = millis();
  TimeDiff_sec = (EndMicros - StartMicros)/1000;
  AngleChange = (abs(Encoder2-Encoder1)*0.0879);
  Speed = (abs(Encoder2-Encoder1)*0.0879)/TimeDiff_sec;
    Serial.print("Encoder1: ");
    Serial.print(Encoder1);
    Serial.print(" Encoder 2: ");
    Serial.print(Encoder2);
    Serial.print(" AngleChange: ");
    Serial.print(AngleChange);
    Serial.print(" Time_diff: ");
    Serial.print(TimeDiff_sec);
    Serial.print(" instantaneous speed: ");
    Serial.println(Speed);
  
  
}


  unsigned int readSensor() {
  unsigned int dataOut = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(1); //Waiting for Tclkfe
  //Passing 12 times, from 0 to 11
  for (int x = 0; x < 12; x++) {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1); //Tclk/2
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1); //Tdo valid, like Tclk/2
    dataOut = (dataOut << 1) | digitalRead(DO); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
    //Serial.print(dataOut);
    //AnglePosition = (dataOut * 0.0879);
    //      Serial.print("Angle (deg):");
    //      Serial.print(AnglePosition);
  }
  digitalWrite(CSn, HIGH); //deselects the encoder from reading
  //Serial.print(" Encoder: ");
  if (Value == 1){
    Encoder1 = dataOut;
 }
  if (Value == 0){
    Encoder2 = dataOut;
  }

  return dataOut;
}
