//
// Accel4.ino
// Written by Ronan Byrne, adapted from other sources which are referanced 
// where approperiate
// Last updated 30.04/2016
/*
   This code controls a MSP432 which is connected to two IMUs which record both
   accelerometer and gyroscope signals and communicate through the I2C protocol. The data is 
   recorded when a record button is held, this data is saved to the RAM of the MSP.Once the send button is pressed this data is read from
   the SD and sent over the Serial port to a PC which will process the data.
*/
// Include application, user and local libraries
#include <Wire.h>
// Timer library and code adapted from https://github.com/ArduCAM/Energia/tree/master/examples/10.MultiTasking/TimerLibrary
#include "Timer.h"

#define BUFFER_SIZE 5500

Timer myTimer;

// Define variables and constants
boolean buttonState, buttonState2;
const uint8_t ON_BUTTON = 30;
const uint8_t RECORD_BUTTON = 8;
const uint8_t SEND_BUTTON = 6;
const uint8_t LedG = 26;
const uint8_t LedB = 27;
const uint8_t LedR = 28;
const uint8_t LedY = 29;
const int8_t MPU6050=0x68;  // I2C address of the MPU-6050
const int8_t MPU9150 = 0x69; // I2C address of the MPU-9150
int16_t AcX[BUFFER_SIZE],AcY[BUFFER_SIZE],Ac2Y[BUFFER_SIZE],
Ac2Z[BUFFER_SIZE], gX[BUFFER_SIZE];
volatile unsigned Index=0;
volatile unsigned n;
volatile int Tic=0;

// Add setup code
void setup()
{
  Serial.begin(115200); // Set baudrate at max speed(115200)
  pinMode(LedB, OUTPUT);
  pinMode(LedG,OUTPUT);
  pinMode(LedR,OUTPUT);
  pinMode(ON_BUTTON,INPUT);
  pinMode(RECORD_BUTTON,INPUT);
  pinMode(SEND_BUTTON, INPUT);

  Serial.print(Timer_getNumTimers(), DEC);
  Serial.println(" timers");

  Serial.print("myTimer.begin... ");
  myTimer.begin(timerFunction, 1, 1000); // Run timer at 1kHz.
  Serial.println("done");    

  Serial.print("myTimer.start... ");
  myTimer.start();
  Serial.println("done");
  Wire.begin();
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU9150);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-9150)
  Wire.endTransmission(true);
}

// Add loop code
void loop() 
{
  if(digitalRead(ON_BUTTON)==1){
    digitalWrite(LedG, HIGH);
    unsigned count;
    // Check Button States
    buttonState = digitalRead(RECORD_BUTTON);
    buttonState2 = digitalRead(SEND_BUTTON);

    if (buttonState == 1){ // If recorded button is pressed recorded data
      if (Index < BUFFER_SIZE) 
      {
        while (Tic==0);// wait until the timer ticks
        readAccelerometer(); // read from the MPU-6050
        readAccelerometer2(); // read from the MPU_9150
        Index++;
        Tic=0;// reset timer
      }
      else {
        // If index is greater than the buffer size, turn on red LED
        digitalWrite(LedR,HIGH);
      }
    }
    else if (buttonState2 == 1){// if send button is pressed, send data over Serial
      n = Index;
      count = 0;
      digitalWrite(LedY,HIGH);
      Serial.println("B");
      establishContact();// Wait for a response from the PC
      while (count < n)
      {
        if(digitalRead(ON_BUTTON)==0) break;
        // Wait for communication over serial
        if (Serial.available() > 0){
          Serial.print(AcX[count]);
          Serial.print(","); 
          Serial.print(AcY[count]);
          Serial.print(","); 
          Serial.print(Ac2Y[count]);
          Serial.print(","); 
          Serial.print(Ac2Z[count]);
          Serial.print(",");
          Serial.println(gX[count]);
          count++;
          delay(1);
        }
      }
      delay(300);
      Serial.println("Done");// Send "Done" to PC
      delay(1000);
    }
    else{ 
      // Switch all outputs but Green led
      digitalWrite(LedY,LOW);
      digitalWrite(LedB,LOW);
      digitalWrite(LedR,LOW);
    }
  }
  else{
    // Switch out all outputs and reset index
    Index = 0;
    digitalWrite(LedG,LOW);
    digitalWrite(LedB,LOW);
    digitalWrite(LedR,LOW);
    digitalWrite(LedY,LOW);
  }
}

void establishContact() {
  // Send "B" until a response is heard
  while (Serial.available() <= 0) {
    if(digitalRead(ON_BUTTON)==0) break;
    Serial.println("B");
    delay(300);
  }
}

void readAccelerometer() // Can not be called from within an interrupt.
{
  if (Index < BUFFER_SIZE)
  {
    digitalWrite(LedB,HIGH);
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050,4,true);  // request a total of 6 registers
    AcX[Index]=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY[Index]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    digitalWrite(LedB,LOW);
  }
}
void readAccelerometer2() // Can not be called from within an interrupt.
{
  if (Index < BUFFER_SIZE)
  {
    digitalWrite(LedB,HIGH);
    Wire.beginTransmission(MPU9150);
    Wire.write(0x3D);  // starting with register 0x3D (ACCEL_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9150,4,true);  // request a total of 6 registers
    Ac2Y[Index]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)     
    Ac2Z[Index]=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    Wire.write(0x43);  // starting with register 0x3D (ACCEL_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9150,2,true);  // request a total of 6 registers
    gX[Index]=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)     
    //gZ[Index]=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    digitalWrite(LedB,LOW);
  }
}

// Timer Function
void timerFunction()
{
  Tic=1;
}
