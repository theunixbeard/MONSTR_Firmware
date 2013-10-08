#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>
#include "QSLib.h"

int led = 13;
QSLib qsLib(&Serial1, (HardwareSerial *)&Serial);
MPUVector3 testVector3;

void setup() {      
  Serial.begin(9600);
  Serial1.begin(9600);  
  for(int i = 0; i < 3; ++i) {
    testVector3[i] = (i+1) * -10.1;
  }  
  pinMode(led, OUTPUT);     
}

void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
  qsLib.sendAcceleration(testVector3);
  float temp = testVector3[0];
  testVector3[0] = testVector3[1];
  testVector3[1] = temp;
}
