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

//#define DEMO_DEBUG 1

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                  // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                  // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                 // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                 // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   5

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

MPUQuaternion gravity;  // this is our earth frame gravity vector                 

int counter = 0;
int counter_speed = 50;
#define MY_SENSOR_RANGE2 32767.0
#define  DEVICE_TO_CALIBRATE    0
#define MAX_G_FLOAT MAX_G * 1.0
#define SHANGWEN

QSLib qsLib(&Serial1, (HardwareSerial *)&Serial);

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Serial1.begin(9600);
  Serial.println("Accel9150 starting");
  Wire.begin(); 
  //mpu_set_accel_fsr(8);
  

  // Setup calibration data manually due to Teensy 3 EEPROM erase on program bug
  CALLIB_DATA calData;  
  calData.accelValid = true;
  calData.accelMinX = -16434; // mix/max may be swapped... mpulib code reverses rawaAccelX sign...
  calData.accelMaxX = 16820;
  calData.accelMinY = -16780;                              
  calData.accelMaxY = 16510;
  calData.accelMinZ = -18014;                             
  calData.accelMaxZ = 15294;
#ifdef SHANGWEN
  calData.magValid = true;
  calData.magMinX = -150; 
  calData.magMaxX = 146;
  calData.magMinY = -115;                              
  calData.magMaxY = 176;
  calData.magMinZ = -69;                             
  calData.magMaxZ = 241;
#endif
  calLibWrite(DEVICE_TO_CALIBRATE, &calData);
  
  
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);     // start the MPU
}

void loop()
{
  MPUVector3 gravity;  
  MPUVector3 result;
  //////////
  if(MPU.read()) {
#ifdef SHANGWEN
   MPUQuaternionConjugate(MPU.m_fusedQuaternion, fusedConjugate); 
   //  rotate the acceleration vector into the earth frame
      
   MPUQuaternionMultiply(accel, fusedConjugate, qTemp);
   MPUQuaternionMultiply(MPU.m_fusedQuaternion, qTemp, ratated_accel);
   
   eFrame_accel =  -(ratated_accel[QUAT_Z]-MY_SENSOR_RANGE2 / MAX_G_FLOAT);
   
       // Begin velocity calculation//
   if (eFrame_accel > 300) // start to calculate velocity
   {
     velocity += (eFrame_accel/MY_SENSOR_RANGE2*MAX_G_FLOAT*9.8*0.05);
   }   
   else
   {
     if (velocity > 0.5)
     {
       reps++;
       Serial.print("Velocity: ");
       Serial.print(velocity);
       Serial.print("m/s\n");
       Serial.println("Reps: ");
       Serial.print(reps);
       Serial.println("");
     }     
     velocity = 0;
   }   
    //approximately 50 ms per loop!!!!!!!!!!!!!
#else
    gravity[VEC3_X] = (2 * (MPU.m_fusedQuaternion[QUAT_X] * MPU.m_fusedQuaternion[QUAT_Z] 
                          - MPU.m_fusedQuaternion[QUAT_W] * MPU.m_fusedQuaternion[QUAT_Y])) 
                          * (MY_SENSOR_RANGE2 / MAX_G_FLOAT);
    gravity[VEC3_Y] = (2 * (MPU.m_fusedQuaternion[QUAT_W] * MPU.m_fusedQuaternion[QUAT_X] 
                          + MPU.m_fusedQuaternion[QUAT_Y] * MPU.m_fusedQuaternion[QUAT_Z]))
                          * (MY_SENSOR_RANGE2 / MAX_G_FLOAT);
    gravity[VEC3_Z] = (MPU.m_fusedQuaternion[QUAT_W] * MPU.m_fusedQuaternion[QUAT_W] 
                      - MPU.m_fusedQuaternion[QUAT_X] * MPU.m_fusedQuaternion[QUAT_X] 
                      - MPU.m_fusedQuaternion[QUAT_Y] * MPU.m_fusedQuaternion[QUAT_Y] 
                      + MPU.m_fusedQuaternion[QUAT_Z] * MPU.m_fusedQuaternion[QUAT_Z])
                      * (MY_SENSOR_RANGE2 / MAX_G_FLOAT);    

    result[VEC3_X] = -(MPU.m_calAccel[VEC3_X] - gravity[VEC3_X]);
    result[VEC3_Y] = -(MPU.m_calAccel[VEC3_Y] - gravity[VEC3_Y]);
    result[VEC3_Z] = -(MPU.m_calAccel[VEC3_Z] - gravity[VEC3_Z]);
 #endif   
    
    //qsLib.sendAcceleration(result);
    
    counter++;
    if (counter % counter_speed == 0) {
      counter = 0;
      //MPU.printVector(result);      // print the residual accelerations
      qsLib.sendAcceleration(result);
      #ifdef DEMO_DEBUG
        Serial.print("  Cal Accel: ");
        MPU.printVector(MPU.m_calAccel);                               // print the residual accelerations
        Serial.print("\n  Raw Accel: ");
        MPU.printVector(MPU.m_rawAccel);  
        Serial.print("\n  Gravity:   ");
        MPU.printVector(gravity); 
        Serial.print("\n  fsr:   ");
        unsigned char fsr;
        mpu_get_accel_fsr(&fsr);
        Serial.print((float)fsr);
        Serial.print("\n  1G equals:   ");
        Serial.print(sqrt(sq(MPU.m_rawAccel[VEC3_X]) + sq(MPU.m_rawAccel[VEC3_Y]) + sq(MPU.m_rawAccel[VEC3_Z])));    
        Serial.print("\n  accel_scalar:   ");   
        Serial.println(g_accel_scalar);
      #endif
      Serial.println("");
    }
  }
}
