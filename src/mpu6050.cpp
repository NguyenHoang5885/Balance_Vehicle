#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "mpu6050.h"  
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//-----------------------------------------------------
extern MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
uint8_t FIFOBuffer[64]; // FIFO storage buffer //all
Quaternion q;           // [w, x, y, z]         Quaternion container 
VectorFloat gravity;    // [x, y, z]            Gravity vector 
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector 
extern float Pitch, Roll;

void Get_MPU(){
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
        /* Display Euler angles in degrees */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180/M_PI); //Yaw
        //Serial.print("\t");
        // Serial.print(ypr[1] * 180/M_PI); //Pitch
        // Serial.print("\t");
        // Serial.println(ypr[2] * 180/M_PI); //Row
        Pitch = ypr[1] * 180/M_PI;
        Roll  = ypr[2] * 180/M_PI;
        #endif
    }
}
