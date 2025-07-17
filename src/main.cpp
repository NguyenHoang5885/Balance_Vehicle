#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "control_engine.h"

extern int16_t ax, ay ,az, temp, gx, gy, gz;
extern int PID_output;
extern float startPoint;
unsigned long lasttime;
extern float Angle_previous;
extern float Angle_currently;
bool once = true;

void setup(){
    Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    lasttime = millis();

    if(Check_ADDR_MPU() == 0xFF){
        Serial.println("FALSE");
    }
    else{
        Serial.println("TRUE");
        MPU_Init();
        offset();
    }
    
}

void loop(){
    float Pitch, Roll;
    MPU_Getvalue();
    Convert_Pitch_Raw();
    MPU_Kalman_filter(&Pitch, &Roll);
    if(once){
        once = false;
        startPoint = Pitch;
        Angle_previous = Pitch;
        Angle_currently = Pitch;
    }

    // Serial.print(">");
    // Serial.print("Pitch: ");
    // Serial.print(Pitch);
    // Serial.print(",");
    // Serial.print("Roll: ");
    // Serial.println(Roll);
    //delay(2);
    float delta_time = (millis() - lasttime) / 1000.0;
    lasttime = millis();

    PID(Pitch, delta_time);
    Control(PID_output);
}