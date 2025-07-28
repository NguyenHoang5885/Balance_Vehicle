#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "control_engine.h"

extern int16_t ax, ay ,az, temp, gx, gy, gz;
extern int PID_output;
extern float startPoint;
unsigned long lasttime;
extern float previousAngle, currentAngle;
bool once = true;
float Pitch, Roll;
float original_sumPitch = 0;

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
        if(once){
            once = false;
            for(int i = 0 ; i < 20; i++){
                MPU_Getvalue();
                Convert_Pitch_Raw();
                MPU_Kalman_filter(&Pitch, &Roll);
                original_sumPitch += Pitch;
            }
            startPoint    = original_sumPitch / 20.0;
            previousAngle = startPoint;
            currentAngle  = startPoint;
        }
        offset();
    }
    
}
void T(){
    static unsigned long timeT = 0;
    Serial.println(micros() - timeT);
    timeT = micros();
}
float output = 0;
void loop(){
    MPU_Getvalue();
    Convert_Pitch_Raw();
    MPU_Kalman_filter(&Pitch, &Roll);
    

    // Serial.print(">");
    // Serial.print("Pitch: ");
    // Serial.println(Pitch);
    // Serial.print(",");
    // Serial.print("Roll: ");
    // Serial.println(Roll);
    // delay(1);
    float delta_time = (millis() - lasttime) / 1000.0;
    lasttime = millis();

    PID(Pitch, delta_time, &output);
    //Serial.print(output);
    //Serial.print(" ");
    Control((int)output);
    // T();
}