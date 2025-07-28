#include <Arduino.h>
#include "control_engine.h"

float Kp = 4, Ki = 0, Kd = 0.8;
float previousAngle = 0, currentAngle   = 0;
float PID_integral  = 0, PID_derivative = 0;
int   PID_output    = 0;
extern float startPoint;

void Control(int pwm){
    pwm = constrain(pwm, -255, 255);
    Serial.println(pwm);
    if(pwm < 30 && pwm >=-30 ){
        pwm = 0;
        analogWrite(EN1, -pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); 
        analogWrite(EN2, -pwm) ; digitalWrite(IN3, LOW ); digitalWrite(IN4, LOW); 
    }     
    else {
        if(pwm < -30){
            analogWrite(EN1, -pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
            analogWrite(EN2, -pwm) ; digitalWrite(IN3, HIGH ); digitalWrite(IN4, LOW); 
        }
        else{
            analogWrite(EN1, pwm) ; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
            analogWrite(EN2, pwm) ; digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
        }
    }
}

void PID(float pid_angle, float dt, float *output){
    currentAngle = pid_angle;
    PID_integral += currentAngle * dt;
    PID_derivative = (currentAngle - previousAngle) / dt;

    *output = Kp * (startPoint - pid_angle) + Ki * PID_integral + Kd * PID_derivative;
    previousAngle = currentAngle;
}