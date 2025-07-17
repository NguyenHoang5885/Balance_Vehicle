#include <Arduino.h>
#include "control_engine.h"

float Kp = 0.51, Ki = 0.00000000101 , Kd = 0.0;
float Angle_previous = 0 , Angle_currently ;
float PID_integral = 0, PID_derivative = 0;
int PID_output = 0;
extern float startPoint;
void Control(int pwm){
    pwm = constrain(pwm, -85, 85);
    if(pwm > 0 && pwm < 75){
        pwm = 75;
    }     
    if(pwm < 0 && pwm > -75){
        pwm = -75;
    }
    if(pwm < 0){
        analogWrite(EN1, abs(pwm)) ; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
        analogWrite(EN2, abs(pwm)) ; digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
    }
    else{
        analogWrite(EN1, pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
        analogWrite(EN2, pwm) ; digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    }
}

void PID(float pid_angle, float dt){
    Angle_currently = pid_angle;
    PID_integral += Angle_currently * dt;
    PID_derivative = (Angle_currently - Angle_previous) / dt;

    PID_output = Kp * (startPoint - pid_angle) + Ki * PID_integral + Kd * PID_derivative;
    Angle_previous = Angle_currently;
    Serial.print(startPoint);
    Serial.print(" ");
    Serial.println(pid_angle);
}