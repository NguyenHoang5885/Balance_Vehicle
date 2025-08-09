#include <Arduino.h>
#include "control_engine.h"

float Kp = 15, Ki = 0.00, Kd = 1;
float preError = 0, error = 0 ;
float PID_integral  = 0, PID_derivative = 0;
extern float startPoint;
const float maxIntegral = 100.0; 

void Control(int pwm){
    pwm = constrain(pwm, -255, 255);
    if(pwm <= 30 && pwm >= -30 ){
        pwm = 0;
        analogWrite(EN1, pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        analogWrite(EN2, pwm) ; digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    }
    else if(pwm >= 30 && pwm < 55){
        analogWrite(EN1, pwm) ; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
        analogWrite(EN2, pwm) ; digitalWrite(IN3, LOW) ; digitalWrite(IN4, HIGH); 
    }
    else if(pwm < -30 && pwm >- 55){ 
        analogWrite(EN1, -pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
        analogWrite(EN2, -pwm) ; digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    }
    else{
        if(pwm < -55){
            pwm -= 0;
            analogWrite(EN1, -pwm) ; digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
            analogWrite(EN2, -pwm) ; digitalWrite(IN3, HIGH ); digitalWrite(IN4, LOW); 
        }
        else{
            pwm += 0;
            analogWrite(EN1, pwm) ; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
            analogWrite(EN2, pwm) ; digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
        }
    }
    Serial.println(pwm);
}

void PID(float pid_angle, float dt, float *output){
    error =  startPoint - pid_angle;
    PID_integral  += error * dt;
    PID_integral = constrain(PID_integral, -maxIntegral, maxIntegral);
    if(error <= 1 && error >=-1 ){
        PID_integral = 0;
    }
    PID_derivative = (error - preError) / dt;
    *output = Kp * error + Ki * PID_integral + Kd * PID_derivative;
    preError = error;
}