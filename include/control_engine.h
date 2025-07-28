#ifndef CONTROL_ENGINE_H
#define CONTROL_ENGINE_H

#define EN1 11
#define EN2 9
#define IN1 13  
#define IN2 12 
#define IN3 10 
#define IN4 8 

void Control(int pwm);
void PID(float pid_angle, float dt, float *output);

#endif