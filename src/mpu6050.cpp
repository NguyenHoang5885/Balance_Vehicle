#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"  
#include "SimpleKalmanFilter.h"

SimpleKalmanFilter Pitch_filter( 2, 10, 0.01);
SimpleKalmanFilter Roll_filter ( 2, 10, 0.01);

int16_t ax, ay ,az, temp, gx, gy, gz;
float pitch_raw, roll_raw;
float gx_offset, gy_offset;
float startPoint = 0;

uint8_t Check_ADDR_MPU(void){
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR,1);
    if(Wire.available() != 0){
        uint8_t ID = Wire.read();
        if(ID == 0x68 || ID == 0X69){
            return ID; 
        }
        else{
            return 0xFF;
        }
    }
    else{ return 0xFF; }
}

void MPU_Init(){
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);   Wire.write(0x00); Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(CONFIG);       Wire.write(0x00); Wire.endTransmission();
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_CONFIG);  Wire.write(0x18); Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_CONFIG); Wire.write(0x18); Wire.endTransmission();
}

void MPU_Getvalue(){
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire. requestFrom(MPU6050_ADDR,14);
    uint8_t data[14];
    for(int i=0; i<14;i++){
        data[i] = Wire.read();
    }
    ax = data[0] <<8 | data[1] ;
    ay = data[2] <<8 | data[3] ;
    az = data[4] <<8 | data[5] ;

    temp= data[6]<<8 | data[7] ;

    gx = data[8] <<8 | data[9] ;
    gy = data[10]<<8 | data[11];
    gz = data[12]<<8 | data[13];  
}

void offset(){
    long sum_gx = 0, sum_gy = 0;
    for(int i=0; i < 500; i++){
        MPU_Getvalue();
        sum_gx += gx;
        sum_gy += gy;
        delay(2);
    }
    gx_offset = (float)sum_gx/500.0;
    gy_offset = (float)sum_gy/500.0;
}

void Convert_Pitch_Raw(void){
    float alpha = 0.955, dt = 0.01, gyro_sensitive = 16.4;
    static float Integral_fgx = 0, Integral_fgy = 0;

    float fax = (float)ax; float fay = (float)ay; float faz = (float)az;
    float fgx = (float)gx - gx_offset; float fgy = (float)gy - gy_offset; 

    Integral_fgx += (fgx / gyro_sensitive) * dt;
    Integral_fgy += (fgy / gyro_sensitive) * dt;

    float Gravity_fax = atan2(fax, sqrt(fay * fay + faz * faz)) * 180.0 / PI;
    float Gravity_fay = atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / PI;

    pitch_raw = alpha * Integral_fgx + (1.0 - alpha) * Gravity_fax;
    roll_raw  = alpha * Integral_fgy + (1.0 - alpha) * Gravity_fay;
}

void MPU_Kalman_filter(float *Pitch_Up, float *Roll_Up){
    *Pitch_Up =  Pitch_filter.updateEstimate(pitch_raw);
    *Roll_Up  =  Roll_filter.updateEstimate (roll_raw);
}