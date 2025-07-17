#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR 0x68

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B

#define CONFIG       0X1A
#define GYRO_CONFIG  0X1B
#define ACCEL_CONFIG 0X1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

uint8_t Check_ADDR_MPU(void);
void MPU_Init(void);
void MPU_Getvalue();

void offset();

void Convert_Pitch_Raw(void);
void MPU_Kalman_filter(float *Pitch_Up, float *Roll_Up);

#endif