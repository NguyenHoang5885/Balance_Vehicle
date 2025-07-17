🦾 Self-Balancing Robot
- This project describes a self-balancing robot system using Arduino, MPU6050, and L298N motor driver. The robot integrates a PID control algorithm to stabilize and control the motor speed.

🔧 Features
- I2C driver implemented for MPU6050 sensor
- Sensor fusion using Kalman Filter and Complementary Filter
- PID controller for real-time balance adjustment
- Smooth movement and reduced slipping

📦 Hardware Used
- Arduino UNO
- MPU6050 (Gyroscope + Accelerometer)
- L298N Motor Driver
- DC Motors + Wheels
- Battery Pack

🔌 Wiring Diagram
- Component	Pin on Arduino
//-----------------------------
      MPU6050    |  UNO
   MPU6050 - VCC |	5V
   MPU6050 - GND |	GND
   MPU6050 - SDA |	SDA
   MPU6050 - SCL |	SCL
//-----------------------------
      L298N      |  UNO
   L298N - IN1	 |  D13
   L298N - IN2	 |  D12
   L298N - IN3   |	D10
   L298N - IN4	 |  D8
   L298N - ENA	 |  D11 (PWM)
   L298N - ENB	 |  D9 (PWM)
   L298N - VCC	 |  12V (Battery)
   L298N - GND	 |  GND
