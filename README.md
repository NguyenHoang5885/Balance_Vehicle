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
- Battery Pack (BMS integration)

🔌 Wiring Diagram

#### MPU6050 → Arduino UNO
| MPU6050 Pin | Arduino Pin |
|-------------|-------------|
| VCC         | 5V          |
| GND         | GND         |
| SDA         | A4 (SDA)    |
| SCL         | A5 (SCL)    |

#### L298N → Arduino UNO
| L298N Pin | Arduino Pin |
|-----------|-------------|
| IN1       | D13         |
| IN2       | D12         |
| IN3       | D10         |
| IN4       | D8          |
| ENA       | D11 (PWM)   |
| ENB       | D9  (PWM)   |
| VCC       | 12V Battery |
| GND       | GND         |
