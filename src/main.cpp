#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "mpu6050.h"
#include "control_engine.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
float startPoint =0 ;
unsigned long lastTime = 0;
float  Pitch = 0, Roll = 0;
float  sumPitch = 0;
double deltaTime = 0 ;
float output = 0;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)

void setup(){
    Serial.begin(115200);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
     #endif

    /*Initialize device*/
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if(mpu.testConnection() == false){
        Serial.println("MPU6050 connection failed");
        while(true);
    }
    else { Serial.println("MPU6050 connection successful"); }
    
    /* Initializate and configure the DMP*/
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */ 
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));   //Turning ON DMP
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
        
        /*Get set point*/
        for(int i = 0; i < 20; i++){
            Get_MPU();
            sumPitch += Pitch;
        }
        startPoint = sumPitch / 20;
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); //Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}
void T(){
    static unsigned long timeT = 0;
    Serial.println(micros() - timeT);
    timeT = micros();
}

void loop(){
    if (!DMPReady) return;
    Get_MPU();

    Serial.print("Main: ");
    Serial.print(Pitch);  //Pitch
    Serial.print(" / ");
    Serial.println(Roll); //Row 

    deltaTime = (micros() - lastTime) / 1000000.0;
    lastTime = micros();
    
    PID(Pitch, deltaTime, &output);
    Control((int)output);
}