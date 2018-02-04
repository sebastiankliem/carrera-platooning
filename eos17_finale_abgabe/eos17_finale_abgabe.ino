#include <PID_v1.h>
#include "Adafruit_VL53L0X.h"
#include<Wire.h>

// VL53L0X ToF sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

// gyroscope
const int MPU_addr = 0x68;
int16_t acX, acY, acZ, tmp, gyX, gyY, gyZ;

// status LEDs
const int LED1_PIN = 4;
const int LED2_PIN = 5;

// motor pin (PWM)
const int MOTOR_PIN = 9;

// speed
const int baseSpeed = 150;
double currentSpeed = baseSpeed;
const int minSpeed = 0;
const int maxSpeed = 255;

// distances
double baseDistance = 160;
double distance = 0; // current distance
int lastDistance = 0; // last measured distance

// PID
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.1, consKd=0.25;
PID myPID(&distance, &currentSpeed, &baseDistance, consKp, consKi, consKd, REVERSE);

void setup() {
  Serial.begin(115200);

  // I2C devices initialization
  initializeDevices();

  // pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // first ToF distance reading for moving average
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    distance = measure.RangeMilliMeter;
  }

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}



void loop() {
  // measure distance
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) { // successful measurement
    int newDistance = measure.RangeMilliMeter;
    distance = ema(newDistance, distance);
    deviation = abs(distance - baseDistance);
    lastDistance = distance;

    // set speed using PID
    if (deviation < 25) {
      // close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else {
       // far from setpoint, use aggressive tuning parameters
       myPID.SetTunings(aggKp, aggKi, aggKd);
    }    
    myPID.Compute();
    
    Serial.print("Distance (mm): "); Serial.print(distance); 
    Serial.print(" | currentSpeed: "); Serial.print(currentSpeed);
    Serial.print(" | deviation: "); Serial.println(deviation);
  } else { 
    Serial.println("out of range");
  }

  // set motor speed
  analogWrite(MOTOR_PIN, currentSpeed);

  /*
    // Gyro
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    acX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    acY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print(" | gyZ = "); Serial.println(abs(gyZ));

    int speed = 0;
    if(abs(gyZ) > 15000) {
      speed = 0;
      digitalWrite(LED1_PIN, 1);
      digitalWrite(LED2_PIN, 1);

    } else if (abs(gyZ) > 10000) {
      speed = 135;
      digitalWrite(LED1_PIN, 1);
      digitalWrite(LED2_PIN, 0);
    } else if (abs(gyZ) > 5000) {
      speed = 155;
      digitalWrite(LED1_PIN, 0);
      digitalWrite(LED2_PIN, 1);

    } else {
      speed = 155;
      digitalWrite(LED1_PIN, 0);
      digitalWrite(LED2_PIN, 0);
    }

    //Serial.println(speed);
    analogWrite(MOTOR_PIN, speed);
  */
}


void initializeDevices() {
  Serial.println("Initializing I2C devices...");
  
  // ToF
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  // Gyro
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.println("I2C devices available");
}

// Exponential moving average
// used for smooth distance mesures of ToF sensor
int ema(int currentVal, int lastVal) {
  float weight = 0.1;
  return (weight * currentVal) + ((1 - weight) * lastVal);
}

