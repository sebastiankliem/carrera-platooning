#include <PID_v1.h>
#include "Adafruit_VL53L0X.h"
#include<Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t acX, acY, acZ, tmp, gyX, gyY, gyZ;

// status LEDs
const int LED1_PIN = 4;
const int LED2_PIN = 5;

// motor pin
const int MOTOR_PIN = 9;

const int baseSpeed     = 150;
double currentSpeed  = baseSpeed;
const int minSpeed      = 0;
const int maxSpeed      = 255;

double baseDistance = 160;

int acceleration  = 10;
int brake         = -30;


double distance = 0;
int lastDistance = 0;
int deviation = 0;
int deviationChange = 0;

// ToF measure
VL53L0X_RangingMeasurementData_t measure;

// PID
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.1, consKd=0.25;
PID myPID(&distance, &currentSpeed, &baseDistance, consKp, consKi, consKd, REVERSE);

void setup() {

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // ToF
  Serial.println(F("Initializing I2C devices..."));
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  // Gyro
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // power
  Serial.println(F("devices available\n"));

  // first ToF distance reading for moving average
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    distance = measure.RangeMilliMeter;
  }

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}



void loop() {
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    int newDistance = measure.RangeMilliMeter;
    distance = ema(newDistance, distance);
    deviation = abs(distance - baseDistance);
    
    if (deviation < 25)
    {  //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
       //we're far from setpoint, use aggressive tuning parameters
       myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    
    myPID.Compute();

    
    Serial.print("Distance (mm): "); Serial.print(distance); 
    Serial.print(" | currentSpeed: "); Serial.println(currentSpeed);
//     Serial.print(" | deviation: "); Serial.print(deviation);
//     Serial.print(" | deviationChange: "); Serial.println(deviationChange);

/*
    if ( deviation > 200 ) {
      acceleration = 10;
    } else if ( deviation > 100 ) {
      acceleration = 5;
    } else if ( deviation > 50 ) {
      acceleration = 1;
    } else if ( deviation = 0 ) {
    } else if ( deviation > -45) {
      if (deviationChange < -25) {
        acceleration = -1;
      }

    } else if ( deviation > -90 ) {
      if (deviationChange < -25) {
        acceleration = -5;
      }
    }
*/
    lastDistance = distance;
    //‚adjustSpeed(acceleration);
    
    /*
      if(measure.RangeMilliMeter < (baseDistance) ) {
      adjustSpeed(brake);
      } else {
        adjustSpeed(acceleration);
      }
    */
  } else {
    Serial.println(" out of range ");
  }
  

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


void adjustSpeed(int adjustment) {
  currentSpeed += adjustment;

  ( currentSpeed > 255 ) ? currentSpeed = maxSpeed : currentSpeed = currentSpeed;
  ( currentSpeed < 0   ) ? currentSpeed = minSpeed : currentSpeed = currentSpeed;

}

// Exponential moving average
int ema(int currentVal, int lastVal) {
  float weight = 0.1;
  return (weight * currentVal) + ((1 - weight) * lastVal);
}

