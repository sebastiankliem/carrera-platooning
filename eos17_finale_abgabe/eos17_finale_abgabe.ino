#include "Adafruit_VL53L0X.h"
#include<Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// status LEDs
const int LED1_PIN = 4;
const int LED2_PIN = 5;
  
// motor pin
const int MOTOR_PIN = 9;

const int baseSpeed     = 150;
      int currentSpeed  = baseSpeed;
const int minSpeed      = 0;
const int maxSpeed      = 255;

const int baseDistance = 160;

int acceleration  = 10;
int brake         = -30;


int distance = 0;
int lastDistance = 0;
int deviation = 0;
int deviationChange = 0;
   
void setup(){

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // ToF
    Serial.println(F("Initializing I2C devices..."));
    if (!lox.begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
      while(1);
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
}



void loop(){

    // ToF
    VL53L0X_RangingMeasurementData_t measure;
    
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    




    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        Serial.print("Distance (mm): "); Serial.print(measure.RangeMilliMeter); 
        Serial.print(" | currentSpeed: "); Serial.print(currentSpeed);
        Serial.print(" | deviation: "); Serial.print(deviation);
        Serial.print(" | deviationChange: "); Serial.println(deviationChange);

        distance = measure.RangeMilliMeter;
        deviation = distance - baseDistance;
        deviationChange = distance - lastDistance;
        

        if ( deviation > 200 ){
          acceleration = 10;          
        } else if ( deviation > 100 ) {
          acceleration = 5;
        } else if ( deviation > 50 ) {
          acceleration = 1;
        } else if ( deviation = 0 ) {
        } else if ( deviation > -45) {
          if(deviationChange < -25) {
            acceleration = -1;
          }
          
        } else if ( deviation > -90 ) {
          if(deviationChange < -25) {
            acceleration = -5;
          }
        } 

        lastDistance = distance;
        adjustSpeed(acceleration);


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
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print(" | GyZ = "); Serial.println(abs(GyZ));

    int speed = 0;
    if(abs(GyZ) > 15000) {
        speed = 0;
        digitalWrite(LED1_PIN, 1);
        digitalWrite(LED2_PIN, 1);
 
    } else if (abs(GyZ) > 10000) {
        speed = 135;
        digitalWrite(LED1_PIN, 1);
        digitalWrite(LED2_PIN, 0);
    } else if (abs(GyZ) > 5000) {
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
  
  ( currentSpeed > 255 ) ? currentSpeed = maxSpeed : currentSpeed=currentSpeed;
  ( currentSpeed < 0   ) ? currentSpeed = minSpeed : currentSpeed=currentSpeed;
  
}
