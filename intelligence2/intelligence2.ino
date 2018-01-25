
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  
// status LEDs
const int LED1_PIN = 4;
const int LED2_PIN = 5;
  
// motor pin
const int MOTOR_PIN = 9;
   
void setup(){
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.begin(19200);
      
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
}

const int FAST = 0.6 * 255;
const int MEDIUM = 0.5 * 255;
const int SLOW = 0.3 * 255;

void loop(){
    
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
      
    //Serial.print("AcX = "); Serial.print(abs(AcX));
    //Serial.print(" | AcY = "); Serial.print(abs(AcY));
    //Serial.print(" | AcZ = "); Serial.println(abs(AcZ));
    Serial.print(" | GyX = "); Serial.print(abs(GyX));
    Serial.print(" | GyY = "); Serial.print(abs(GyY));
    Serial.print(" | GyZ = "); Serial.println(abs(GyZ));
    
    int speed = 0;
    if(abs(GyZ) > 3000) {
        speed = 115;
        digitalWrite(LED1_PIN, 1);
        digitalWrite(LED2_PIN, 1);
    } else if (abs(GyZ) > 2250) {
        speed = 125;
        digitalWrite(LED1_PIN, 1);
        digitalWrite(LED2_PIN, 0);
    } else if (abs(GyZ) > 1500) {
        speed = 132;
        digitalWrite(LED1_PIN, 0);
        digitalWrite(LED2_PIN, 1);
    } else {
        speed = 140;
        digitalWrite(LED1_PIN, 0);
        digitalWrite(LED2_PIN, 0);
    }

    //Serial.println(speed);
    analogWrite(MOTOR_PIN, speed);
    //digitalWrite(LED1_PIN, speed == SLOW   || speed == FAST);
    //digitalWrite(LED2_PIN, speed == MEDIUM || speed == FAST);
            
    delay(20);
}

