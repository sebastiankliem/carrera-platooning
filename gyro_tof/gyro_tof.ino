#include "Adafruit_VL53L0X.h"
#include<Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const int mpu_addr=0x68;  // I2C address of the MPU-6050
int16_t acX, acY, acZ, tmp, gyX, gyY, gyZ;

void setup() {
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
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    
    // power 
    Serial.println(F("devices available\n"));
}

void loop() {
    // ToF
    VL53L0X_RangingMeasurementData_t measure;
    
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
        Serial.println(" out of range ");
    }
    
    // Gyro
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_addr,14,true);  // request a total of 14 registers
    acX=Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    acY=Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acZ=Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    tmp=Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyX=Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyY=Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyZ=Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Serial.print("acX: ");
    Serial.print(acX);
    Serial.print(" | acY: ");
    Serial.print(acY);
    Serial.print(" | acZ: ");
    Serial.println(acZ);
    Serial.print("tmp: ");
    Serial.println(tmp);
    Serial.print("gyX: ");
    Serial.print(gyX);
    Serial.print(" | gyY: ");
    Serial.print(gyY);
    Serial.print(" | gyZ: ");
    Serial.println(gyZ);
}
