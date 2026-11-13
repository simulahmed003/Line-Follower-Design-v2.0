#include <Wire.h>
#include <MPU6050.h>

#include <SoftwareSerial.h>       // Library for software-based serial communication

// Create a software serial object using PA10 as RX and PA9 as TX
SoftwareSerial mySerial(PA10, PA9); 

MPU6050 mpu;

void setup() {
   // Initialize software serial port at 115200 baud rate
  mySerial.begin(115200);
  Wire.begin();  // uses PB6(SCL), PB7(SDA) on STM32

  mySerial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    mySerial.println("MPU6050 connection successful!");
  } else {
    mySerial.println("MPU6050 connection failed!");
    while (1);  // halt
  }

  // Optional: Wake up the MPU6050 if in sleep mode
  mpu.setSleepEnabled(false);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Get raw accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  mySerial.print("Accel: ");
  mySerial.print("X="); Serial.print(ax);
  mySerial.print(" Y="); Serial.print(ay);
  mySerial.print(" Z="); Serial.print(az);

  mySerial.print(" | Gyro: ");
  mySerial.print("X="); Serial.print(gx);
  mySerial.print(" Y="); Serial.print(gy);
  mySerial.print(" Z="); Serial.println(gz);

  delay(100);  // adjust sampling rate
}
