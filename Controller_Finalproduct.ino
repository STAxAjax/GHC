
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create an MPU6050 object
MPU6050 mpu;

// Define the command struct and pack it to avoid padding issues
typedef struct {
  char a;
  float x;
} __attribute__((packed)) command;
command com;

// nRF24L01 setup (transmitter)
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// Variables for sensor readings and computed angles
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float angleX, angleY, angleZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050 and set offsets
  mpu.initialize();
  Serial.println("dmp ok");
  mpu.setXGyroOffset(-39);
  mpu.setYGyroOffset(64);
  mpu.setZGyroOffset(35);
  mpu.setXAccelOffset(-3775);
  mpu.setYAccelOffset(-266);
  mpu.setZAccelOffset(1613);

  // Check if the MPU6050 is connected properly
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connection successful!");

  // Initialize timing
  previousTime = millis();

  // Setup the nRF24L01 radio for transmission
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  // Setup control button pins
  pinMode(4, INPUT);
  pinMode(5, INPUT);
}

void loop() {
  // Read raw accelerometer data
  accelX = mpu.getAccelerationX();
  accelY = mpu.getAccelerationY();
  accelZ = mpu.getAccelerationZ();

  // Read raw gyroscope data
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();

  // Calculate elapsed time in seconds
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Convert raw gyroscope data to degrees per second
  float gyroXrate = gyroX / 131.0;
  float gyroYrate = gyroY / 131.0;
  float gyroZrate = gyroZ / 131.0;

  // Integrate gyroscope data to calculate angles
  gyroAngleX += gyroXrate * elapsedTime;
  gyroAngleY += gyroYrate * elapsedTime;
  gyroAngleZ += gyroZrate * elapsedTime;

  // Calculate angles from accelerometer data
  float accelAngleX = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
  float accelAngleY = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;

  // Combine the accelerometer and gyroscope data using a complementary filter
  angleX = 0.96 * (angleX + gyroXrate * elapsedTime) + 0.04 * accelAngleX;
  angleY = 0.96 * (angleY + gyroYrate * elapsedTime) + 0.04 * accelAngleY;
  angleZ = gyroAngleZ;  // Yaw angle relies solely on gyroscope (may drift)

  // Debug: print computed angles
  Serial.print("Angle X (Pitch): ");
  Serial.print(angleX);
  Serial.print(" | Angle Y (Roll): ");
  Serial.print(angleY);
  Serial.print(" | Angle Z (Yaw): ");
  Serial.println(angleZ);

  // Set the command based on button input
  if (digitalRead(4) == HIGH && digitalRead(5) == LOW) {
    com.a = 'f'; // forward
  } else if (digitalRead(5) == HIGH && digitalRead(4) == LOW) {
    com.a = 'b'; // backward
  } else if (digitalRead(4) == HIGH && digitalRead(5) == HIGH) {
    com.a = 'k'; // stop or neutral command
  } else {
    com.a = 'k';
  }

  // Assign the computed angle (for example, using angleX)
  com.x = angleX;

  // Transmit the command struct via nRF24L01
  radio.write(&com, sizeof(com));
}
