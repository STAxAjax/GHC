#include <Wire.h>
#include <MPU6050.h>
  //nrf
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// Create an MPU6050 object
MPU6050 mpu;

 //buat struct
  typedef struct {
    char a;
    float x;
  } command;
command com;

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// Variables to store raw data and calculated angles
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float angleX, angleY, angleZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;

void setup() {

  Serial.begin(9600);
  Wire.begin();
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

  // Initialize time variables
  previousTime = millis();

 //nrf
    radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  //maju mundur
  pinMode (4, INPUT);
  pinMode (5, INPUT);
}
void loop(){
  // Read raw accelerometer and gyroscope data
  accelX = mpu.getAccelerationX();
  accelY = mpu.getAccelerationY();
  accelZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();

  // Calculate elapsed time
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  // Convert gyroscope raw data to degrees/sec
  float gyroXrate = gyroX / 131.0;
  float gyroYrate = gyroY / 131.0;
  float gyroZrate = gyroZ / 131.0;

  // Calculate angles from gyroscope
  gyroAngleX += gyroXrate * elapsedTime;
  gyroAngleY += gyroYrate * elapsedTime;
  gyroAngleZ += gyroZrate * elapsedTime;

  // Calculate angles from accelerometer
  float accelAngleX = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
  float accelAngleY = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;

  // Combine accelerometer and gyroscope data using a complementary filter
  angleX = 0.96 * (angleX + gyroXrate * elapsedTime) + 0.04 * accelAngleX;
  angleY = 0.96 * (angleY + gyroYrate * elapsedTime) + 0.04 * accelAngleY;
  angleZ = gyroAngleZ; // Z-axis relies only on gyroscope (may drift over time)

  // Print the angles
  Serial.print("Angle X (Pitch): ");
  Serial.print(angleX);
  Serial.print(" | Angle Y (Roll): ");
  Serial.print(angleY);
  Serial.print(" | Angle Z (Yaw): ");
  Serial.println(angleZ);
 
   //button maju 
if (digitalRead (4)==HIGH) {
  if (digitalRead (5)==HIGH) {
    com.a = "k" ;
}else {
  com.a = "f" ;

}}
//button mundur
if (digitalRead (5)==HIGH) {
  if (digitalRead (4)==HIGH) {
    com.a = "k" ;
}else {
  com.a = "b" ;
}}

//nrf
  com.x = angleZ ;
  char senddata[20];
  memcpy(senddata,&com,sizeof(com));
  radio.write(senddata, sizeof(senddata));
}
