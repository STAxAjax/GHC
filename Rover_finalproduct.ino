#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// Servo setup
Servo Servo1;
const int servoPin = 6;

// nRF24L01 setup
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

// Define the command struct (packed to avoid padding)
typedef struct {
  char a;
  float x;
} __attribute__((packed)) command;
command com;

void setup() {
  // Initialize Servo
  Servo1.attach(servoPin);

  // Set input pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

  // Start Serial communication
  Serial.begin(9600);

  // Initialize I2C (Wire)
  Wire.begin();

  // Initialize radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  // Check for incoming radio data
  if (radio.available()) {
    radio.read(&com, sizeof(com));
    Serial.print("com.x: ");
    Serial.println(com.x);
    Serial.print("com.a: ");
    Serial.println(com.a);
  }

  // Motor control based on command letter
  if (com.a == 'f') {
    digitalWrite(5, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
  } else if (com.a == 'b') {
    digitalWrite(5, LOW);
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
  } else {
    digitalWrite(5, LOW);
    digitalWrite(3, LOW);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
  }

  // Servo control based on com.x value
  if (15 < com.x && com.x < 30) {
    Servo1.write(120);
  } else if (30 <= com.x && com.x < 90) {
    Servo1.write(145);
  } else if (-30 < com.x && com.x < -15) {
    Servo1.write(80);
  } else if (-90 < com.x && com.x <= -30) {
    Servo1.write(55);
  } else {
    Servo1.write(100);
  }
}
