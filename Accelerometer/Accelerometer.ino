// Basic OLED demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  // while (!Serial);
  Serial.println("MPU6050 OLED demo");

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");
  }

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//  Serial.print("Accelerometer ");
//  Serial.print("X: ");
//  Serial.print(a.acceleration.x, 1);
//  Serial.print(" m/s^2, ");
//  Serial.print("Y: ");
//  Serial.print(a.acceleration.y, 1);
//  Serial.print(" m/s^2, ");
//  Serial.print("Z: ");
//  Serial.print(a.acceleration.z, 1);
//  Serial.println(" m/s^2");

  if (a.acceleration.x >= 3) {
    Serial.print("forward\n");
  }
  else if (a.acceleration.x <= -3) {
    Serial.print("backward\n");
  }
  else if (a.acceleration.y >= 3) {
    Serial.print("left\n");
  }
  else if (a.acceleration.y <= -3) {
    Serial.print("right\n");
  }

//  Serial.print("Gyroscope ");
//  Serial.print("X: ");
//  Serial.print(g.gyro.x, 1);
//  Serial.print(" rps, ");
//  Serial.print("Y: ");
//  Serial.print(g.gyro.y, 1);
//  Serial.print(" rps, ");
//  Serial.print("Z: ");
//  Serial.print(g.gyro.z, 1);
//  Serial.println(" rps");

  delay(100);
}
