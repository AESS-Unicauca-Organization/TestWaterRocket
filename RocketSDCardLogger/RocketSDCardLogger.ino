/*
 * Arduino NANO Rocket Logger  (SDCARD Version)
 * CS=9, SPI+I2C
 */
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"
#include "MPU9250_asukiaaa.h"
#include <SD.h>
#include <ArduinoQueue.h>

#define BMP_INTERVAL 100 // ms

File logFile;
MPU9250_asukiaaa mySensor;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

struct BufferObject {
    char *keyword[2]; // Can be: aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,p
    float value;
};

ArduinoQueue<BufferObject> bufferQueue(100, 1024);

void setup() {
  Wire.begin();
  delay(2000);

  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

  if (!bmp.begin()) {
    digitalWrite(13, HIGH);
  }

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X8,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  if (!SD.begin(9))
  {
    digitalWrite(13, HIGH);
    return;
  }
}

void loop() {
  if (mySensor.accelUpdate() == 0) {
    writeToSD("accX", mySensor.accelX());
    writeToSD("accY", mySensor.accelY());
    writeToSD("accZ", mySensor.accelZ());
  }

  if (mySensor.gyroUpdate() == 0) {
    writeToSD("gyrX", mySensor.gyroX());
    writeToSD("gyrY", mySensor.gyroY());
    writeToSD("gyrZ", mySensor.gyroZ());
  }

  if (mySensor.magUpdate() == 0) {
    writeToSD("magX", mySensor.magX());
    writeToSD("magY", mySensor.magY());
    writeToSD("magZ", mySensor.magZ());
  }
  
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + BMP_INTERVAL) {
          
      sensors_event_t pressure_event;
      bmp_pressure->getEvent(&pressure_event);

      float pressure = (float) pressure_event.pressure;
      writeToSD("prs", pressure);
            
      prev_ms = millis();
  }
}

// Writing to SD can take up to 200ms
void writeToSD(String keyword, float value) {
  // Abrir archivo y escribir valor
  logFile = SD.open("datalog.txt", FILE_WRITE);
      
  if (logFile) { 
      logFile.print((String) millis() + "," + keyword + "," + value + "\n");
 
      logFile.close();
  } else {
    digitalWrite(13, HIGH);
  }
}
